import argparse
import csv
import queue
import struct
import sys
import threading
import time

import serial

SYNC = b"\xAA\x55"

# packet types from the OBC
PKT_TELEMETRY = 0x01
PKT_ACK = 0x02
PKT_NACK = 0x03
PKT_LOG_ENTRY = 0x04

# commands to the OBC
CMD_PING = 0x10
CMD_GOTO_SAFE = 0x11
CMD_RECOVER = 0x12
CMD_GET_LOG = 0x13
CMD_CLEAR_LOG = 0x14
CMD_REBOOT = 0x15

STATES = ["BOOT", "SELF_TEST", "NOMINAL", "SAFE_MODE"]

FAULTS = {
    0x00: "NONE",
    0x01: "WDG_RESET",
    0x02: "CLOCK",
    0x10: "IMU1_COMM",
    0x11: "IMU2_COMM",
    0x12: "IMU_MISMATCH",
    0x20: "TEMP_COMM",
    0x21: "TEMP_RANGE",
    0x30: "PWR_COMM",
    0x31: "PWR_VOLTAGE",
    0x32: "PWR_CURRENT",
    0x40: "FLASH_COMM",
    0x50: "GROUND_CMD",
}

NACK_REASONS = {
    1: "bad crc",
    2: "unknown command",
    3: "wrong state",
    4: "bad length",
    5: "no such log entry",
    6: "flash error",
}

FLAG_NAMES = [(0x01, "IMU1"), (0x02, "IMU2"), (0x04, "TEMP"),
              (0x08, "PWR"), (0x10, "FLASH"), (0x20, "CLK")]

# telemetry payload: time, state, fault, flags, imu1, imu2, temp, bus mV, current, log count
TLM = struct.Struct("<IBBB6h6hhHhH")
# flash log record: magic, time, code, state, flags, detail, crc
LOG = struct.Struct("<IIHBBHH")
LOG_MAGIC = 0xFA17C0DE

ACCEL_LSB = 8192.0   # per g
GYRO_LSB = 65.5      # per dps


def crc16(data, crc=0xFFFF):
    # CRC-16/CCITT-FALSE, same as the firmware
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(ptype, seq, payload=b""):
    body = bytes([ptype, seq & 0xFF, len(payload)]) + payload
    return SYNC + body + crc16(body).to_bytes(2, "big")


class Parser:
    # turns a raw byte stream into (type, seq, payload) frames
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                self.buf = self.buf[-1:]      # keep a possible lone 0xAA
                break
            del self.buf[:i]
            if len(self.buf) < 5:
                break
            length = self.buf[4]
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            rx_crc = (self.buf[5 + length] << 8) | self.buf[6 + length]
            if crc16(body) == rx_crc:
                out.append((body[0], body[1], body[3:]))
                del self.buf[:total]
            else:
                self.crc_errors += 1
                del self.buf[:2]              # bad frame, look for the next sync
        return out


def flag_text(flags):
    ok = [n for bit, n in FLAG_NAMES if flags & bit]
    return " ".join(ok) if ok else "-"


def decode_telemetry(payload):
    v = TLM.unpack(payload)
    return {
        "time_ms": v[0],
        "state": v[1],
        "fault": v[2],
        "flags": v[3],
        "imu1": v[4:10],
        "imu2": v[10:16],
        "temp_c": v[16] / 100.0,
        "bus_mv": v[17],
        "current_ma": v[18],
        "log_count": v[19],
    }


def accel_g(imu):
    return tuple(x / ACCEL_LSB for x in imu[:3])


def gyro_dps(imu):
    return tuple(x / GYRO_LSB for x in imu[3:])


def state_name(s):
    return STATES[s] if s < len(STATES) else "?%d" % s


def fault_name(f):
    return FAULTS.get(f, "0x%02X" % f)


class Station:
    def __init__(self, port, baud, csv_path=None):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.parser = Parser()
        self.replies = queue.Queue()       # ack / nack / log frames
        self.lock = threading.Lock()
        self.seq = 0
        self.running = True
        self.watch = True
        self.last = None                   # last telemetry packet
        self.last_seq = None
        self.dropped = 0
        self.count = 0
        self.prev_state = None
        self.prev_fault = None

        self.csv_file = None
        self.csv = None
        if csv_path:
            self.csv_file = open(csv_path, "w", newline="")
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(["host_time", "time_ms", "state", "fault", "flags",
                               "a1x_g", "a1y_g", "a1z_g", "g1x_dps", "g1y_dps", "g1z_dps",
                               "a2x_g", "a2y_g", "a2z_g", "g2x_dps", "g2y_dps", "g2z_dps",
                               "temp_c", "bus_mv", "current_ma", "log_count"])

    # print a line of output without breaking the status line
    def say(self, text):
        with self.lock:
            print("\r\033[K" + text, flush=True)

    # background thread: read bytes, parse frames
    def reader(self):
        while self.running:
            try:
                data = self.ser.read(256)
            except serial.SerialException:
                self.say("serial port lost")
                self.running = False
                return
            if not data:
                continue
            for ptype, seq, payload in self.parser.feed(data):
                if ptype == PKT_TELEMETRY and len(payload) == TLM.size:
                    self.on_telemetry(seq, decode_telemetry(payload))
                elif ptype in (PKT_ACK, PKT_NACK, PKT_LOG_ENTRY):
                    self.replies.put((ptype, seq, payload))

    def on_telemetry(self, seq, t):
        # count missing packets using the sequence number
        if self.last_seq is not None:
            gap = (seq - self.last_seq) & 0xFF
            if gap > 1:
                self.dropped += gap - 1
        self.last_seq = seq
        self.last = t
        self.count += 1

        # report state and fault changes
        if t["state"] != self.prev_state:
            self.say("[%8.1fs] state -> %s" % (t["time_ms"] / 1000, state_name(t["state"])))
            self.prev_state = t["state"]
        if t["fault"] != self.prev_fault:
            if t["fault"] != 0:
                self.say("[%8.1fs] FAULT %s" % (t["time_ms"] / 1000, fault_name(t["fault"])))
            self.prev_fault = t["fault"]

        if self.csv:
            row = [time.time(), t["time_ms"], state_name(t["state"]), fault_name(t["fault"]), t["flags"]]
            row += [round(x, 4) for x in accel_g(t["imu1"]) + gyro_dps(t["imu1"])]
            row += [round(x, 4) for x in accel_g(t["imu2"]) + gyro_dps(t["imu2"])]
            row += [t["temp_c"], t["bus_mv"], t["current_ma"], t["log_count"]]
            self.csv.writerow(row)

        if self.watch:
            a1 = accel_g(t["imu1"])
            a2 = accel_g(t["imu2"])
            line = ("%7.1fs %-9s %-12s [%s] T=%.2fC V=%.3fV I=%dmA "
                    "A1=(%+.2f %+.2f %+.2f)g A2=(%+.2f %+.2f %+.2f)g logs=%d drop=%d crc=%d" % (
                        t["time_ms"] / 1000, state_name(t["state"]), fault_name(t["fault"]),
                        flag_text(t["flags"]), t["temp_c"], t["bus_mv"] / 1000.0, t["current_ma"],
                        a1[0], a1[1], a1[2], a2[0], a2[1], a2[2],
                        t["log_count"], self.dropped, self.parser.crc_errors))
            with self.lock:
                print("\r\033[K" + line, end="", flush=True)

    # send a command and wait for ack / nack, retry on loss
    def command(self, cmd, payload=b"", retries=3, timeout=1.0):
        for _ in range(retries):
            self.seq = (self.seq + 1) & 0xFF
            seq = self.seq
            while not self.replies.empty():
                self.replies.get_nowait()          # drop old replies
            self.ser.write(build_frame(cmd, seq, payload))

            end = time.time() + timeout
            while time.time() < end:
                try:
                    ptype, rseq, rpayload = self.replies.get(timeout=0.05)
                except queue.Empty:
                    continue
                if rseq != seq:
                    continue
                if ptype == PKT_NACK and len(rpayload) >= 2 and rpayload[1] == 1:
                    break                           # uplink was corrupted, resend
                return ptype, rpayload
        return None

    def close(self):
        self.running = False
        time.sleep(0.1)
        self.ser.close()
        if self.csv_file:
            self.csv_file.close()


def show_result(res):
    if res is None:
        print("no reply (timeout)")
    elif res[0] == PKT_ACK:
        print("ACK")
    elif res[0] == PKT_NACK:
        reason = res[1][1] if len(res[1]) > 1 else 0
        print("NACK: %s" % NACK_REASONS.get(reason, reason))
    else:
        print("reply type 0x%02X" % res[0])


def get_log_entry(st, idx):
    res = st.command(CMD_GET_LOG, struct.pack("<H", idx))
    if res is None or res[0] != PKT_LOG_ENTRY or len(res[1]) != 2 + LOG.size:
        return res, None
    raw = res[1][2:]
    magic, t_ms, code, state, flags, detail, crc = LOG.unpack(raw)
    ok = magic == LOG_MAGIC and crc == crc16(raw[:-2])
    return res, (t_ms, code, state, flags, detail, ok)


def print_entry(idx, e):
    t_ms, code, state, flags, detail, ok = e
    print("#%03d  t=%8.1fs  %-12s  state=%-9s  flags=[%s]  detail=%d%s" % (
        idx, t_ms / 1000, fault_name(code), state_name(state), flag_text(flags), detail,
        "" if ok else "  (BAD CRC)"))


HELP = """commands:
  ping            check the link
  safe            force safe mode
  recover         leave safe mode (re-runs self test)
  log N           read fault log entry N
  logs            read the whole fault log
  clear           erase the fault log
  reboot          reboot the OBC
  imu             show full imu data from the last packet
  watch           toggle live telemetry line
  help / quit"""


def shell(st):
    print(HELP)
    while st.running:
        try:
            line = input("\robc> ").strip().lower().split()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        cmd, args = line[0], line[1:]

        if cmd in ("quit", "exit", "q"):
            break
        elif cmd == "help":
            print(HELP)
        elif cmd == "watch":
            st.watch = not st.watch
            print("live line", "on" if st.watch else "off")
        elif cmd == "ping":
            show_result(st.command(CMD_PING))
        elif cmd == "safe":
            show_result(st.command(CMD_GOTO_SAFE))
        elif cmd == "recover":
            show_result(st.command(CMD_RECOVER))
        elif cmd == "clear":
            show_result(st.command(CMD_CLEAR_LOG))
        elif cmd == "reboot":
            show_result(st.command(CMD_REBOOT))
        elif cmd == "log":
            if not args or not args[0].isdigit():
                print("usage: log N")
                continue
            res, e = get_log_entry(st, int(args[0]))
            if e:
                print_entry(int(args[0]), e)
            else:
                show_result(res)
        elif cmd == "logs":
            if st.last is None:
                print("no telemetry yet")
                continue
            n = st.last["log_count"]
            print("%d entries" % n)
            for i in range(n):
                res, e = get_log_entry(st, i)
                if e:
                    print_entry(i, e)
                else:
                    print("#%03d  unreadable" % i)
        elif cmd == "imu":
            t = st.last
            if t is None:
                print("no telemetry yet")
                continue
            for name in ("imu1", "imu2"):
                a, g = accel_g(t[name]), gyro_dps(t[name])
                print("%s accel (%+.3f %+.3f %+.3f) g   gyro (%+.2f %+.2f %+.2f) dps" % (
                    name.upper(), a[0], a[1], a[2], g[0], g[1], g[2]))
        else:
            print("unknown command, type help")


def main():
    ap = argparse.ArgumentParser(description="Satellite OBC ground station")
    ap.add_argument("port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", help="save telemetry to this csv file")
    args = ap.parse_args()

    try:
        st = Station(args.port, args.baud, args.csv)
    except serial.SerialException as e:
        print("could not open port:", e)
        sys.exit(1)

    threading.Thread(target=st.reader, daemon=True).start()
    try:
        shell(st)
    finally:
        st.close()
        print("\nclosed")


if __name__ == "__main__":
    main()
