import argparse
import json
import os
import select
import shutil
import socket
import struct
import sys
import threading
import time
import zlib

try:
    import termios
    import tty
    _HAS_TERMIOS = True
except ImportError:  # pragma: no cover - non-POSIX
    _HAS_TERMIOS = False



# Settings

SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.json")
SETTINGS_POLL_INTERVAL = 0.5    # seconds between mtime checks
CALIBRATION_SECONDS = 2.0

DEFAULT_SETTINGS = {
    "_help": {
        "accel_map": "DSU accel output. 'x/y/z' = accel axes, 'gx/gy/gz' = pull from gyro, '0' = zero. Prefix '-' to negate.",
        "gyro_map": "DSU gyro output. 'x/y/z' = gyro axes, 'ax/ay/az' = pull from accel, '0' = zero. Prefix '-' to negate.",
        "accel_scale": "Multiply all accel output values (1.0 = no change).",
        "gyro_scale": "Multiply all gyro output values (1.0 = no change).",
        "gyro_deadzone": "deg/s below which gyro output is clamped to zero, applied after bias calibration.",
    },
    "accel_map": {"x": "0", "y": "0", "z": "0"},
    "gyro_map": {"x": "gx", "y": "-gz", "z": "gy"},
    "accel_scale": 1.0,
    "gyro_scale": 1.0,
    "gyro_deadzone": 0.4,
}

_settings = DEFAULT_SETTINGS
_settings_mtime = 0.0
_settings_lock = threading.Lock()


def write_default_settings():
    with open(SETTINGS_FILE, "w") as f:
        json.dump(DEFAULT_SETTINGS, f, indent=2)
    print(f"[config] Created {SETTINGS_FILE}")


def load_settings(announce=True):
    """Read settings.json. Returns True if it changed."""
    global _settings, _settings_mtime
    try:
        mtime = os.path.getmtime(SETTINGS_FILE)
        if mtime <= _settings_mtime:
            return False
        with open(SETTINGS_FILE) as f:
            cfg = json.load(f)
        with _settings_lock:
            _settings = cfg
            _settings_mtime = mtime
        if announce:
            _log("[config] settings.json reloaded")
        return True
    except Exception as e:
        _log(f"[config] failed to load settings.json: {e}")
        return False


def get_settings():
    with _settings_lock:
        return _settings


# Axis mapping

def apply_map(primary, raw_a, raw_g, mapping, scale):
    combined = {
        "ax": raw_a["x"], "ay": raw_a["y"], "az": raw_a["z"],
        "gx": raw_g["x"], "gy": raw_g["y"], "gz": raw_g["z"],
    }

    def resolve(src):
        src = str(src)
        neg = src.startswith("-")
        key = src.lstrip("-")
        if key == "0":
            return 0.0
        v = primary.get(key, 0.0) if key in ("x", "y", "z") else combined.get(key, 0.0)
        return -v if neg else v

    return tuple(resolve(mapping.get(k, k)) * scale for k in ("x", "y", "z"))


# Sensor state

_state_lock = threading.Lock()
_state = {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}
_gyro_bias = {"gx": 0.0, "gy": 0.0, "gz": 0.0}

_stop = threading.Event()
_debug = False
_last_status = None
_packets_sent = 0
_rate_window_start = time.monotonic()
_rate_hz = 0.0


def _log(msg):
    """Print above the live display without leaving fragments behind."""
    global _last_status
    _clear_debug_block()
    _last_status = None  # force the status line to be redrawn underneath
    sys.stdout.write("\r\033[K" + msg + "\n")
    sys.stdout.flush()


def calibrate_gyro(duration=CALIBRATION_SECONDS):
    """Average the gyro at rest to estimate its zero-rate bias."""
    global _gyro_bias
    _log(f"[calib]  Hold still for {duration:.0f}s ...")

    sums = {"gx": 0.0, "gy": 0.0, "gz": 0.0}
    n = 0
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and not _stop.is_set():
        with _state_lock:
            for k in sums:
                sums[k] += _state[k]
        n += 1
        time.sleep(0.01)

    if n:
        _gyro_bias = {k: v / n for k, v in sums.items()}
        _log("[calib]  bias = ({:+.3f}, {:+.3f}, {:+.3f}) deg/s".format(
            _gyro_bias["gx"], _gyro_bias["gy"], _gyro_bias["gz"]))


def start_sensor():
    try:
        from macimu import IMU
    except ImportError:
        raise SystemExit(
            "ERROR: macimu is not installed.\n"
            "       Install it with:  uv sync"
        )

    if not IMU.available():
        raise SystemExit(
            "ERROR: No IMU sensor found.\n"
            "       This tool needs an Apple Silicon MacBook with a built-in IMU."
        )

    def on_accel(s):
        with _state_lock:
            _state["ax"], _state["ay"], _state["az"] = s.x, s.y, s.z

    def on_gyro(s):
        with _state_lock:
            _state["gx"], _state["gy"], _state["gz"] = s.x, s.y, s.z
        emit_packet()

    imu = IMU(accel=True, gyro=True, sample_rate=100)
    imu.start()
    imu.on_accel(on_accel)
    imu.on_gyro(on_gyro)
    print("[sensor] MacBook IMU opened (accel + gyro at 100 Hz).")
    return imu


# DSU / CemuHook protocol

DSU_HOST = "127.0.0.1"
DSU_PORT = 26760
SERVER_ID = 0xAABBCCDD
PROTO_VER = 1001

_sock = None
_clients = {}
_clients_lock = threading.Lock()
_pkt_num = 0
_pkt_lock = threading.Lock()


def _crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF


def _make_packet(msg_type, payload):
    data_len = 4 + len(payload)
    packet = bytearray()
    packet += b"DSUS"
    packet += struct.pack("<H", PROTO_VER)
    packet += struct.pack("<H", data_len)
    packet += b"\x00\x00\x00\x00"
    packet += struct.pack("<I", SERVER_ID)
    packet += struct.pack("<I", msg_type)
    packet += payload
    crc = _crc32(bytes(packet))
    packet[8:12] = struct.pack("<I", crc)
    return bytes(packet)


def _controller_header(slot, connected=True):
    state = 2 if connected else 0
    return (
        struct.pack("<BB", slot, state)
        + struct.pack("<BB", 2, 1)
        + b"\x00" * 6
        + struct.pack("<B", 0xEF if connected else 0x00)
    )


def _send_info(sock, addr, slot):
    payload = _controller_header(slot, connected=(slot == 0)) + b"\x00"
    sock.sendto(_make_packet(0x100001, payload), addr)


_TOUCH_INACTIVE = struct.pack("<BBHH", 0, 0, 0, 0)


def _build_data_packet(slot, pkt_num, ts_us, ax, ay, az, gx, gy, gz):
    payload = (
        _controller_header(slot)
        + struct.pack("<B", 1)
        + struct.pack("<I", pkt_num)
        + struct.pack("<BB", 0, 0)
        + struct.pack("<BB", 0, 0)
        + struct.pack("<BBBB", 128, 128, 128, 128)
        + bytes(12)
        + _TOUCH_INACTIVE
        + _TOUCH_INACTIVE
        + struct.pack("<Q", ts_us)
        + struct.pack("<fff", ax, ay, az)
        + struct.pack("<fff", gx, gy, gz)
    )
    assert len(payload) == 80
    return _make_packet(0x100002, payload)


def current_output():
    """Mapped values as they would be sent, independent of whether anyone
    is listening. Returns (ax, ay, az, gx, gy, gz)."""
    cfg = get_settings()
    deadzone = cfg.get("gyro_deadzone", 0.0)

    with _state_lock:
        raw_a = {"x": _state["ax"], "y": _state["ay"], "z": _state["az"]}
        raw_g = {
            "x": _state["gx"] - _gyro_bias["gx"],
            "y": _state["gy"] - _gyro_bias["gy"],
            "z": _state["gz"] - _gyro_bias["gz"],
        }

    if deadzone:
        for k in raw_g:
            if abs(raw_g[k]) < deadzone:
                raw_g[k] = 0.0

    accel = apply_map(raw_a, raw_a, raw_g,
                      cfg.get("accel_map", {}), cfg.get("accel_scale", 1.0))
    gyro = apply_map(raw_g, raw_a, raw_g,
                     cfg.get("gyro_map", {}), cfg.get("gyro_scale", 1.0))
    return accel + gyro


def raw_sensors():
    """Uncorrected sensor values, for the debug view."""
    with _state_lock:
        return dict(_state)


def emit_packet():
    """Called from the gyro callback: one sensor sample -> one DSU packet."""
    global _pkt_num, _packets_sent

    if _sock is None or _stop.is_set():
        return

    with _clients_lock:
        targets = list(_clients.keys())
    if not targets:
        return

    ax, ay, az, gx, gy, gz = current_output()
    ts_us = time.monotonic_ns() // 1000

    with _pkt_lock:
        num = _pkt_num
        _pkt_num = (_pkt_num + 1) & 0xFFFFFFFF

    pkt = _build_data_packet(0, num, ts_us, ax, ay, az, gx, gy, gz)

    for addr in targets:
        try:
            _sock.sendto(pkt, addr)
        except OSError:
            pass

    _packets_sent += 1


def bind_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((DSU_HOST, DSU_PORT))
    except OSError as e:
        if e.errno in (48, 98):  # EADDRINUSE
            raise SystemExit(
                f"ERROR: Port {DSU_PORT} is already in use.\n"
                "       Another DSU server (or an older copy of this script) is running.\n"
                f"       Find it with:  sudo lsof -iUDP:{DSU_PORT}"
            )
        raise SystemExit(f"ERROR: could not bind {DSU_HOST}:{DSU_PORT}: {e}")
    sock.settimeout(0.2)
    return sock


def listener():
    while not _stop.is_set():
        try:
            data, addr = _sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError:
            break

        if len(data) < 20 or data[:4] not in (b"DSUC", b"DSUS"):
            continue

        msg_type = struct.unpack_from("<I", data, 16)[0]

        if msg_type == 0x100001:
            if len(data) < 24:
                continue
            num_ports = struct.unpack_from("<i", data, 20)[0]
            for i in range(min(num_ports, 4)):
                if 24 + i < len(data):
                    _send_info(_sock, addr, data[24 + i])

        elif msg_type == 0x100002:
            if len(data) < 28:
                continue
            # Cemu registers several times with different flag/slot combinations
            # from the same socket; key on the address so it counts as one client.
            with _clients_lock:
                if addr not in _clients:
                    _log(f"[dsu]    client connected: {addr[0]}:{addr[1]}")
                _clients[addr] = time.monotonic()


def reap_clients():
    now = time.monotonic()
    with _clients_lock:
        stale = [a for a, t in _clients.items() if now - t > 5.0]
        for a in stale:
            _log(f"[dsu]    client disconnected: {a[0]}:{a[1]}")
            del _clients[a]
        return len(_clients)


# Terminal UI

HELP_FULL = "[c] calibrate  [d] debug  [r] reload  [q] quit"
HELP_SHORT = "[c]alib [d]ebug [q]uit"

_debug_lines = 0


def _clear_debug_block():
    """Move back up over the debug block so it can be rewritten in place."""
    global _debug_lines
    if _debug_lines:
        sys.stdout.write("\r\033[K")
        for _ in range(_debug_lines):
            sys.stdout.write("\033[A\033[K")
        _debug_lines = 0


def draw_status(n_clients):
    """Redraw the status line, plus the debug block when it is enabled.
    Nothing may exceed the terminal width, or the line wraps and \\r can no
    longer clear the rows above it."""
    global _last_status, _debug_lines

    if n_clients:
        client_str = f"{n_clients} client{'s' if n_clients != 1 else ''}"
        rate_str = f"{_rate_hz:5.1f} Hz"
    else:
        client_str = "waiting for Cemu"
        rate_str = "  0.0 Hz"

    width = shutil.get_terminal_size((80, 24)).columns - 1
    head = f"{client_str} | {rate_str}"

    line = head
    for help_text in (HELP_FULL, HELP_SHORT):
        candidate = f"{head} | {help_text}"
        if len(candidate) <= width:
            line = candidate
            break
    line = line[:width]

    if not _debug:
        _clear_debug_block()
        if line == _last_status:
            return
        _last_status = line
        sys.stdout.write("\r\033[K" + line)
        sys.stdout.flush()
        return

    raw = raw_sensors()
    ax, ay, az, gx, gy, gz = current_output()
    block = [
        "  raw   accel=({:+6.2f},{:+6.2f},{:+6.2f})  gyro=({:+8.2f},{:+8.2f},{:+8.2f})".format(
            raw["ax"], raw["ay"], raw["az"], raw["gx"], raw["gy"], raw["gz"]),
        "  bias  gyro=({:+8.3f},{:+8.3f},{:+8.3f})".format(
            _gyro_bias["gx"], _gyro_bias["gy"], _gyro_bias["gz"]),
        "  sent  accel=({:+6.2f},{:+6.2f},{:+6.2f})  gyro=({:+8.2f},{:+8.2f},{:+8.2f})".format(
            ax, ay, az, gx, gy, gz),
    ]

    _clear_debug_block()
    sys.stdout.write("\r\033[K" + line + "\n")
    for b in block:
        sys.stdout.write("\033[K" + b[:width] + "\n")
    # +1 for the status line itself: the cursor now sits one row below the
    # last block line, so the rewind has to cover status + block.
    _debug_lines = len(block) + 1
    _last_status = None
    sys.stdout.flush()


def handle_key(ch):
    """Returns False if the program should quit."""
    global _debug, _settings_mtime

    if ch in ("q", "\x03", "\x04"):
        return False
    if ch == "c":
        threading.Thread(target=calibrate_gyro, daemon=True).start()
    elif ch == "d":
        _debug = not _debug
        _log(f"[debug]  {'on' if _debug else 'off'}")
    elif ch == "r":
        _settings_mtime = 0.0
        load_settings()
    return True


def main_loop(interactive):
    global _rate_hz, _rate_window_start, _packets_sent

    last_poll = 0.0
    while not _stop.is_set():
        now = time.monotonic()

        if now - last_poll > SETTINGS_POLL_INTERVAL:
            load_settings()
            last_poll = now

        elapsed = now - _rate_window_start
        if elapsed >= 1.0:
            _rate_hz = _packets_sent / elapsed
            _packets_sent = 0
            _rate_window_start = now

        n_clients = reap_clients()

        if interactive:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                if not handle_key(sys.stdin.read(1)):
                    break
            draw_status(n_clients)
        else:
            time.sleep(0.1)


# Entry point

def main():
    global _sock, _debug

    parser = argparse.ArgumentParser(
        description="MacBook IMU -> DSU/CemuHook server for Cemu motion controls."
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="continuously print the mapped sensor values instead of a status line",
    )
    args = parser.parse_args()
    _debug = args.verbose

    if os.geteuid() != 0:
        raise SystemExit(
            "ERROR: This must run as root (IOKit HID access).\n"
            "       Try:  sudo $(which uv) run python3 macimu2dsu.py"
        )

    if not os.path.exists(SETTINGS_FILE):
        write_default_settings()
    load_settings(announce=False)

    _sock = bind_socket()
    print(f"[dsu]    Listening on {DSU_HOST}:{DSU_PORT}")

    imu = start_sensor()
    calibrate_gyro()

    threading.Thread(target=listener, daemon=True).start()

    interactive = _HAS_TERMIOS and sys.stdin.isatty()
    if not interactive:
        print("[main]   Non-interactive stdin: keyboard controls disabled.")

    print("[main]   Ready. In Cemu: Options > GamePad motion source > DSU1 "
          f"({DSU_HOST}:{DSU_PORT})\n")

    old_term = None
    try:
        if interactive:
            old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        main_loop(interactive)
    except KeyboardInterrupt:
        pass
    finally:
        _stop.set()
        _clear_debug_block()
        if old_term is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        sys.stdout.write("\r\033[K")
        sys.stdout.flush()
        try:
            imu.stop()
        except Exception:
            pass
        try:
            _sock.close()
        except Exception:
            pass
        print("[main]   Stopped.")


if __name__ == "__main__":
    main()
