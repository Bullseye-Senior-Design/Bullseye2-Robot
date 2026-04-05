# ============================================================
# receive.py  –  Bullseye Serial Packet Monitor
# ============================================================
# Simple test utility that listens on the XBee serial port and
# pretty-prints every packet the Steam Deck sends.
#
# Run this on any machine with the XBee receiver plugged in:
#   python receive.py
#
# Toggle flags below to adjust behavior.
# ============================================================

import serial
import json
import sys
import time
from datetime import datetime

from Robot.Constants import Constants

# ── Config ────────────────────────────────────────────────────────────────────
PORT      = Constants.controller_serial_port 
BAUD      = Constants.serial_baud_rate      
TIMEOUT   = 1                # Serial read timeout in seconds

# Set to True to also print the raw JSON line before the parsed output.
# Useful when you suspect a packet is malformed and the parser is hiding it.
SHOW_RAW  = False
# ── End config ────────────────────────────────────────────────────────────────


# ── Packet-type pretty printers ───────────────────────────────────────────────
# Each function receives the already-parsed json_data dict/value and returns
# a human-readable string. Add a new entry here when a new packet type is added.

def _fmt_state(data: dict) -> str:
    """StateData: robot mode change (DISABLED / TELEOP / AUTONOMOUS / TEST)."""
    state_names = {0: "DISABLED", 1: "AUTONOMOUS", 2: "TELEOP", 3: "TEST"}
    state_val = data.get("state")
    # StateData serialises state as {"value": N} when it is an enum
    if isinstance(state_val, dict):
        state_val = state_val.get("value", state_val)
    state_name = state_names.get(state_val, f"UNKNOWN({state_val})")
    path_id    = data.get("path_id")
    path_speed = data.get("path_speed")
    parts = [f"state={state_name}"]
    if path_id is not None:
        parts.append(f"path_id={path_id}")
    if path_speed is not None:
        parts.append(f"speed={path_speed:.0%}")
    return "  " + "  |  ".join(parts)


def _fmt_controller(data: dict) -> str:
    """Controller delta: only the axes/buttons that changed this cycle."""
    # Axes: show value; buttons: show name only when pressed (value=1/True)
    axis_keys   = {"lx", "ly", "rx", "ry", "r2", "l2"}
    button_names = {
        "du": "D↑", "dd": "D↓", "dl": "D←", "dr": "D→",
        "ba": "A",  "bb": "B",  "bx": "X",  "by": "Y",
        "lb": "LB", "rb": "RB", "ls": "LS", "rs": "RS",
        "sh": "Share", "op": "Options",
    }
    full_names = {
        "lx": "L-X", "ly": "L-Y", "rx": "R-X", "ry": "R-Y",
        "r2": "R2",  "l2": "L2",
    }
    parts = []
    for key, val in data.items():
        if key in axis_keys:
            parts.append(f"{full_names[key]}={float(val):+.2f}")
        elif key in button_names:
            if val:   # Only show buttons that are currently pressed
                parts.append(f"[{button_names[key]}]")
            else:
                parts.append(f"[{button_names[key]} released]")
    return "  " + ("  ".join(parts) if parts else "(no changes)")


def _fmt_kfx(data: dict) -> str:
    """KFX config: button number → route_id assignments."""
    lines = []
    for btn_num in sorted(data.keys()):
        rid = data[btn_num]
        assignment = f"route {rid}" if rid is not None else "unassigned"
        lines.append(f"    Button {btn_num} → {assignment}")
    return "\n" + "\n".join(lines)


def _fmt_simple(label: str) -> str:
    """For packets that carry no payload (record_start, record_finish, etc.)."""
    return f"  [{label}]"


# Map packet type strings to formatter functions.
# Formatters receive a parsed dict; simple ones just return a label string.
FORMATTERS = {
    "state":         lambda d: _fmt_state(d),
    "c":             lambda d: _fmt_controller(d),
    "kfx_config":    lambda d: _fmt_kfx(d),
    "record_start":  lambda d: _fmt_simple("START RECORDING"),
    "record_finish": lambda d: _fmt_simple("FINISH RECORDING"),
    "record_cancel": lambda d: _fmt_simple("CANCEL RECORDING"),
    "battery":       lambda d: (
        f"  V={d.get('voltage', 0):.2f}V  "
        f"SOC={d.get('state_of_charge', 0):.1f}%  "
        f"TTG={d.get('time_remaining', 0):.0f}min"
    ),
}

# Color codes for terminal output (ANSI – work on Linux/Mac and Windows 10+)
RESET  = "\033[0m"
BOLD   = "\033[1m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
GREEN  = "\033[32m"
RED    = "\033[31m"
GREY   = "\033[90m"

TYPE_COLORS = {
    "state":         YELLOW,
    "c":             CYAN,
    "kfx_config":    GREEN,
    "record_start":  GREEN,
    "record_finish": GREEN,
    "record_cancel": RED,
    "battery":       GREY,
}


def process_line(line: str):
    """
    Parse one newline-stripped serial line as a DataPacket and print it.
    Unknown packet types are printed as-is so nothing is silently dropped.
    """
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]   # HH:MM:SS.mmm

    if SHOW_RAW:
        print(f"{GREY}[RAW] {line}{RESET}")

    try:
        outer = json.loads(line)
        ptype     = outer.get("type", "unknown")
        json_data = outer.get("json_data", "{}")

        # json_data is itself a JSON string (DataPacket model) – parse it
        try:
            inner = json.loads(json_data)
        except (json.JSONDecodeError, TypeError):
            inner = json_data   # Already a plain value or unparseable

        color     = TYPE_COLORS.get(ptype, BOLD)
        formatter = FORMATTERS.get(ptype)

        if formatter:
            detail = formatter(inner) if isinstance(inner, dict) else _fmt_simple(str(inner))
        else:
            # Unknown type – just dump the inner data compactly
            detail = f"  {json.dumps(inner)}"

        print(f"{GREY}{ts}{RESET}  {color}{BOLD}{ptype.upper():<16}{RESET}{detail}")

    except (json.JSONDecodeError, KeyError) as e:
        # Not a valid DataPacket – print raw with a warning
        print(f"{GREY}{ts}{RESET}  {RED}[PARSE ERROR]{RESET}  {line}  ({e})")


def main():
    print(f"\n{'='*60}")
    print(f"  Bullseye Packet Monitor")
    print(f"  Port: {PORT}  |  Baud: {BAUD}")
    print(f"  Waiting for packets… (Ctrl+C to exit)")
    print(f"{'='*60}\n")

    # ── Open serial port ──────────────────────────────────────────────────
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(0.5)   # Let port settle
        print(f"{GREEN}[OK] Port open{RESET}\n")
    except serial.SerialException as e:
        print(f"{RED}[ERROR] Could not open {PORT}: {e}{RESET}")
        print("Check that the XBee is plugged in and PORT is set correctly.")
        sys.exit(1)

    # ── Read loop ─────────────────────────────────────────────────────────
    buffer = ""
    try:
        while True:
            try:
                # Read whatever is available; accumulate into buffer
                if ser.in_waiting:
                    chunk = ser.read(ser.in_waiting).decode(errors="ignore")
                    buffer += chunk

                    # Process every complete newline-terminated line
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            process_line(line)
                else:
                    time.sleep(0.01)   # Avoid busy-spinning when idle

            except serial.SerialException as e:
                print(f"\n{RED}[ERROR] Serial disconnected: {e}{RESET}")
                break

    except KeyboardInterrupt:
        print(f"\n{GREY}[MONITOR] Stopped by user.{RESET}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
