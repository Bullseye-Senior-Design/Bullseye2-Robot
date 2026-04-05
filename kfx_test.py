"""
KFX Remote Test - Standalone
Listens on UART pins 14/15 (GPIO 14=TX, GPIO 15=RX -> /dev/ttyAMA0 on Pi5)
and prints each button value received.

Run with:  python3 kfx_test.py
Stop with: Ctrl+C
"""

import serial
import sys

PORT = "/dev/ttyAMA0"    # UART on pins 14 (TX) / 15 (RX) on Pi5
BAUD = 115200             

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {PORT}: {e}")
        print("Check that UART is enabled and no other process is using the port.")
        sys.exit(1)

    print(f"Listening for KFX remote input on {PORT}. Press Ctrl+C to stop.\n")

    try:
        while True:
            byte = ser.read(1)
            if byte:
                value = byte[0]
                print(f"Received: {value}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
