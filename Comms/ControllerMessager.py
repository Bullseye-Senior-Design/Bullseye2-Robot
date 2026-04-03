import json
import serial
import time
import pygame
import sys
import os
import threading

# Add the parent directory to the Python path so Comms module can be found
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Comms.ControllerData import ControllerData
from Comms.StateData import State, StateData
from Comms.BatteryData import BatteryData
from Comms.DataPacket import DataPacket
from Robot.Constants import Constants

# ==== CONFIG ====
DEBUG = False                # Set to True for debugging output
DEBUGPRINT = False           # Set to True for debug print statements
ErrTEST = False             # Set to True to allow running without serial connection (for testing without hardware)
MENU = False
ESTOP = False              # If True, pressing the Y button will immediately send a DISABLED state to the robot (emergency stop)
PORT = Constants.controller_serial_port
BAUD = Constants.serial_baud_rate
DEADZONE = Constants.controller_deadzone
UPDATE_RATE = Constants.controller_update_rate  # ~20 Hz

# ==== DELTA ENCODING ====
# Maps full ControllerData field names to short keys sent over the wire.
# Receiver must use the same map to reconstruct the full field name.
_FIELD_MAP = {
    'left_x': 'lx', 'left_y': 'ly', 'right_x': 'rx', 'right_y': 'ry',
    'dpad_up': 'du', 'dpad_down': 'dd', 'dpad_left': 'dl', 'dpad_right': 'dr',
    'btn_A': 'ba', 'btn_B': 'bb', 'btn_X': 'bx', 'btn_Y': 'by',
    'btn_LB': 'lb', 'btn_RB': 'rb', 'btn_LS': 'ls', 'btn_RS': 'rs',
    'btn_R2': 'r2', 'btn_L2': 'l2', 'btn_share': 'sh', 'btn_options': 'op',
}
_FLOAT_FIELDS = {'left_x', 'left_y', 'right_x', 'right_y', 'btn_R2', 'btn_L2'}

def _build_delta(current: ControllerData, previous: ControllerData | None) -> dict:
    """Return only the fields that changed since the previous packet, with short keys.
    If previous is None (first packet), all fields are included.
    Floats are rounded to 2dp to avoid sending tiny floating point noise."""
    delta = {}
    for field, short_key in _FIELD_MAP.items():
        curr_val = getattr(current, field)
        if field in _FLOAT_FIELDS:
            curr_val = round(curr_val, 2)
            prev_val = round(getattr(previous, field), 2) if previous else None
        else:
            curr_val = int(curr_val)
            prev_val = int(getattr(previous, field)) if previous else None
        if curr_val != prev_val:
            delta[short_key] = curr_val
    return delta

# Shared battery data (updated by receiver thread)
_battery_data: BatteryData = BatteryData(voltage=0.0, current=0.0, power=0.0, state_of_charge=0.0, time_remaining=0.0)
_battery_lock = threading.Lock()


def _receive_robot_data(ser: serial.Serial):
    """Background thread that receives DataPackets from the robot (e.g. battery data)."""
    global _battery_data
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            try:
                packet = DataPacket.model_validate_json(line)
                if packet.type == "battery":
                    with _battery_lock:
                        _battery_data = BatteryData.model_validate_json(packet.json_data)
                    if DEBUG:
                        with _battery_lock:
                            b = _battery_data
                        print(f"[BATTERY] V:{b.voltage:.2f}V  I:{b.current:.2f}A  SOC:{b.state_of_charge:.1f}%  TTG:{b.time_remaining:.0f}min")
            except Exception as e:
                if DEBUG:
                    print(f"[RECV] Could not parse: {line} -> {e}")
        except Exception:
            break



def printmenu():
    print("\n" + "="*50)
    print("BULLSEYE CONTROLLER MESSAGER - MODE SELECTION")
    print("="*50)
    print("Press the corresponding button to select mode:")
    print("  D-Pad UP    -> TELEOP")
    print("  D-Pad RIGHT -> AUTONOMOUS (Path Following)")
    print("  D-Pad LEFT  -> TEST (WARNING)")
    print("  D-Pad DOWN  -> (Show Battery Data)")
    print("  Y Button    -> DISABLED (Emergency Stop)")
    print("="*50) 
def main():
    # Initialize current state
    current_state = State.DISABLED
    
    if MENU:
         # Print mode selection instructions
        printmenu()
    # Print mode selection instructions

    # ==== Serial ====
    ser = None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        print(f"[OK] Connected to XBee on {PORT} at {BAUD} baud")
        # Start background thread to receive data from robot
        recv_thread = threading.Thread(target=_receive_robot_data, args=(ser,), daemon=True, name="RobotReceiver")
        recv_thread.start()
    except serial.SerialException:
        if ErrTEST:
             print(f"[TEST MODE] Could not open serial port {PORT}, but continuing in test mode.")
        else:
            print(f"[ERROR] Could not open serial port {PORT}. Make sure the controller receiver is connected and the port is correct.")
            sys.exit(1)

    # ==== Pygame ====
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("❌ No controller detected!")
        sys.exit(1)



    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    if joystick:
        print(f"[OK] Detected controller: {joystick.get_name()}")
    else:
        print("[OK] Detected a controller, but could not get its name.")
    
    print("Number of buttons:", joystick.get_numbuttons())
    print("Number of hats:", joystick.get_numhats())   # Important for D-pad
    print("Number of axes:", joystick.get_numaxes())

    # Track previous button states to detect presses (not holds)
    prev_dpad_up = False
    prev_dpad_down = False
    prev_dpad_left = False
    prev_dpad_right = False
    prev_btn_B = False
    l2_prev = -1.0
    r2_prev = -1.0
    prev_data = None

    try:
        while True:
            pygame.event.pump()  # Process event queue

            # Read joystick axes
            left_x = joystick.get_axis(0)
            left_y = joystick.get_axis(1)
            left_y = -left_y  # Invert Y-axis for intuitive control
            right_x = joystick.get_axis(2)
            right_y = joystick.get_axis(3)
            right_x = -right_x  # Invert Y-axis for intuitive control

            l2_axis = joystick.get_axis(4)
            r2_axis = joystick.get_axis(5)

            # Debug: Print axis values
            if DEBUG:
                # if abs(left_x) > DEADZONE:
                #     print(f"axis(0) - {left_x:.2f}")
                # if abs(left_y) > DEADZONE:
                #     print(f"axis(1) - {left_y:.2f}")
                # if abs(right_x) > DEADZONE:
                #     print(f"axis(2) - {right_x:.2f}")
                # if abs(right_y) > DEADZONE:
                #     print(f"axis(3) - {right_y:.2f}")
                if abs(l2_axis - l2_prev) > 0.01:  # Only print if changed significantly to reduce spam
                    print(f"axis(4-L2) - {l2_axis:.2f}")
                if abs(r2_axis - r2_prev) > 0.01:  # Only print if changed significantly to reduce spam
                    print(f"axis(5-R2) - {r2_axis:.2f}")
            l2_prev = l2_axis
            r2_prev = r2_axis

            # Read button states
            btn_A = joystick.get_button(0)
            btn_B = joystick.get_button(1)
            btn_X = joystick.get_button(2)
            btn_Y = joystick.get_button(3)
            btn_LB = joystick.get_button(9)
            btn_RB = joystick.get_button(10)
            btn_LS = joystick.get_button(7)
            btn_RS = joystick.get_button(8)
            btn_share = joystick.get_button(4)
            btn_options = joystick.get_button(6)

            # Alternative D-pad reading
            dpad_up = joystick.get_button(11)
            dpad_down = joystick.get_button(12)
            dpad_left = joystick.get_button(13)
            dpad_right = joystick.get_button(14)

            if ESTOP and btn_Y:
                print("EMERGENCY STOP ACTIVATED!")
                current_state = State.DISABLED
                state_data = StateData(state=current_state, path_speed=None, path_id=None)
                packet = DataPacket(type="state", json_data=state_data.model_dump_json())
                if ser:
                    ser.write((packet.model_dump_json() + "\n").encode())
                print(f"[STATE] Sent: {packet.model_dump_json()}")
                continue  # Skip rest of loop to immediately send stop command
            
            # Debug: Print button presses
            if DEBUG:
                if btn_A:
                    print("button(0-A) pressed")
                if btn_B:
                    print("button(1-B) pressed")
                if btn_X:
                    print("button(2-X) pressed")
                if btn_Y:
                    print("button(3-Y) pressed")
                if btn_LB:
                    print("button(9-LB) pressed")
                if btn_RB:
                    print("button(10-RB) pressed")
                if btn_LS:
                    print("button(7-LS) pressed")
                if btn_RS:
                    print("button(8-RS) pressed")

                if btn_share:
                    print("button(4-Share) pressed")
                if btn_options:
                    print("button(6-Options) pressed")

            # Debug: Print D-pad presses
            if DEBUG:
                if dpad_up:
                    print("dpad(up) pressed")
                if dpad_down:
                    print("dpad(down) pressed")
                if dpad_left:
                    print("dpad(left) pressed")
                if dpad_right:
                    print("dpad(right) pressed")

            # ===== MODE SELECTION LOGIC =====
            state_changed = False
            new_state = current_state

            # print("am here")

            # Check for mode change inputs (only on press, not hold)
            if current_state != State.DISABLED:
                if btn_B:
                    current_state = State.DISABLED
                    state_changed = True
                    if MENU:
                        printmenu()
            elif current_state == State.DISABLED:    
                if dpad_up and not prev_dpad_up:
                    new_state = State.TELEOP
                    state_changed = True
                    print("🔄 MODE CHANGE: TELEOP (Manual Control)")
                elif dpad_right and not prev_dpad_right:
                    new_state = State.AUTONOMOUS
                    state_changed = True
                    print("🔄 MODE CHANGE: AUTONOMOUS (Path Following)")
                elif dpad_left and not prev_dpad_left:
                    new_state = State.TEST
                    state_changed = True
                    print("🔄 MODE CHANGE: TEST (Same as Teleop)")
                elif dpad_down and not prev_dpad_down:
                    with _battery_lock:
                        b = _battery_data
                    print(f"🔋 BATTERY DATA: V:{b.voltage:.2f}V")  
                    print(f" I:{b.current:.2f}A")  
                    print(f" SOC:{b.state_of_charge:.1f}%")
                    hours = int(b.time_remaining // 60)
                    minutes = int(b.time_remaining % 60)     
                    print(f" TTG:{hours}h {minutes}m")

            # Update current state if changed
            if state_changed:
                current_state = new_state
                # Send state change as DataPacket
                state_data = StateData(state=current_state, path_speed=None, path_id=None)
                packet = DataPacket(type="state", json_data=state_data.model_dump_json())
                if ser:
                    ser.write((packet.model_dump_json() + "\n").encode())
                if DEBUG:
                    print(f"[STATE] Sent: {packet.model_dump_json()}")
                # Skip controller data send this iteration so both packets
                # don't go out simultaneously and get combined by the XBee
                prev_dpad_up = dpad_up
                prev_dpad_down = dpad_down
                prev_dpad_left = dpad_left
                prev_dpad_right = dpad_right
                time.sleep(UPDATE_RATE)
                continue

            # Update previous button states
            prev_dpad_up = dpad_up
            prev_dpad_down = dpad_down
            prev_dpad_left = dpad_left
            prev_dpad_right = dpad_right
            prev_btn_B = btn_B

            # Apply deadzone
            left_x = 0 if abs(left_x) < DEADZONE else left_x
            left_y = 0 if abs(left_y) < DEADZONE else left_y
            right_x = 0 if abs(right_x) < DEADZONE else right_x
            right_y = 0 if abs(right_y) < DEADZONE else right_y
            l2_axis = 0 if abs(l2_axis) < DEADZONE else l2_axis
            r2_axis = 0 if abs(r2_axis) < DEADZONE else r2_axis

            # Create ControllerData object with all fields in correct order
            data = ControllerData(
                left_x=left_x,
                left_y=left_y,
                right_x=right_x,
                right_y=right_y,
                dpad_up=dpad_up,
                dpad_down=dpad_down,
                dpad_left=dpad_left,
                dpad_right=dpad_right,
                btn_A=btn_A,
                btn_B=btn_B,
                btn_X=btn_X,
                btn_Y=btn_Y,
                btn_LB=btn_LB,
                btn_RB=btn_RB,
                btn_LS=btn_LS,
                btn_RS=btn_RS,
                btn_R2= l2_axis,
                btn_L2= r2_axis,
                btn_share=btn_share,
                btn_options=btn_options,
            )

            # Send only changed fields as a delta packet
            delta = _build_delta(data, prev_data)
            if delta:
                packet = DataPacket(type="c", json_data=json.dumps(delta))
                if ser:
                    ser.write((packet.model_dump_json() + "\n").encode())
                if DEBUGPRINT:
                    print(f"[JOYSTICK] Sent delta: {delta}")

            prev_data = data
            time.sleep(UPDATE_RATE)


    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if ser:
            ser.close()
        pygame.quit()
    
if __name__ == "__main__":
    main()


