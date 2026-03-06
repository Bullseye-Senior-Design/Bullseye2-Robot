import json
import serial
import time
import pygame
import sys
import os

# Add the parent directory to the Python path so Comms module can be found
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Comms.ControllerData import ControllerData
from Comms.StateData import State
from Robot.Constants import Constants
from dataclasses import asdict

# ==== CONFIG ====
DEBUG = True                # Set to True for debugging output
ErrTEST = False             # Set to True to allow running without serial connection (for testing without hardware)
MENU = False
PORT = Constants.controller_serial_port
BAUD = Constants.serial_baud_rate
DEADZONE = Constants.controller_deadzone
UPDATE_RATE = Constants.controller_update_rate  # ~20 Hz


def main():
    # Initialize current state
    current_state = State.DISABLED
    
    if MENU:
         # Print mode selection instructions
        print("\n" + "="*50)
        print("BULLSEYE CONTROLLER MESSAGER - MODE SELECTION")
        print("="*50)
        print("Press the corresponding button to select mode:")
        print("  D-Pad UP    -> TELEOP")
        print("  D-Pad RIGHT -> AUTONOMOUS (Path Following)")
        print("  D-Pad LEFT  -> TEST (WARNING)")
        print("  B Button    -> DISABLED (Emergency Stop)")
        print("="*50)
        print(f"Current Mode: {current_state.name}")
        print("="*50)
    # Print mode selection instructions

    # ==== Serial ====
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        print(f"[OK] Connected to XBee on {PORT} at {BAUD} baud")
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
    print(f"[OK] Detected controller: {joystick.get_name()}")

    # Track previous button states to detect presses (not holds)
    prev_dpad_up = False
    prev_dpad_down = False
    prev_dpad_left = False
    prev_dpad_right = False
    prev_btn_B = False

    try:
        while True:
            pygame.event.pump()  # Process event queue

            # Read joystick axes
            left_x = joystick.get_axis(0)
            left_y = joystick.get_axis(1)
            right_x = joystick.get_axis(2)
            right_y = joystick.get_axis(3)
            #l2_axis = joystick.get_axis(4) if joystick.get_numaxes() > 4 else 0.0
            #r2_axis = joystick.get_axis(5) if joystick.get_numaxes() > 5 else 0.0

            # Debug: Print axis values
            if DEBUG:
                if abs(left_x) > DEADZONE:
                    print(f"axis(0) - {left_x:.2f}")
                if abs(left_y) > DEADZONE:
                    print(f"axis(1) - {left_y:.2f}")
                if abs(right_x) > DEADZONE:
                    print(f"axis(2) - {right_x:.2f}")
                if abs(right_y) > DEADZONE:
                    print(f"axis(3) - {right_y:.2f}")
                #if abs(l2_axis) > DEADZONE:
                    #print(f"axis(4-L2) - {l2_axis:.2f}")
                #if abs(r2_axis) > DEADZONE:
                    #print(f"axis(5-R2) - {r2_axis:.2f}")

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

            # Read D-pad states
            #dpad_x, dpad_y = joystick.get_hat(0)
            #dpad_up = dpad_y == 1
            #dpad_down = dpad_y == -1
            #dpad_right = dpad_x == 1
            #dpad_left = dpad_x == -1

            # Alternative D-pad reading (some controllers might not use hats)
            dpad_up = joystick.get_button(11)
            dpad_down = joystick.get_button(12)
            dpad_left = joystick.get_button(13)
            dpad_right = joystick.get_button(14)
            
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

            # Check for mode change inputs (only on press, not hold)
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
            elif btn_B and not prev_btn_B:
                new_state = State.DISABLED
                state_changed = True
                print("🛑 EMERGENCY STOP: DISABLED")

            # Update current state if changed
            if state_changed:
                current_state = new_state
                print(f"Current Mode: {current_state.name}")
                
                # Send state change command as JSON
                state_command = {
                    "type": "state_change",
                    "state": current_state.value,
                    "state_name": current_state.name
                }
                ser.write((json.dumps(state_command) + "\n").encode())
                #if DEBUG:
                    #print(f"[STATE] Sent: {json.dumps(state_command)}")

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
            #l2_axis = 0 if abs(l2_axis) < DEADZONE else l2_axis
            #r2_axis = 0 if abs(r2_axis) < DEADZONE else r2_axis

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
                btn_R2= 0,
                btn_L2= 0,
                btn_share=btn_share,
                btn_options=btn_options,
            )

            # Send over serial (joystick data as JSON; state commands sent separately above)
            joystick_payload = {
                "type": "joystick",
        **asdict(data),        
            }
            ser.write((json.dumps(joystick_payload) + "\n").encode())
            #if DEBUG:
                #print(f"[JOYSTICK] Sent: {json.dumps(joystick_payload)}")

            time.sleep(UPDATE_RATE)


    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()
        pygame.quit()
    
if __name__ == "__main__":
    main()


