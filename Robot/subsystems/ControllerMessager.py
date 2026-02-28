from Comms.JoystickData import JoystickData
import serial
import time
import pygame
import sys

# ==== CONFIG ====
DEBUG = True                # Set to True for debugging output
ErrTEST = False             # Set to True to allow running without serial connection (for testing without hardware)
PORT = "/dev/ttyUSB0"       # change if needed
BAUD = 19200
DEADZONE = 0.15
UPDATE_RATE = 0.05          # ~20 Hz


def main():
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

    try:
        while True:
            pygame.event.pump()  # Process event queue

            # Read joystick axes
            left_x = joystick.get_axis(0)
            left_y = joystick.get_axis(1)
            right_x = joystick.get_axis(2)
            right_y = joystick.get_axis(3)
            l2_axis = joystick.get_axis(4) if joystick.get_numaxes() > 4 else 0.0
            r2_axis = joystick.get_axis(5) if joystick.get_numaxes() > 5 else 0.0

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
                if abs(l2_axis) > DEADZONE:
                    print(f"axis(4-L2) - {l2_axis:.2f}")
                if abs(r2_axis) > DEADZONE:
                    print(f"axis(5-R2) - {r2_axis:.2f}")

            # Read button states
            btn_A = joystick.get_button(0)
            btn_B = joystick.get_button(1)
            btn_X = joystick.get_button(2)
            btn_Y = joystick.get_button(3)
            btn_LB = joystick.get_button(4) if joystick.get_numbuttons() > 4 else False
            btn_RB = joystick.get_button(5) if joystick.get_numbuttons() > 5 else False
            btn_LS = joystick.get_button(8) if joystick.get_numbuttons() > 8 else False
            btn_RS = joystick.get_button(9) if joystick.get_numbuttons() > 9 else False
            btn_share = joystick.get_button(6) if joystick.get_numbuttons() > 6 else False
            btn_options = joystick.get_button(7) if joystick.get_numbuttons() > 7 else False

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
                    print("button(4-LB) pressed")
                if btn_RB:
                    print("button(5-RB) pressed")
                if btn_LS:
                    print("button(8-LS) pressed")
                if btn_RS:
                    print("button(9-RS) pressed")
                if btn_share:
                    print("button(6-Share) pressed")
                if btn_options:
                    print("button(7-Options) pressed")

            # Read D-pad states
            dpad_x, dpad_y = joystick.get_hat(0)
            dpad_up = dpad_y == 1
            dpad_down = dpad_y == -1
            dpad_right = dpad_x == 1
            dpad_left = dpad_x == -1

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

            # Apply deadzone
            left_x = 0 if abs(left_x) < DEADZONE else left_x
            left_y = 0 if abs(left_y) < DEADZONE else left_y
            right_x = 0 if abs(right_x) < DEADZONE else right_x
            right_y = 0 if abs(right_y) < DEADZONE else right_y
            l2_axis = 0 if abs(l2_axis) < DEADZONE else l2_axis
            r2_axis = 0 if abs(r2_axis) < DEADZONE else r2_axis

            # Create JoystickData object with all fields in correct order
            data = JoystickData(
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
                btn_R2=r2_axis > DEADZONE,
                btn_L2=l2_axis > DEADZONE,
                btn_share=btn_share,
                btn_options=btn_options,
            )

            # Send over serial
            ser.write(f"{data.left_x:.2f},{data.left_y:.2f},{data.right_x:.2f},{data.right_y:.2f}\n".encode())

            time.sleep(UPDATE_RATE)


    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()
        pygame.quit()
    
if __name__ == "__main__":
    main()


