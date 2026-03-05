from Comms.JoystickData import JoystickData
from structure.Subsystem import Subsystem
import serial
import sys

SERIAL_PORT = "/dev/ttyUSB1"
BAUD_RATE = 19200

class PiControllerReceiver(Subsystem):
    def __init__(self):
        super().__init__()
        
        # Initialize local joystick data with default values
        self.joystick_data = JoystickData(
            left_x=0.0,
            left_y=0.0,
            right_x=0.0,
            right_y=0.0,
            dpad_up=False,
            dpad_down=False,
            dpad_left=False,
            dpad_right=False,
            btn_A=False,
            btn_B=False,
            btn_X=False,
            btn_Y=False,
            btn_LB=False,
            btn_RB=False,
            btn_LS=False,
            btn_RS=False,
            btn_R2=False,
            btn_L2=False,
            btn_share=False,
            btn_options=False,
        )
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"[OK] Connected to controller receiver on {SERIAL_PORT} at {BAUD_RATE} baud")
        except serial.SerialException as e:
            print(f"[ERROR] Could not open serial port {SERIAL_PORT}: {e}")
            self.ser = None

    def periodic(self):
        """Called periodically to update joystick data from serial port"""
        if self.ser is None:
            return
        
        self._update_joystick_data()

    def _update_joystick_data(self):
        """Read data from ControllerMessager and update local joystick data"""
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                parts = line.split(",")
                if len(parts) == 4:
                    # Update analog stick values from ControllerMessager
                    self.joystick_data.left_x = float(parts[0])
                    self.joystick_data.left_y = float(parts[1])
                    self.joystick_data.right_x = float(parts[2])
                    self.joystick_data.right_y = float(parts[3])
        except ValueError:
            pass  # Ignore malformed lines
        except Exception as e:
            print(f"[ERROR] Error reading joystick data: {e}")

    def get_joystick_data(self):
        """Returns the current joystick data"""
        return self.joystick_data

    def close(self):
        """Close the serial connection"""
        if self.ser is not None:
            self.ser.close()