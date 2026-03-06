from Comms.ControllerData import JoystickData
from Robot.Constants import Constants
from structure.Subsystem import Subsystem
import serial
import sys
import json

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
            self.ser = serial.Serial(Constants.controller_serial_port, Constants.serial_baud_rate, timeout=1)
            print(f"[OK] Connected to controller receiver on {Constants.controller_serial_port} at {Constants.serial_baud_rate} baud")
        except serial.SerialException as e:
            print(f"[ERROR] Could not open serial port {Constants.controller_serial_port}: {e}")
            self.ser = None

    def periodic(self):
        """Called periodically to update joystick data from serial port"""
        if self.ser is None:
            return
        
        self._update_joystick_data()

    def _update_joystick_data(self):
        """Read JSON data from ControllerMessager and update local joystick data"""
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                return

            # Expect JSON payload from the controller
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                # Fallback: treat as legacy comma-separated format
                parts = line.split(",")
                if len(parts) == 4:
                    self.joystick_data.left_x = float(parts[0])
                    self.joystick_data.left_y = float(parts[1])
                    self.joystick_data.right_x = float(parts[2])
                    self.joystick_data.right_y = float(parts[3])
                return

            # Only handle joystick payloads for this receiver
            if isinstance(data, dict) and data.get("type") != "joystick":
                return

            # Filter out any extra fields (e.g. "type") before constructing objects
            filtered_data = {k: data[k] for k in JoystickData.__annotations__.keys() if k in data}

            # Deserialize into our data class (using pydantic if available)
            if hasattr(JoystickData, "__annotations__"):
                try:
                    # Try using Pydantic model if available
                    from Comms.ControllerData import JoystickDataModel
                    model = JoystickDataModel(**filtered_data)
                    self.joystick_data = JoystickData(**model.dict())
                except Exception:
                    # Fallback to dataclass construction if pydantic not available or fails
                    self.joystick_data = JoystickData(**filtered_data)
            else:
                self.joystick_data = JoystickData(**filtered_data)

        except Exception as e:
            print(f"[ERROR] Error reading joystick data: {e}")

    def get_joystick_data(self):
        """Returns the current joystick data"""
        return self.joystick_data

    def close(self):
        """Close the serial connection"""
        if self.ser is not None:
            self.ser.close()