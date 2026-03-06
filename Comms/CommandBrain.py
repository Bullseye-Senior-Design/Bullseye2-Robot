"""
COMMAND BRAIN - Main Robot Controller

The CommandBrain is the central orchestrator of the Bullseye-2 robot system.
It continuously monitors and updates all subsystems:
- BMS (Battery Management System): Reads battery voltage, current, power, and state of charge
- ControllerMessager: Receives state change commands from the controller
- State Management: Tracks the current operating mode and robot status
- Threading: Runs subsystem updates in background loops to prevent blocking

This module acts as the "brain" of the robot, coordinating all subsystems and
maintaining a local cache of system data that other components can query.
"""

import serial
import time
import threading
import json
import sys

from Comms.JoystickData import JoystickData
from Comms.BatteryData import BatteryData
from Comms.StateData import StateData, State

# Optional pydantic support for JSON deserialization
try:
    from Comms.JoystickData import JoystickDataModel
except ImportError:
    JoystickDataModel = None

from Robot.Constants import Constants

# ==== DEBUG/CONFIGURATION ====
DEBUG = True                    # Set to True for debugging output
SUBSYSTEM_UPDATE_RATE = Constants.controller_update_rate  # ~10 Hz for subsystem polling

# ==== ROBOT STATE DATA ====
class RobotBrainData:
    """Holds all the robot's current state and sensor data"""
    def __init__(self):
        # Operating mode
        self.mode = State.DISABLED  # Current mode (DISABLED, AUTONOMOUS, TELEOP, TEST)

        # Battery data from BMS
        self.battery_voltage = 0.0      # Volts
        self.battery_current = 0.0      # Amps
        self.battery_power = 0.0        # Watts
        self.battery_soc = 0.0          # State of Charge (%)
        self.battery_time_remaining = 0.0  # Minutes

        # Controller input data (from PiControllerReceiver)
        self.controller_data = JoystickData(
            left_x=0.0, left_y=0.0,
            right_x=0.0, right_y=0.0,
            dpad_up=False, dpad_down=False,
            dpad_left=False, dpad_right=False,
            btn_A=False, btn_B=False,
            btn_X=False, btn_Y=False,
            btn_LB=False, btn_RB=False,
            btn_LS=False, btn_RS=False,
            btn_R2=False, btn_L2=False,
            btn_share=False, btn_options=False,
        )

        # Last received command
        self.last_command = "None"

        # System timestamps
        self.battery_last_update = 0.0
        self.controller_last_update = 0.0
        self.state_last_update = 0.0


class CommandBrain:
    """
    Central coordinator for the Bullseye-2 robot system.

    Manages:
    - Continuous monitoring of subsystems (BMS, ControllerMessager)
    - Local state and data caching
    - Robot mode transitions
    - Thread-safe access to shared data
    """

    def __init__(self, pi_controller_receiver=None, drivetrain=None, bms=None):
        """
        Initialize CommandBrain with references to subsystems

        Args:
            pi_controller_receiver: PiControllerReceiver subsystem instance
            drivetrain: DriveTrain subsystem instance for motor control
            bms: BMS subsystem instance
        """
        self.pi_controller_receiver = pi_controller_receiver
        self.drivetrain = drivetrain
        self.bms = bms
        self.robot_data = RobotBrainData()

        # Thread management
        self._update_threads = []
        self._running = True
        self._data_lock = threading.Lock()

        # Serial connection for receiving from ControllerMessager
        self.controller_ser = None
        
        # TELEOP mode control flag
        self._teleop_active = False

        if DEBUG:
            print("[CommandBrain] Initialized")

    def start(self):
        """Start all monitoring threads"""
        # Start battery monitoring thread
        battery_thread = threading.Thread(
            target=self._update_battery_data,
            daemon=True,
            name="BatteryUpdate"
        )
        battery_thread.start()
        self._update_threads.append(battery_thread)
        if DEBUG:
            print("[CommandBrain] Started battery update thread")

        # Start controller receiver thread
        controller_thread = threading.Thread(
            target=self._receive_controller_commands,
            daemon=True,
            name="ControllerReceiver"
        )
        controller_thread.start()
        self._update_threads.append(controller_thread)
        if DEBUG:
            print("[CommandBrain] Started controller receiver thread")

        # Start controller data monitoring thread (if PiControllerReceiver is available)
        if self.pi_controller_receiver:
            joystick_thread = threading.Thread(
                target=self._update_controller_data,
                daemon=True,
                name="ControllerDataUpdate"
            )
            joystick_thread.start()
            self._update_threads.append(joystick_thread)
            if DEBUG:
                print("[CommandBrain] Started controller data update thread")
        
        # Start TELEOP control thread (if DriveTrain is available)
        if self.drivetrain:
            teleop_thread = threading.Thread(
                target=self._teleop_control_loop,
                daemon=True,
                name="TeleopControl"
            )
            teleop_thread.start()
            self._update_threads.append(teleop_thread)
            if DEBUG:
                print("[CommandBrain] Started TELEOP control thread")

    def _update_battery_data(self):
        """
        Continuously poll battery data from BMS.
        Runs in background thread.
        """
        while self._running:
            try:
                if self.bms:
                    battery_data = self.bms.read_smartshunt()
                else:
                    battery_data = None

                if battery_data:
                    with self._data_lock:
                        self.robot_data.battery_voltage = battery_data.voltage
                        self.robot_data.battery_current = battery_data.current
                        self.robot_data.battery_power = battery_data.power
                        self.robot_data.battery_soc = battery_data.state_of_charge
                        self.robot_data.battery_time_remaining = battery_data.time_remaining
                        self.robot_data.battery_last_update = time.time()

                    if DEBUG:
                        print(f"[Battery] V:{battery_data.voltage:.2f}V "
                              f"I:{battery_data.current:.2f}A "
                              f"P:{battery_data.power:.1f}W "
                              f"SOC:{battery_data.state_of_charge:.1f}%")

                time.sleep(SUBSYSTEM_UPDATE_RATE)

            except Exception as e:
                print(f"[ERROR] Error updating battery data: {e}")
                time.sleep(SUBSYSTEM_UPDATE_RATE)

    def _receive_controller_commands(self):
        """
        Continuously receive commands from ControllerMessager over serial.
        Runs in background thread.
        """
        try:
            # Initialize serial connection to receive from ControllerMessager
            self.controller_ser = serial.Serial(Constants.controller_serial_port, Constants.serial_baud_rate, timeout=1)
            if DEBUG:
                print(f"[CommandBrain] Connected to controller receiver on {Constants.controller_serial_port}")

            while self._running:
                try:
                    # Read line from serial
                    line = self.controller_ser.readline().decode().strip()

                    if line:
                        try:
                            # Try to parse as JSON (state change commands)
                            msg = json.loads(line)

                            # Determine packet type
                            packet_type = msg.get("type")

                            if packet_type == "state_change":
                                new_state_value = msg.get("state")
                                new_state_name = msg.get("state_name")

                                # Convert state value to State enum
                                try:
                                    new_state = State(new_state_value)
                                    self._handle_state_change(new_state)
                                    if DEBUG:
                                        print(f"[STATE] Received: {new_state_name} ({new_state_value})")
                                except ValueError:
                                    print(f"[ERROR] Invalid state value: {new_state_value}")

                            elif packet_type == "joystick" or ("left_x" in msg and "right_x" in msg):
                                # Deserialize joystick packet into our data class
                                try:
                                    # Filter out any non-joystick fields (e.g. "type")
                                    filtered = {k: msg[k] for k in JoystickData.__annotations__.keys() if k in msg}

                                    if JoystickDataModel is not None:
                                        model = JoystickDataModel(**filtered)
                                        joystick = JoystickData(**model.dict())
                                    else:
                                        joystick = JoystickData(**filtered)

                                    with self._data_lock:
                                        self.robot_data.controller_data = joystick
                                        self.robot_data.controller_last_update = time.time()

                                    if DEBUG:
                                        print(f"[JOYSTICK] Received: {joystick}")
                                except Exception as e:
                                    print(f"[ERROR] Failed to parse joystick packet: {e}")

                        except json.JSONDecodeError:
                            # Not JSON; log if debug
                            if DEBUG and line:
                                print(f"[Controller] Raw data: {line}")

                except Exception as e:
                    print(f"[ERROR] Error reading controller command: {e}")
                    time.sleep(0.1)

        except serial.SerialException as e:
            print(f"[ERROR] Could not connect to controller receiver: {e}")
            if DEBUG:
                print("[CommandBrain] Running without controller command receiver")

    def _update_controller_data(self):
        """
        Continuously poll controller data from PiControllerReceiver.
        Runs in background thread.
        """
        while self._running:
            try:
                if self.pi_controller_receiver:
                    joystick_data = self.pi_controller_receiver.get_joystick_data()

                    with self._data_lock:
                        self.robot_data.controller_data = joystick_data
                        self.robot_data.controller_last_update = time.time()

                    if DEBUG:
                        # Only print when there's actual input (not all zeros/defaults)
                        has_input = (
                            abs(joystick_data.left_x) > 0.01 or
                            abs(joystick_data.left_y) > 0.01 or
                            abs(joystick_data.right_x) > 0.01 or
                            abs(joystick_data.right_y) > 0.01 or
                            any([
                                joystick_data.btn_A, joystick_data.btn_B,
                                joystick_data.btn_X, joystick_data.btn_Y,
                                joystick_data.dpad_up, joystick_data.dpad_down,
                                joystick_data.dpad_left, joystick_data.dpad_right
                            ])
                        )
                        if has_input:
                            print(f"[Controller] LX:{joystick_data.left_x:.2f} "
                                  f"LY:{joystick_data.left_y:.2f} "
                                  f"RX:{joystick_data.right_x:.2f} "
                                  f"RY:{joystick_data.right_y:.2f}")

                time.sleep(SUBSYSTEM_UPDATE_RATE)

            except Exception as e:
                print(f"[ERROR] Error updating controller data: {e}")
                time.sleep(SUBSYSTEM_UPDATE_RATE)

    def _teleop_control_loop(self):
        """
        Continuously process TELEOP mode joystick inputs and send to DriveTrain.
        Runs in background thread.
        
        Maps:
        - left_y: -1.0 to 1.0 (throttle) -> sent as t: -1.00 to 1.00
        - right_x: -1.0 to 1.0 (steering) -> sent as s: -1.00 to 1.00
        """
        while self._running:
            try:
                # Only process if TELEOP is active
                if self._teleop_active:
                    with self._data_lock:
                        controller_data = self.robot_data.controller_data
                    
                    # Get joystick inputs
                    throttle = controller_data.left_y      # -1.0 to 1.0
                    steering = controller_data.right_x     # -1.0 to 1.0
                    
                    # Send command to DriveTrain via set_speed_angle
                    # Throttle maps to speed, steering maps to angle
                    if self.drivetrain:
                        # Convert steering (-1.0 to 1.0) to angle (0 to 180)
                        # steering=-1.0 -> angle=180 (full left)
                        # steering=0 -> angle=90 (straight)
                        # steering=1.0 -> angle=0 (full right)
                        angle = 90 - (steering * 90)
                        angle = max(0, min(180, angle))  # Clamp to 0-180
                        
                        self.drivetrain.set_speed_angle(throttle, angle)
                        
                        if DEBUG and (abs(throttle) > 0.01 or abs(steering) > 0.01):
                            print(f"[TELEOP] t: {throttle:.2f} s: {steering:.2f}")
                
                elif self.drivetrain and not self._teleop_active:
                    # Stop motors when TELEOP is not active
                    self.drivetrain.stop()
                
                time.sleep(SUBSYSTEM_UPDATE_RATE)
            
            except Exception as e:
                print(f"[ERROR] Error in TELEOP control loop: {e}")
                time.sleep(SUBSYSTEM_UPDATE_RATE)

    def _handle_state_change(self, new_state):
        """
        Handle robot state changes and execute mode-specific logic

        Args:
            new_state (State): The new operating state
        """
        old_state = self.robot_data.mode

        with self._data_lock:
            self.robot_data.mode = new_state
            self.robot_data.state_last_update = time.time()

        if DEBUG and old_state != new_state:
            print(f"[CommandBrain] Mode changed: {old_state.name} -> {new_state.name}")

        # ===== MODE-SPECIFIC LOGIC =====
        # This is where future logic for different operating modes will go

        if new_state == State.DISABLED:
            # Emergency stop - disable all systems
            if DEBUG:
                print("[MODE] DISABLED: Emergency stop - all systems disabled")
            self._teleop_active = False
            if self.drivetrain:
                self.drivetrain.stop()

        elif new_state == State.TELEOP:
            # Manual control mode - enable joystick input handling
            if DEBUG:
                print("[MODE] TELEOP: Manual control enabled")
            self._teleop_active = True

        elif new_state == State.AUTONOMOUS:
            # Autonomous/path following mode
            if DEBUG:
                print("[MODE] AUTONOMOUS: Path following enabled")
            self._teleop_active = False
            # TODO: Add logic for autonomous navigation

        elif new_state == State.TEST:
            # Test mode (same as teleop for now)
            if DEBUG:
                print("[MODE] TEST: Test mode enabled (same as teleop)")
            self._teleop_active = True

    def set_mode(self, mode):
        """
        Manually set the robot's operating mode

        Args:
            mode (State): One of DISABLED, AUTONOMOUS, TELEOP, TEST
        """
        self._handle_state_change(mode)

    def get_robot_data(self):
        """
        Get a copy of current robot data (thread-safe)

        Returns:
            RobotBrainData: Current state of all monitored systems
        """
        with self._data_lock:
            # Create a copy so external code doesn't interfere
            data_copy = RobotBrainData()
            data_copy.mode = self.robot_data.mode
            data_copy.battery_voltage = self.robot_data.battery_voltage
            data_copy.battery_current = self.robot_data.battery_current
            data_copy.battery_power = self.robot_data.battery_power
            data_copy.battery_soc = self.robot_data.battery_soc
            data_copy.battery_time_remaining = self.robot_data.battery_time_remaining
            data_copy.controller_data = self.robot_data.controller_data
            data_copy.last_command = self.robot_data.last_command
            data_copy.battery_last_update = self.robot_data.battery_last_update
            data_copy.controller_last_update = self.robot_data.controller_last_update
            data_copy.state_last_update = self.robot_data.state_last_update
            return data_copy

    def get_battery_data(self):
        """Get current battery data as BatteryData object"""
        with self._data_lock:
            return BatteryData(
                voltage=self.robot_data.battery_voltage,
                current=self.robot_data.battery_current,
                power=self.robot_data.battery_power,
                state_of_charge=self.robot_data.battery_soc,
                time_remaining=self.robot_data.battery_time_remaining
            )

    def get_controller_data(self):
        """Get current controller input data as JoystickData object"""
        with self._data_lock:
            return self.robot_data.controller_data

    def get_mode(self):
        """Get current operating mode"""
        with self._data_lock:
            return self.robot_data.mode

    def print_status(self):
        """Print a formatted status report of all systems"""
        with self._data_lock:
            data = self.robot_data

        print("\n" + "="*50)
        print("ROBOT STATUS REPORT")
        print("="*50)
        print(f"Mode: {data.mode.name}")
        print(f"\nBATTERY:")
        print(f"  Voltage: {data.battery_voltage:.2f}V")
        print(f"  Current: {data.battery_current:.2f}A")
        print(f"  Power: {data.battery_power:.1f}W")
        print(f"  SOC: {data.battery_soc:.1f}%")
        print(f"  Time Remaining: {data.battery_time_remaining:.1f} min")
        print(f"\nCONTROLLER:")
        ctrl = data.controller_data
        print(f"  Left Stick: X={ctrl.left_x:.2f}, Y={ctrl.left_y:.2f}")
        print(f"  Right Stick: X={ctrl.right_x:.2f}, Y={ctrl.right_y:.2f}")
        print(f"  D-Pad: U={ctrl.dpad_up}, D={ctrl.dpad_down}, "
              f"L={ctrl.dpad_left}, R={ctrl.dpad_right}")
        print(f"  Buttons: A={ctrl.btn_A}, B={ctrl.btn_B}, "
              f"X={ctrl.btn_X}, Y={ctrl.btn_Y}")
        print("="*50)

    def shutdown(self):
        """Gracefully shutdown the CommandBrain"""
        if DEBUG:
            print("[CommandBrain] Shutting down...")

        self._running = False

        # Wait for all threads to finish (with timeout)
        for thread in self._update_threads:
            thread.join(timeout=2.0)

        # Close serial connection
        if self.controller_ser:
            try:
                self.controller_ser.close()
            except Exception as e:
                print(f"[ERROR] Error closing controller serial: {e}")

        # Close PiControllerReceiver if available
        if self.pi_controller_receiver:
            try:
                self.pi_controller_receiver.close()
            except Exception as e:
                print(f"[ERROR] Error closing controller receiver: {e}")

        if DEBUG:
            print("[CommandBrain] Shutdown complete")


# ==== INTERACTIVE TEST MENU ====
def main():
    """
    Main function for testing CommandBrain with a menu interface.
    This simulates the proof of concept program with actual subsystems.
    """
    print("\n" + "="*50)
    print("BULLSEYE-2 COMMAND BRAIN TEST")
    print("="*50)

    # Initialize subsystems (these would be created in main.py normally)
    try:
        from Robot.subsystems.PiControllerReceiver import PiControllerReceiver
        from Robot.subsystems.DriveTrain import DriveTrain
        from Robot.subsystems.BMS import BMS

        print("[INFO] Initializing subsystems...")
        controller_receiver = PiControllerReceiver()
        drivetrain = DriveTrain()
        bms = BMS()

        brain = CommandBrain(pi_controller_receiver=controller_receiver, drivetrain=drivetrain, bms=bms)
        brain.start()
        print("[OK] CommandBrain started successfully\n")

    except Exception as e:
        print(f"[ERROR] Failed to initialize: {e}")
        print("[INFO] Running in debug mode without hardware\n")
        brain = CommandBrain()
        brain.start()

    debug_mode = False

    try:
        while True:
            print("\n==== COMMAND BRAIN MENU ====")
            print("1. View robot status")
            print("2. Set robot mode")
            print(f"3. Toggle debug mode (currently: {'ON' if debug_mode else 'OFF'})")
            print("4. Get battery data")
            print("5. Get controller data")
            print("6. Exit")

            choice = input("\nSelect option: ").strip()

            if choice == "1":
                brain.print_status()

            elif choice == "2":
                print("\nAvailable modes: DISABLED, AUTONOMOUS, TELEOP, TEST")
                mode_input = input("Enter mode: ").strip().upper()
                try:
                    mode = State[mode_input]
                    brain.set_mode(mode)
                    print(f"Mode set to {mode.name}")
                except KeyError:
                    print("Invalid mode")

            elif choice == "3":
                debug_mode = not debug_mode
                print(f"Debug mode {'enabled' if debug_mode else 'disabled'}")

            elif choice == "4":
                battery = brain.get_battery_data()
                print(f"\n--- Battery Data ---")
                print(f"Voltage: {battery.voltage:.2f}V")
                print(f"Current: {battery.current:.2f}A")
                print(f"Power: {battery.power:.1f}W")
                print(f"SOC: {battery.state_of_charge:.1f}%")
                print(f"Time Remaining: {battery.time_remaining:.1f} min")

            elif choice == "5":
                controller = brain.get_controller_data()
                print(f"\n--- Controller Data ---")
                print(f"Left X: {controller.left_x:.2f}, Left Y: {controller.left_y:.2f}")
                print(f"Right X: {controller.right_x:.2f}, Right Y: {controller.right_y:.2f}")
                print(f"D-Pad: Up={controller.dpad_up}, Down={controller.dpad_down}, "
                      f"Left={controller.dpad_left}, Right={controller.dpad_right}")

            elif choice == "6":
                print("\nExiting...")
                break

            else:
                print("Invalid option")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        brain.shutdown()
        print("[OK] Exited cleanly")


if __name__ == "__main__":
    main()
