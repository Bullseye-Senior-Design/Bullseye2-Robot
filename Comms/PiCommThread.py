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
import logging

from Comms.DataPacket import DataPacket
from Comms.ControllerData import ControllerData
from Comms.BatteryData import BatteryData
from Comms.StateData import State, StateData
from Robot.Constants import Constants
from structure.RobotState import RobotState

# ==== LOGGING CONFIGURATION ====
logger = logging.getLogger(f"{__name__}.PiCommThread")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output

# ==== DEBUG/CONFIGURATION ====
SUBSYSTEM_UPDATE_RATE = Constants.controller_update_rate  # ~10 Hz for subsystem polling

# ==== ROBOT STATE DATA ====
class CommData:
    """Holds all the robot's current state and sensor data"""
    def __init__(self):
        # Battery data from BMS
        self.battery_data = BatteryData(
            voltage=0.0,
            current=0.0,
            power=0.0,
            state_of_charge=0.0,
            time_remaining=0.0
        )

        # Controller input data (from PiControllerReceiver)
        self.controller_data = ControllerData(
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

        self.state_data = StateData(
            state=State.DISABLED,
            path_speed=0.0,
            path_id=0
        )

        # Last received command
        self.last_command = "None"

        # System timestamps
        self.battery_last_update = 0.0
        self.controller_last_update = 0.0
        self.state_last_update = 0.0


class PiCommThread:
    """
    Central coordinator for the Bullseye-2 robot system.

    Manages:
    - Continuous monitoring of subsystems (BMS, ControllerMessager)
    - Local state and data caching
    - Robot mode transitions
    - Thread-safe access to shared data
    """

    def __init__(self, bms=None):
        """
        Initialize PiCommThread with references to subsystems

        Args:
            bms: BMS subsystem instance
        """
        self.bms = bms
        self.comm_data = CommData()
        self.robot_state = RobotState()  # Additional state tracking if needed

        # Thread management
        self._update_threads = []
        self._running = True
        self._data_lock = threading.Lock()

        # Serial connection for receiving from ControllerMessager
        self.controller_ser = None

        logger.info("PiCommThread initialized")

    def start(self):
        # Start controller receiver thread
        self.comms_thread = threading.Thread(
            target=self._receive_controller_commands,
            daemon=True,
            name="ControllerReceiver"
        )
        self.comms_thread.start()
        logger.info("Started controller receiver thread")

    def _receive_controller_commands(self):
        """
        Continuously receive commands from ControllerMessager over serial.
        Runs in background thread.
        """
        try:
            # Initialize serial connection to receive from ControllerMessager
            self.controller_ser = serial.Serial(Constants.controller_serial_port, Constants.serial_baud_rate, timeout=1)
            logger.info(f"Connected to controller receiver on {Constants.controller_serial_port}")

            while self._running:
                try:
                    # Read line from serial
                    line = self.controller_ser.readline().decode().strip()

                    packet = DataPacket.model_validate_json(line)

                    if(packet.type == "state"):
                        state_data = StateData.model_validate_json(packet.json_data)
                        self._handle_state_change(state_data.state)
                        self.comm_data.state_data = state_data
                    elif(packet.type == "controller"):
                        controller_data = ControllerData.model_validate_json(packet.json_data)
                        self.comm_data.controller_data = controller_data

                    if self.bms:
                        self.comm_data.battery_data = self.bms.get_battery_data()

                    payload = self.comm_data.battery_data.model_dump_json()  # Serialize controller data to JSON string
                    data_packet = DataPacket(type="battery", json_data=payload).model_dump_json()  # Wrap in DataPacket and serialize to JSON string
                    
                    self.controller_ser.write((data_packet + "\n").encode())  # Send to ControllerMessager

                except Exception as e:
                    logger.error(f"Error reading controller command: {e}")

                time.sleep(SUBSYSTEM_UPDATE_RATE)  # Small sleep to prevent CPU hogging

        except serial.SerialException as e:
            logger.error(f"Could not connect to controller receiver: {e}")

    def _handle_state_change(self, new_state):
        """
        Handle robot state changes and execute mode-specific logic

        Args:
            new_state (State): The new operating state
        """
        old_state = self.comm_data.state_data.state

        with self._data_lock:
            self.comm_data.state_data.state = new_state
            self.comm_data.state_last_update = time.time()

        if old_state != new_state:
            logger.info(f"Mode changed: {old_state.name} -> {new_state.name}")

        # ===== MODE-SPECIFIC LOGIC =====
        # This is where future logic for different operating modes will go

        if new_state == State.DISABLED:
            # Emergency stop - disable all systems
            logger.info("MODE DISABLED: Emergency stop - all systems disabled")
            self.robot_state.disable_robot() 

        elif new_state == State.TELEOP:
            # Manual control mode - enable joystick input handling
            logger.info("MODE TELEOP: Manual control enabled")
            self.robot_state.enable_teleop()

        elif new_state == State.AUTONOMOUS:
            # Autonomous/path following mode
            logger.info("MODE AUTONOMOUS: Path following enabled")
            self.robot_state.enable_autonomous()

        elif new_state == State.TEST:
            # Test mode (same as teleop for now)
            logger.info("MODE TEST: Test mode enabled (same as teleop)")
            self.robot_state.enable_teleop()

    def get_robot_data(self):
        """
        Get a copy of current robot data (thread-safe)

        Returns:
            RobotBrainData: Current state of all monitored systems
        """
        with self._data_lock:
            return self.comm_data
        
    def get_battery_data(self):
        """Get current battery data (thread-safe)"""
        with self._data_lock:
            return self.comm_data.battery_data
        
    def get_controller_data(self):
        """Get current controller data (thread-safe)"""
        with self._data_lock:
            return self.comm_data.controller_data

    def close(self):
        """Gracefully shutdown the CommandBrain"""
        logger.info("CommandBrain shutting down...")

        self._running = False

        # Wait for all threads to finish (with timeout)
        self.comms_thread.join(timeout=2.0)

        # Close serial connection
        if self.controller_ser:
            try:
                self.controller_ser.close()
            except Exception as e:
                logger.error(f"Error closing controller serial: {e}")

        logger.info("CommandBrain shutdown complete")

