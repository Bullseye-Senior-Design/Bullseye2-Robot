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

from Comms.Models.DataPacket import DataPacket
from Comms.Models.ControllerData import ControllerData
from Comms.Models.BatteryData import BatteryData
from Comms.Models.StateData import State, StateData
from Comms.KFX import KFXController
from Robot.Constants import Constants
from structure.RobotState import RobotState

# Reverse of the sender's _FIELD_MAP — short key -> full ControllerData field name
_FIELD_UNMAP = {
    'lx': 'left_x',  'ly': 'left_y',  'rx': 'right_x', 'ry': 'right_y',
    'du': 'dpad_up',  'dd': 'dpad_down', 'dl': 'dpad_left', 'dr': 'dpad_right',
    'ba': 'btn_A',   'bb': 'btn_B',   'bx': 'btn_X',   'by': 'btn_Y',
    'lb': 'btn_LB',  'rb': 'btn_RB',  'ls': 'btn_LS',  'rs': 'btn_RS',
    'r2': 'btn_R2',  'l2': 'btn_L2',  'sh': 'btn_share', 'op': 'btn_options',
}

# ==== LOGGING CONFIGURATION ====
logger = logging.getLogger(f"{__name__}.PiCommThread")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output

# ==== DEBUG/CONFIGURATION ====
SUBSYSTEM_UPDATE_RATE = Constants.controller_update_rate  # ~10 Hz for subsystem polling
BATTERY_SOC_THRESHOLD = 0.5  # Send battery data when SOC drops by this %
BATTERY_CHECK_RATE = 5.0     # How often to poll SOC for threshold change (seconds)

# ==== ROBOT STATE DATA ====
class CommData:
    """Holds all the robot's current state and sensor data"""
    _instance = None

    # When a new instance is created, sets it to the same global instance
    def __new__(cls):
        # If the instance is None, create a new instance
        # Otherwise, return already created instance
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._start()
        return cls._instance
    
    def _start(self):
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
            btn_R2=-1.0, btn_L2=-1.0,
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

        # Serial connection for receiving from ControllerMessager (Steam Deck XBee).
        # Assigned inside _receive_controller_commands once the port opens so that
        # KFXController can use it to send kfx_ack packets back to the Deck.
        self.pi_ser = None

        # KFXController is created here but NOT started yet – we start it in
        # start() so that pi_ser is guaranteed to be assigned before KFX needs
        # it.  pi_ser may still be None at start() time if the port fails, which
        # is fine – KFXController skips the ack and logs a warning.
        self.kfx = KFXController(
            robot_state=self.robot_state,
            comm_data=self.comm_data,
            data_lock=self._data_lock,
            pi_ser=None,   # Updated to the real serial object once port opens
        )

        logger.info("PiCommThread initialized")

    def start(self):
        # Start controller receiver thread.
        # This also opens pi_ser; once that succeeds _receive_controller_commands
        # updates self.kfx.pi_ser so KFX ack packets can reach the Steam Deck.
        self.comms_thread = threading.Thread(
            target=self._receive_controller_commands,
            daemon=True,
            name="ControllerReceiver"
        )
        self.comms_thread.start()
        logger.info("Started controller receiver thread")

        # Start KFX remote listener thread.
        # KFX runs independently of the Steam Deck connection – the rider can
        # press buttons regardless of whether the Deck is communicating.
        self.kfx_thread = threading.Thread(
            target=self._receive_kfx_commands,
            daemon=True,
            name="KFXListener"
        )
        self.kfx_thread.start()
        logger.info("Started KFX remote listener thread")

        # Start BMS update thread (polls SmartShunt and keeps battery_data fresh)
        if self.bms:
            self.bms_thread = threading.Thread(
                target=self.bms.update,
                daemon=True,
                name="BMSUpdate"
            )
            self.bms_thread.start()
            logger.info("Started BMS update thread")

        # Start battery sender thread
        self.battery_thread = threading.Thread(
            target=self._send_battery_data,
            daemon=True,
            name="BatterySender"
        )
        self.battery_thread.start()
        logger.info("Started battery sender thread")

    def _receive_kfx_commands(self):
        """
        Continuously receive KFX button state updates over serial.
        Runs in background thread.
        """
        while self._running:
            button = self.kfx.listen()

            if button is None:
                continue  # No button data received, skip

            self.kfx.update_button_data(button)

            if button == 8:
                # Example: Button 8 triggers an emergency stop
                self._handle_state_change(State.DISABLED)
            elif button == 7:
                # Example: Button 7 triggers return to home
                self._handle_state_change(State.RETURN_TO_HOME)

                

             

    def _receive_controller_commands(self):
        """
        Continuously receive commands from ControllerMessager over serial.
        Runs in background thread.
        """
        try:
            # Initialize serial connection to receive from ControllerMessager
            self.pi_ser = serial.Serial(Constants.pi_serial_port, Constants.serial_baud_rate, timeout=1)
            logger.info(f"Connected to controller receiver on {Constants.pi_serial_port}")

            # Give KFXController a reference to the now-open serial port so it
            # can send kfx_ack packets back to the Steam Deck.
            self.kfx.pi_ser = self.pi_ser

            buffer = ""  # Accumulate incoming data

            while self._running:
                try:
                    # Read line from serial
                    if(self.pi_ser.in_waiting == 0):
                        logger.debug("No data available from controller")
                        time.sleep(SUBSYSTEM_UPDATE_RATE)
                        continue

                    chunk = self.pi_ser.read(self.pi_ser.in_waiting).decode(errors="ignore")
                    buffer += chunk

                    # Split on newlines and process each complete line
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            self._process_packet(self.pi_ser, line)
                        except Exception as e:
                            logger.error(f"Error processing packet: {e}")

                except Exception as e:
                    logger.error(f"Error reading controller command: {e}")
                    time.sleep(SUBSYSTEM_UPDATE_RATE)

        except serial.SerialException as e:
            logger.error(f"Could not connect to controller receiver: {e}")

    def _process_packet(self, serial_port, line: str):
        """Process a single received packet line."""
        logger.debug(f"data received {line}")

        packet = DataPacket.model_validate_json(line)

        if packet.type == "state":
            state_data = StateData.model_validate_json(packet.json_data)
            self._handle_state_change(state_data.state)
            self.comm_data.state_data = state_data

        elif packet.type == "c":
            delta = json.loads(packet.json_data)
            current = self.comm_data.controller_data.model_dump()
            for short_key, value in delta.items():
                full_key = _FIELD_UNMAP.get(short_key)
                if full_key:
                    current[full_key] = value
            self.comm_data.controller_data = ControllerData(**current)

        elif packet.type == "kfx_config":
            # Steam Deck is sending updated KFX button assignments.
            # Forward to KFXController which persists them and sends kfx_ack.
            new_config = json.loads(packet.json_data)
            self.kfx.update_config(new_config)
            logger.info(f"KFX config received and forwarded: {new_config}")

    def _send_battery_data(self):
        """
        Sends battery data to the controller when SOC has dropped by >= 0.5%
        since the last send. Checks every 5 seconds.
        Runs in background thread, decoupled from controller message receive rate.
        """
        last_sent_soc = None

        while self._running:
            time.sleep(BATTERY_CHECK_RATE)
            try:
                if self.bms:
                    self.comm_data.battery_data = self.bms.get_battery_data()

                b = self.comm_data.battery_data
                logger.debug(f"[BATTERY] V:{b.voltage:.2f}V  I:{b.current:.2f}A  P:{b.power:.1f}W  SOC:{b.state_of_charge:.1f}%  TTG:{b.time_remaining:.0f}min")

                current_soc = self.comm_data.battery_data.state_of_charge

                soc_dropped = (
                    last_sent_soc is None or
                    abs(current_soc - last_sent_soc) >= BATTERY_SOC_THRESHOLD
                )

                if soc_dropped and self.pi_ser and self.pi_ser.is_open:
                    payload = self.comm_data.battery_data.model_dump_json()
                    data_packet = DataPacket(type="battery", json_data=payload).model_dump_json()
                    self.pi_ser.write((data_packet + "\n").encode())
                    last_sent_soc = current_soc
                    logger.debug(f"Sent battery data — SOC: {current_soc:.1f}%")
            except Exception as e:
                logger.error(f"Error sending battery data: {e}")

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

        elif new_state == State.RETURN_TO_HOME:
            # Return to home mode
            logger.info("MODE RETURN_TO_HOME: Returning to home position")
            self.robot_state.enable_return_to_home()

        elif new_state == State.RECORD_PATH:
            # Record path mode
            logger.info("MODE RECORD_PATH: Recording path")
            self.robot_state.enable_path_creation()

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
        
    def get_kfx_data(self):
        """Get current KFX button state data (thread-safe)"""
        with self._data_lock:
            return self.kfx.get_button_data()
        
    def reset_kfx_button_data(self):
        """Reset KFX button assignments to defaults"""
        with self._data_lock:
            self.kfx.reset_button_data()

    def close(self):
        """Gracefully shutdown the CommandBrain"""
        logger.info("CommandBrain shutting down...")

        self._running = False

        # Stop KFX listener thread
        self.kfx.close()

        # Wait for all threads to finish (with timeout)
        self.comms_thread.join(timeout=2.0)
        self.battery_thread.join(timeout=2.0)

        # Close serial connection
        if self.pi_ser:
            try:
                self.pi_ser.close()
            except Exception as e:
                logger.error(f"Error closing controller serial: {e}")

        logger.info("CommandBrain shutdown complete")

