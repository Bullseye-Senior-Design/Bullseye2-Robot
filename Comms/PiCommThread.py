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
import math
from typing import Optional
import logging

from Comms.Models.DataPacket import DataPacket
from Comms.Models.ControllerData import ControllerData
from Comms.Models.BatteryData import BatteryData
from Comms.Models.PathCreated import PathCreated
from Comms.Models.StateData import State, StateData
from Comms.Models.PingAckData import PingAckData
from Comms.Models.BoundaryData import BoundaryData
from Comms.Models.PosData import PosData
from Comms.Models.HomeCheckResult import HomeCheckResult
from Comms.Models.KFXSpeedData import KFXSpeedData
from Comms.Models.PingAckData import PingAckData
from Comms.Models.KFXSpeedData import KFXSpeedData
from Comms.KFX import KFXController
from Robot.Constants import Constants
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from structure.RobotState import RobotState

from helpers.ArenaBoundaryHelper import ArenaBoundaryManager
from helpers.HomePositionHelper import HomePositionManager

import math

from Robot.Constants import Constants

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

# ==== CONNECTION WATCHDOG ====
# Set to False to disable the idle-timeout / connection-check behaviour entirely.
ENABLE_CONNECTION_WATCHDOG = True

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

        # KFX run speed (0.0–1.0); updated by 'kfx_speed' packets from Steam Deck
        self.kfx_speed: float = 0.5

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
            kalmanStateEstimator: Instance of KalmanStateEstimator
            bms: BMS subsystem instance
        """
        self.bms = bms
        self.kalman_estimator = KalmanStateEstimator()
        self.comm_data = CommData()
        self.robot_state = RobotState()  # Additional state tracking if needed

        # Thread management
        self._update_threads = []
        self._running = True
        self._data_lock = threading.Lock()

        #watchdog enable
        self.activate_watchdog = False

        # Serial connection for receiving from ControllerMessager (Steam Deck XBee).
        # Assigned inside _receive_controller_commands once the port opens so that
        # KFXController can use it to send kfx_ack packets back to the Deck.
        self.pi_ser = None

        # Timestamp of the last packet received from the Steam Deck (used by watchdog).
        self._last_rx_time = time.time()

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

        # Start connection watchdog thread (only if enabled)
        if ENABLE_CONNECTION_WATCHDOG:
            self.watchdog_thread = threading.Thread(
                target=self._connection_watchdog,
                daemon=True,
                name="ConnectionWatchdog"
            )
            self.watchdog_thread.start()
            logger.info("Started connection watchdog thread")

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

            if button == 7:
                # Example: Button 7 triggers an emergency stop
                self._handle_state_change(State.DISABLED)
            elif button == 8:
                # Example: Button 8 triggers return to home
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
                # logger.info(self.comm_data.controller_data)

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
                        self._last_rx_time = time.time()
                        try:
                            self._process_packet(self.pi_ser, line)
                        except Exception as e:
                            logger.error(f"Error processing packet: {e}")

                except Exception as e:
                    logger.error(f"Error reading controller command: {e}")
                    time.sleep(SUBSYSTEM_UPDATE_RATE)

        except serial.SerialException as e:
            logger.error(f"Could not connect to controller receiver: {e}")

    def _connection_watchdog(self):
        """
        Monitors how long it has been since the last packet was received from the
        Steam Deck.  If silence exceeds connection_idle_timeout, sends a ping.
        If no ping_ack arrives within connection_ack_timeout, disables the robot.
        Runs in its own daemon thread; exits when self._running is False.
        """
        while self._running:
            time.sleep(SUBSYSTEM_UPDATE_RATE)

            if not self.activate_watchdog:   #if the watchdog was not activated, skips skips the loop
                continue

            idle = time.time() - self._last_rx_time
            if idle < Constants.connection_idle_timeout:
                continue  # Still hearing from the Deck — nothing to do.

            # Silence threshold crossed: send a ping and wait for ack.
            # logger.warning(f"No packet received for {idle:.2f}s — sending connection-check ping")
            rx_before_ping = self._last_rx_time
            self._send(DataPacket(type="ping", json_data="{}"))

            # Wait up to connection_ack_timeout for any new packet (ack resets _last_rx_time).
            deadline = time.time() + Constants.connection_ack_timeout
            while time.time() < deadline:
                if self._last_rx_time != rx_before_ping:
                    # A new packet came in — connection restored.
                    # logger.info("Connection-check ping acknowledged — link restored")
                    break
                time.sleep(0.05)
            else:
                # No response within the ack window — disable the robot.
                logger.error("No ping_ack received — disabling robot due to lost connection")
                self._handle_state_change(State.DISABLED)
                self.send_error("Controller timeout, pi disabled")
                # Reset the timer so we don't spam disables every loop tick.
                self._last_rx_time = time.time()

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

        elif packet.type == "ping":
            # Steam Deck is checking for a connection.
            # Respond with ping_ack including current SoC so the Deck can
            # update its battery display on first connect.
            self.activate_watchdog = True #Enables the watchdog after the controller has connected
            self._send_ping_ack()

        elif packet.type == "set_boundary":
            # Steam Deck is sending the 4 arena corner coordinates.
            boundary = BoundaryData.model_validate_json(packet.json_data)
            corners_as_tuples = [(corner.x, corner.y) for corner in boundary.corners]
            ArenaBoundaryManager().set_arena_boundary(corners_as_tuples)
            logger.info(f"Boundary received: {boundary.corners}")
            self._send_boundary_ack()

        elif packet.type == "request_pos":
            # Steam Deck is asking for the robot's current x, y, yaw.
            self._send_pos_data()

        elif packet.type == "set_home":
            # Steam Deck is commanding the robot to update its home position.
            pos = PosData.model_validate_json(packet.json_data)
            
            HomePositionManager().set_home_position(pos.x, pos.y, math.radians(pos.yaw))
           
            logger.info(f"Home position set: x={pos.x}, y={pos.y}, yaw={pos.yaw}")
            self._send_home_ack()

        elif packet.type == "record_home_check":
            # Steam Deck is asking whether the robot is currently at home.
            self._send_record_home_check_result()

        elif packet.type == "request_battery":
            # Steam Deck is explicitly requesting current battery telemetry.
            # Respond immediately with the latest cached BMS data.
            self._send_battery_now()

        elif packet.type == "kfx_speed":
            # Steam Deck is setting the KFX run speed.
            speed_data = KFXSpeedData.model_validate_json(packet.json_data)
            self.comm_data.kfx_speed = speed_data.speed
            logger.info(f"KFX speed updated to {speed_data.speed:.2f}")
            self._send_kfx_speed_ack()

        elif packet.type == "record_start":
            # Steam Deck is starting a path recording session.
            logger.info("record_start received — beginning path recording")
            self._handle_state_change(State.RECORD_PATH)

        elif packet.type == "record_finish":
            # Steam Deck is finishing the path recording session.
            # The Pi will save the path and reply with path_created.
            logger.info("record_finish received — finishing path recording")
            self._handle_state_change(State.DISABLED)

        elif packet.type == "record_cancel":
            # Steam Deck is cancelling an in-progress recording.
            logger.info("record_cancel received — cancelling path recording")
            self._handle_state_change(State.DISABLED)

    def _send(self, packet: DataPacket):
        """Write a DataPacket to the Steam Deck over serial."""
        if self.pi_ser and self.pi_ser.is_open:
            try:
                self.pi_ser.write((packet.model_dump_json() + "\n").encode())
            except Exception as e:
                logger.error(f"Serial write failed: {e}")
        else:
            logger.warning(f"Cannot send {packet.type} — serial not open")

    def _send_ping_ack(self):
        """Respond to a ping with current SoC so the Deck can display it on connect."""
        soc = self.comm_data.battery_data.state_of_charge
        ack = PingAckData(state_of_charge=soc)
        packet = DataPacket(type="ping_ack", json_data=ack.model_dump_json())
        self._send(packet)
        logger.info(f"Sent ping_ack (SoC={soc:.1f}%)")

    def _send_boundary_ack(self):
        """Acknowledge that the boundary was received and applied."""
        packet = DataPacket(type="boundary_ack", json_data="{}")
        self._send(packet)
        logger.info("Sent boundary_ack")

    def _send_pos_data(self):
        """
        Respond to request_pos with current robot position and heading.
        Uses the KalmanStateEstimator's current state estimate to get x, y, yaw.
        """
        x, y, yaw = self.kalman_estimator.get_robot_pose()
        x = math.trunc(x * 100) / 100
        y = math.trunc(y * 100) / 100
        yaw = math.trunc(math.degrees(yaw) * 100) / 100
        pos = PosData(x=x, y=y, yaw=yaw)
        packet = DataPacket(type="pos_data", json_data=pos.model_dump_json())
        self._send(packet)
        logger.info(f"Sent pos_data: {pos}")

    def _send_home_ack(self):
        """Acknowledge that the home position was received and stored."""
        packet = DataPacket(type="home_ack", json_data="{}")
        self._send(packet)
        logger.info("Sent home_ack")

    def _send_record_home_check_result(self):
        """
        Respond to record_home_check with whether the robot is at home.
        """
        current_pos = self.kalman_estimator.get_robot_pose()
        home_pos = HomePositionManager().get_home_position()
        
        if home_pos is None:
            logger.warning("No home position set; cannot perform home check")
            result = HomeCheckResult(ok=False)
            
        at_home = False
            
        if current_pos and home_pos:
            distance = ((current_pos[0] - home_pos.x) ** 2 + (current_pos[1] - home_pos.y) ** 2) ** 0.5
            yaw_diff = abs((current_pos[2] - home_pos.yaw + math.pi) % (2 * math.pi) - math.pi)
            threshold = Constants.distance_to_home_threshold
            at_home = (distance < threshold) and (yaw_diff < Constants.difference_in_heading_to_home_threshold)
        
        result = HomeCheckResult(ok=at_home)
        packet = DataPacket(type="record_home_check_result",
                            json_data=result.model_dump_json())
        self._send(packet)
        logger.info(f"Sent record_home_check_result: ok={result.ok}")

    def _send_battery_now(self):
        """
        Immediately send the latest battery data to the Steam Deck in response
        to an explicit 'request_battery' packet.
        """
        if self.bms:
            self.comm_data.battery_data = self.bms.get_battery_data()
        packet = DataPacket(type="battery",
                            json_data=self.comm_data.battery_data.model_dump_json())
        self._send(packet)
        logger.info(f"Sent battery data on request: SOC={self.comm_data.battery_data.state_of_charge:.1f}%")

    def _send_kfx_speed_ack(self):
        """Acknowledge that the KFX speed update was received and applied."""
        packet = DataPacket(type="kfx_speed_ack", json_data="{}")
        self._send(packet)
        logger.info("Sent kfx_speed_ack")

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

    def send_error(self, errstr: str):
        #TODO: Add error codes and types to distinguish recoverable errors. (NAV)
        """Send a recoverable error message to the Steam Deck for operator display."""
        payload = json.dumps({"errstr": errstr})
        packet = DataPacket(type="error", json_data=payload)
        self._send(packet)
        logger.info(f"Sent error: {errstr}")

    def send_route_finished(self, message: str):
        #TODO: Add route finished notification logic. Route completed or Returned to home string. (NAV)
        """Notify the Steam Deck that a KFX route or return-to-home maneuver completed."""
        payload = json.dumps({"message": message})
        packet = DataPacket(type="route_finished", json_data=payload)
        self._send(packet)
        logger.info(f"Sent route_finished: {message}")

    def send_new_path_data(self, path_id: int):
        """Send new path data to the controller."""
        path_data = PathCreated(id=path_id)
        payload = path_data.model_dump_json()
        data_packet = DataPacket(type="path_created", json_data=payload).model_dump_json()
        if self.pi_ser and self.pi_ser.is_open:
            self.pi_ser.write((data_packet + "\n").encode())
            logger.info(f"Sent new path data to controller: path_id={path_id}")

    def _handle_state_change(self, new_state : State):
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
    
    def get_path_data(self) -> tuple[Optional[float], Optional[int]]:
        """Get current path data (thread-safe)
        Returns:
            tuple: (path_speed, path_id)"""
        with self._data_lock:
            return self.comm_data.state_data.path_speed, self.comm_data.state_data.path_id
        
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
    
    def reset_controller_data(self):
        with self._data_lock:
            self.comm_data.controller_data = ControllerData(
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

