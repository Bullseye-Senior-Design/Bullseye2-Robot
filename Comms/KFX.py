"""
Comms/KFX.py  –  KFX Remote Controller Handler (Pi side)
==========================================================
Listens on /dev/ttyAMA0 (UART GPIO 14/15 on Pi 5) for single-byte
button press events from the KFX handheld remote the rider carries.

The KFX remote sends one ASCII byte per button press:
  Button 1 → 49  ('1')
  Button 2 → 50  ('2')
  ...
  Button 8 → 56  ('8')

Button assignments (hardcoded vs configurable):
  Button 1  = DISABLED  – always stops the robot, never reassignable
  Button 2  = Start 
  Buttons 3–8 = user-configured routes, received from the Steam Deck
                as a 'kfx_config' DataPacket and persisted locally

PiCommThread owns this class: it instantiates it, calls start(), and
forwards 'kfx_config' packets to update_config(). KFXController then
sends a 'kfx_ack' packet back so the Steam Deck knows the config was
accepted and can unlock its own local save.
"""

import serial
import threading
import json
import logging
from pathlib import Path

from Comms.Models.KFXButtonData import KFXButtonData
from Comms.Models.DataPacket import DataPacket
from Comms.Models.StateData import State, StateData

logger = logging.getLogger(f"{__name__}.KFXController")
logger.setLevel(logging.DEBUG)  # Set to INFO for high-level events; use DEBUG for more detail

# ── Hardware constants ────────────────────────────────────────────────────────
KFX_PORT = "/dev/ttyAMA0"   # UART on GPIO pins 14 (TX) / 15 (RX) – Pi 5
KFX_BAUD = 115200            # Must match what the KFX receiver module uses

# ── Byte → button mapping ─────────────────────────────────────────────────────
# The KFX remote sends ASCII character bytes for each button.
# Confirmed via kfx_test.py: button N sends ASCII byte N+48.
# If the remote ever sends different values, only this dict needs updating.
KFX_BYTE_TO_BUTTON: dict[int, int] = {
    49: 1,   # ASCII '1'
    50: 2,   # ASCII '2'
    51: 3,   # ASCII '3'
    52: 4,   # ASCII '4'
    53: 5,   # ASCII '5'
    54: 6,   # ASCII '6'
    55: 7,   # ASCII '7'
    56: 8,   # ASCII '8'
}

# ── Config persistence ────────────────────────────────────────────────────────
# Stored at project root so it is easy to inspect and survives Pi reboots.
KFX_CONFIG_FILE = Path(__file__).parent.parent / "kfx_config.json"

# Default config when no file exists yet (all buttons 3-8 unassigned).
# Keys are strings because JSON only supports string keys.
_DEFAULT_CONFIG: dict[str, int | None] = {
    "3": None, "4": None, "5": None,
    "6": None, "7": None, "8": None,
}

# Path speed used when the KFX triggers a route or return-home.
# TODO: Make this configurable from the Steam Deck once the speed-per-route
#       feature is designed. For now all KFX-triggered routes run at 50%.
KFX_DEFAULT_SPEED = 0.5


class KFXController:
    


    """
    Manages the KFX remote receiver thread and button-to-action dispatch.

    Lifecycle (called by PiCommThread):
      1. __init__()       – pass in robot_state, comm_data, data_lock, pi_ser
      2. start()          – opens /dev/ttyAMA0, launches listener thread
      3. update_config()  – called whenever a 'kfx_config' packet arrives
      4. stop()           – signals listener thread to exit cleanly
    """

    def __init__(self, robot_state, comm_data, data_lock: threading.Lock,
                 pi_ser: serial.Serial | None):
        """
        Args:
            robot_state  RobotState instance – used to call enable_*/disable_robot()
            comm_data    CommData instance   – updated with new StateData on button press
            data_lock    Lock protecting comm_data (same lock PiCommThread uses)
            pi_ser       Serial port open to Steam Deck – used to send kfx_ack packets
                         May be None if the port failed to open; ack will be skipped.
        """
        self.robot_state = robot_state
        self.comm_data   = comm_data
        self.data_lock   = data_lock
        self.pi_ser      = pi_ser

        self._running     = True
        self._config_lock = threading.Lock()   # Separate lock – config can update while
                                               # data_lock is held for state changes
        self._config: dict[str, int | None] = self._load_config()

        self.kfx_button_data = KFXButtonData(
            btn_1=False,
            btn_2=False,
            btn_3=False,
            btn_4=False,
            btn_5=False,
            btn_6=False,
            btn_7=False,
            btn_8=False
        )

        logger.info("KFXController initialised – config: %s", self._config)

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self):
        """Launch the background UART listener thread."""
        self._thread = threading.Thread(
            target=self._listen,
            daemon=True,
            name="KFXListener",
        )
        self._thread.start()
        logger.info("KFX listener thread started on %s @ %d baud", KFX_PORT, KFX_BAUD)

    def stop(self):
        """Signal the listener loop to exit on its next timeout cycle."""
        self._running = False

    def update_config(self, new_config: dict):
        """
        Called by PiCommThread when a 'kfx_config' DataPacket arrives
        from the Steam Deck.

        Updates buttons 3–8 from the received dict (buttons 1–2 are fixed
        and are never included in the packet). Persists to disk immediately
        so the assignment survives a reboot. Sends 'kfx_ack' back so the
        Steam Deck can confirm its own local save.

        Args:
            new_config  Parsed dict e.g. {"3": 5, "4": None, ..., "8": 2}
        """
        with self._config_lock:
            for btn in ("3", "4", "5", "6", "7", "8"):
                self._config[btn] = new_config.get(btn)   # None if key absent
            self._save_config()

        logger.info("KFX config updated: %s", self._config)

        # Acknowledge back to Steam Deck – must arrive before the Deck
        # unlocks its local config save so both sides always match.
        self._send_ack()

    # ── Config persistence ────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        """
        Read kfx_config.json from disk. Falls back to all-None defaults
        if the file does not exist or cannot be parsed.
        Missing keys are filled with None so the dict always has all 6 entries.
        """
        if KFX_CONFIG_FILE.exists():
            try:
                with open(KFX_CONFIG_FILE, "r") as f:
                    data = json.load(f)
                for k in _DEFAULT_CONFIG:
                    data.setdefault(k, None)
                logger.info("KFX config loaded from %s", KFX_CONFIG_FILE)
                return data
            except Exception as e:
                logger.error("Could not load kfx_config.json: %s", e)
        logger.info("No KFX config file found – using defaults (all unassigned)")
        return dict(_DEFAULT_CONFIG)

    def _save_config(self):
        """
        Write current _config to kfx_config.json.
        Called inside _config_lock – do NOT acquire the lock again here.
        """
        try:
            with open(KFX_CONFIG_FILE, "w") as f:
                json.dump(self._config, f, indent=2)
        except Exception as e:
            logger.error("Could not save kfx_config.json: %s", e)

    # ── Serial communication ──────────────────────────────────────────────────

    def _send_ack(self):
        """
        Send a 'kfx_ack' DataPacket to the Steam Deck over pi_ser.
        The Steam Deck SerialRXThread is watching for this and will only
        commit the new config locally once it arrives.
        """
        if self.pi_ser and self.pi_ser.is_open:
            try:
                packet = DataPacket(type="kfx_ack", json_data="{}")
                self.pi_ser.write((packet.model_dump_json() + "\n").encode())
                logger.info("Sent kfx_ack to Steam Deck")
            except Exception as e:
                logger.error("Could not send kfx_ack: %s", e)
        else:
            logger.warning("kfx_ack skipped – pi_ser not available")

    # ── Listener thread ───────────────────────────────────────────────────────

    def _listen(self):
        """
        Background thread entry point. Opens /dev/ttyAMA0 and reads one byte
        at a time. Each byte is looked up in KFX_BYTE_TO_BUTTON; recognised
        buttons are dispatched immediately. Unknown bytes are logged and ignored.

        The serial.read(1) call blocks for up to 1 second (timeout=1) before
        returning an empty bytes object, which lets us check self._running
        regularly without spinning the CPU.
        """
        try:
            ser = serial.Serial(KFX_PORT, KFX_BAUD, timeout=1)
            logger.info("KFX port open: %s", KFX_PORT)
        except serial.SerialException as e:
            logger.error("KFX: could not open %s: %s", KFX_PORT, e)
            return

        try:
            while self._running:
                byte = ser.read(1)   # Blocks up to 1 s then returns b''
                if not byte:
                    continue         # Timeout – loop back and check _running

                raw    = byte[0]
                button = KFX_BYTE_TO_BUTTON.get(raw)

                if button is None:
                    logger.debug("KFX: unrecognised byte 0x%02x – ignored", raw)
                    continue

                logger.info("KFX button %d pressed (byte=%d)", button, raw)
                self._dispatch(button)

        except Exception as e:
            logger.error("KFX listener crashed: %s", e)
        finally:
            ser.close()
            logger.info("KFX port closed")

    # ── Button dispatch ───────────────────────────────────────────────────────

    def _dispatch(self, button: int):
        """
        Map a button number to a robot action and execute it.

        Button 1 – DISABLED (hardcoded stop – always works regardless of config)
        Button 2 – Start Route
        Buttons 3-8 – AUTONOMOUS with the route_id from config,
                      or silently ignored if the button is unassigned.
        """

        logger.debug(f"Dispatching KFX button {button} with current config: {self._config}")

        if button == 1:
            self.kfx_button_data.btn_1 = True
        elif button == 2:
            self.kfx_button_data.btn_2 = True
        elif button == 3:
            self.kfx_button_data.btn_3 = True
        elif button == 4:
            self.kfx_button_data.btn_4 = True
        elif button == 5:
            self.kfx_button_data.btn_5 = True
        elif button == 6:
            self.kfx_button_data.btn_6 = True
        elif button == 7:
            self.kfx_button_data.btn_7 = True
        elif button == 8:
            self.kfx_button_data.btn_8 = True

        
            
        # if button == 1:
        #     # ── STOP ─────────────────────────────────────────────────────────
        #     logger.info("KFX dispatch: Button 1 → DISABLED (stop)")
        #     self._apply_state(State.DISABLED, path_id=None, path_speed=None)

        # elif button == 2:
        #     # ── START ROUTE ───────────────────────────────────────────────────
        #     logger.info("KFX dispatch: Button 2 → Start")
        #     #TODO: Logic for start Route

        # else:
        #     # ── CONFIGURED ROUTE ──────────────────────────────────────────────
        #     btn_str = str(button)
        #     with self._config_lock:
        #         route_id = self._config.get(btn_str)

        #     if route_id is None:
        #         logger.info("KFX dispatch: Button %d is unassigned – ignored", button)
        #         return

        #     logger.info("KFX dispatch: Button %d → AUTONOMOUS route_id=%d", button, route_id)
        #     self._apply_state(State.AUTONOMOUS,
        #                       path_id=route_id,
        #                       path_speed=KFX_DEFAULT_SPEED)

    def get_button_data(self) -> KFXButtonData:
        """Returns the latest button data."""
        return self.kfx_button_data
    
    def reset_button_data(self):
        """Resets all button states to False."""
        self.kfx_button_data = KFXButtonData(
            btn_1=False,
            btn_2=False,
            btn_3=False,
            btn_4=False,
            btn_5=False,
            btn_6=False,
            btn_7=False,
            btn_8=False
        )

    def _apply_state(self, state: State,
                     path_id: int | None,
                     path_speed: float | None):
        """
        Write the new state into CommData (under data_lock) and call the
        appropriate RobotState method so the rest of the system reacts.

        Mirrors the logic in PiCommThread._handle_state_change so both
        code paths (Steam Deck commands and KFX button presses) produce
        identical system behaviour.
        """
        new_state_data = StateData(
            state=state,
            path_id=path_id,
            path_speed=path_speed,
        )

        with self.data_lock:
            self.comm_data.state_data = new_state_data

        # Trigger the RobotState transition outside the lock to avoid
        # holding it longer than necessary.
        if state == State.DISABLED:
            self.robot_state.disable_robot()
        elif state == State.AUTONOMOUS:
            self.robot_state.enable_autonomous()
        elif state == State.TELEOP:
            self.robot_state.enable_teleop()
