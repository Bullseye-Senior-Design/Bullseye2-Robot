import json
import logging
import queue
import serial
import threading

from Robot.Constants import Constants

logger = logging.getLogger(__name__)

# Hardcoded button actions — never overwritten by map updates
_FIXED_BUTTONS = {
    1: "estop",
    2: "start",
}

# Default mapping for configurable buttons 3-8
_DEFAULT_MAP = {
    3: "route:0",
    4: "route:1",
    5: "route:2",
    6: "route:3",
    7: "route:4",
    8: "route:5",
}


class KFX:
    """
    KFX 8-button remote subsystem.

    Runs a daemon listener thread on the UART connected to the KFX receiver
    (GPIO pins 14/15 -> /dev/ttyAMA0). Decodes incoming ASCII button bytes
    and acts on them:

      Button 1 -> sets estop_event   (hardcoded, cannot be remapped)
      Button 2 -> sets start_event   (hardcoded, cannot be remapped)
      Buttons 3-8 -> puts route id into route_queue (configurable)

    The caller (PiCommThread) is responsible for checking these in its main loop.
    Button map for 3-8 is persisted to kfx_map.json so it survives reboots.
    """

    def __init__(self):
        self.estop_event = threading.Event()
        self.start_event = threading.Event()
        self.route_queue: queue.Queue[str] = queue.Queue()

        self._map_lock = threading.Lock()
        self._button_map: dict[int, str] = {}   # buttons 3-8 only
        self._load_map()

        self._ser: serial.Serial | None = None
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Open serial port and start the listener thread."""
        try:
            self._ser = serial.Serial(
                Constants.kfx_serial_port,
                Constants.kfx_baud_rate,
                timeout=1,
            )
            logger.info(f"KFX connected on {Constants.kfx_serial_port} at {Constants.kfx_baud_rate} baud")
        except serial.SerialException as e:
            logger.error(f"KFX could not open {Constants.kfx_serial_port}: {e}")
            return

        self._thread = threading.Thread(
            target=self._listen,
            daemon=True,
            name="KFXListener",
        )
        self._thread.start()
        logger.info("KFX listener thread started")

    def update_map(self, new_mapping: dict[str, str]):
        """
        Update configurable button mappings (buttons 3-8 only).
        Called by PiCommThread when a kfx_map packet arrives from the controller GUI.

        Args:
            new_mapping: dict with string keys ("3"-"8") and "route:<id>" values
        """
        with self._map_lock:
            for key, value in new_mapping.items():
                btn = int(key)
                if btn in _FIXED_BUTTONS:
                    logger.warning(f"KFX: ignoring attempt to remap fixed button {btn}")
                    continue
                if 3 <= btn <= 8:
                    self._button_map[btn] = value
            self._save_map()
        logger.info(f"KFX map updated: {self._button_map}")

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        logger.info("KFX serial closed")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _listen(self):
        """Daemon thread: read one byte at a time, decode, dispatch."""
        while True:
            try:
                byte = self._ser.read(1)
                if not byte:
                    continue
                try:
                    btn = int(byte.decode())   # ASCII '1'-'8' -> int 1-8
                except ValueError:
                    logger.debug(f"KFX: unexpected byte {byte!r}, ignoring")
                    continue

                self._dispatch(btn)

            except serial.SerialException as e:
                logger.error(f"KFX serial error: {e}")
                break
            except Exception as e:
                logger.error(f"KFX listener error: {e}")

    def _dispatch(self, btn: int):
        """Route a button press to the appropriate event or queue."""
        if btn == 1:
            logger.info("KFX: E-Stop button pressed")
            self.estop_event.set()
        elif btn == 2:
            logger.info("KFX: Start button pressed")
            self.start_event.set()
        elif 3 <= btn <= 8:
            with self._map_lock:
                action = self._button_map.get(btn)
            if action:
                logger.info(f"KFX: button {btn} -> {action}")
                self.route_queue.put(action)
            else:
                logger.warning(f"KFX: button {btn} has no mapped action")
        else:
            logger.debug(f"KFX: button {btn} out of range, ignoring")

    def _load_map(self):
        """Load button map from disk, falling back to defaults."""
        path = Constants.kfx_map_path
        try:
            if path.exists():
                with open(path) as f:
                    raw = json.load(f)
                # JSON keys are strings; convert to int
                self._button_map = {int(k): v for k, v in raw.items()}
                logger.info(f"KFX map loaded from {path}")
                return
        except Exception as e:
            logger.warning(f"KFX: could not load map from {path}: {e}")

        self._button_map = dict(_DEFAULT_MAP)
        logger.info("KFX map using defaults")

    def _save_map(self):
        """Persist current button map to disk. Must be called under _map_lock."""
        path = Constants.kfx_map_path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w") as f:
                # Store with string keys so JSON is valid
                json.dump({str(k): v for k, v in self._button_map.items()}, f, indent=2)
        except Exception as e:
            logger.error(f"KFX: could not save map to {path}: {e}")
