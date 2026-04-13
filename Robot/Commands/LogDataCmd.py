from structure.commands.Command import Command

import os
import time
import numpy as np
from types import SimpleNamespace
from pathlib import Path
from typing import Optional
import shutil

# subsystems
from Robot.subsystems.sensors.UWB import UWB
from Robot.subsystems.sensors.UWBTag import Position
from Robot.subsystems.sensors.IMU import IMU
from Robot.subsystems.sensors.BackWheelEncoder import BackWheelEncoder
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from Robot.subsystems.algorithms.PathFollowing import PathFollowing
from Robot.Constants import Constants

# SQLite-backed data utilities
from helpers.dbConstants import (
    CONTROL_INPUTS_TABLE,
    ENCODER_DATA_TABLE,
    IMU_ORIENTATION_TABLE,
    PATH_FOLLOWING_TABLE,
    STATE_ESTIMATOR_TABLE,
    Table,
    UWB_ANCHORS_TABLE,
    UWB_POSITIONS_TABLE,
)

from helpers.sqllib import SQLiteFileManager

class LogDataCmd(Command):
    def __init__(self, path_following: PathFollowing):
        super().__init__()
        self.path_following = path_following
        self.db_manager = SQLiteFileManager()
    
    def _delete_old_folders(self, base_dir, max_age_days):
        """Delete folders in base_dir that are older than max_age_days.
        
        Assumes folder names follow YYYYMMDD_HHMMSS format.
        """
        base_path = Path(base_dir)
        if not base_path.exists():
            return
        
        current_time = time.time()
        max_age_seconds = max_age_days * 24 * 60 * 60
        
        for folder in base_path.iterdir():
            if not folder.is_dir():
                continue
            
            folder_name = folder.name
            # Parse the folder name as a timestamp
            # Expected format: YYYYMMDD_HHMMSS
            if len(folder_name) >= 15 and folder_name[8] == '_':
                folder_time = time.mktime(time.strptime(folder_name[:15], '%Y%m%d_%H%M%S'))
                age_seconds = current_time - folder_time
                
                if age_seconds > max_age_seconds:
                    print(f"Deleting old folder: {folder}")
                    shutil.rmtree(folder)
        
    def initialize(self):
        # Clean up old log folders (older than 2 days)
        self._delete_old_folders(Constants.logs_directory, max_age_days=2)
        
        # record when logging begins and create a timestamped folder to hold all logs
        self.begin_timestamp = time.time()
        ts_str = time.strftime('%Y%m%d_%H%M%S', time.localtime(self.begin_timestamp))

        # create a dedicated folder under ./logs/<begin_timestamp>/
        base_log_dir = Constants.logs_directory
        self.log_dir = base_log_dir / ts_str
        self.log_dir.mkdir(parents=True, exist_ok=True)

        print(f"LogDataCmd initialized, logging started. Logs will be stored in: {self.log_dir}")
        # Setup SQLite tables using the manager. The tables live in the root
        # robot_data.db file and are recreated on each boot.
        # table keys
        self.uwb_file_path = UWB_POSITIONS_TABLE
        self.anchors_file_path = UWB_ANCHORS_TABLE
        self.state_file_path = STATE_ESTIMATOR_TABLE
        self.imu_file_path = IMU_ORIENTATION_TABLE
        self.cov_file_path = str(self.log_dir / 'ekf_covariance.txt')
        self.encoder_file_path = ENCODER_DATA_TABLE
        self.control_inputs_file_path = CONTROL_INPUTS_TABLE
        self.path_following_file_path = PATH_FOLLOWING_TABLE

        # Setup table mappings with manager
        self.db_manager.setup_file(self.uwb_file_path, replace_existing=True)
        self.db_manager.setup_file(self.anchors_file_path, replace_existing=True)
        self.db_manager.setup_file(self.state_file_path, replace_existing=True)
        self.db_manager.setup_file(self.imu_file_path, replace_existing=True)
        self.db_manager.setup_file(self.encoder_file_path, replace_existing=True)
        self.db_manager.setup_file(self.control_inputs_file_path, replace_existing=True)
        self.db_manager.setup_file(self.path_following_file_path, replace_existing=True)
        
        # Setup covariance text file separately (not CSV)
        self._cov_fh = open(self.cov_file_path, 'a')
        cov_new = os.path.getsize(self.cov_file_path) == 0
        if cov_new:
            self._cov_fh.write(f"# ekf covariance log started at {self.begin_timestamp}\n")
            self._cov_fh.flush()
    
    def execute(self):
        """Sample UWB positions, estimator state, and IMU orientation once and append to CSVs.

        This method is designed to be called repeatedly by the command scheduler
        (it performs one sample per call). It will gracefully skip subsystems
        that are not available/initialized.
        """
        ts = time.time()

        # 1) UWB positions with tag IDs
        uwb = UWB()
        positions = uwb.get_positions() or []
        
        # Log each position with its tag ID
        for position in positions:
            if position is not None:
                self.save_uwb_pos_to_csv(position, position.id, self.uwb_file_path, timestamp=ts)

        # Also record anchor information (text file) for debugging / reference
        anchors = uwb.get_latest_anchor_info()
        self.save_uwb_anchors_to_csv(anchors, self.anchors_file_path, timestamp=ts)

        # 2) State estimator (only log if initialized)
        kf = KalmanStateEstimator()
        # Only log state if the filter has been initialized with first UWB measurement
        if kf.is_initialized:
            state = kf.get_state()
            # estimate euler from estimator for convenience
            euler = kf.euler  # numpy array [roll, pitch, yaw] in radians
            yaw = float(euler[2]) * 180.0 / 3.141592653589793
            pitch = float(euler[1]) * 180.0 / 3.141592653589793
            roll = float(euler[0]) * 180.0 / 3.141592653589793
            self.save_state_to_csv(state, yaw, pitch, roll, self.state_file_path, timestamp=ts)
            # Also log covariance matrix (EKF P)
            self.save_covariance_to_txt(kf.P, self.cov_file_path, timestamp=ts)

        # 3) IMU orientation
        imu = IMU()
        # IMU may be running in its own thread; get_euler returns (heading, roll, pitch)
        roll, pitch, heading = imu.get_angle()
        # get raw sensor measurements (accel, gyro, mag)
        accel = imu.get_acc()
        gyro = imu.get_gyro()
        mag = imu.get_mag()

        orient = SimpleNamespace(timestamp=ts, yaw=heading, pitch=pitch, roll=roll, accel=accel, gyro=gyro, mag=mag)
        # save orientation and raw sensor values
        self.save_orientation_to_csv(orient, self.imu_file_path)

        # 4) Encoder data
        encoder = BackWheelEncoder()
        count = encoder.get_count_left() + encoder.get_count_right()  # Total count since last reset
        velocity = encoder.get_velocity()
        self.save_encoder_to_csv(count, velocity, self.encoder_file_path, timestamp=ts)

        # 5) Control inputs from Kalman State Estimator
        kf = KalmanStateEstimator()
        ctrl_velocity, ctrl_steering = kf.get_control_inputs()
        self.save_control_inputs_to_csv(ctrl_velocity, ctrl_steering, self.control_inputs_file_path, timestamp=ts)

        # 6) Path following data (motor speed and steering angle)
        if self.path_following.is_running():
            v_cmd, delta_cmd = self.path_following.get_current_commands()
            self.save_path_following_to_csv(v_cmd, delta_cmd, self.path_following_file_path, timestamp=ts)
    
    def end(self, interrupted):
        # Close all managed SQLite connections
        self.db_manager.close_all()
        # Close covariance file separately (not managed by manager)
        if not self._cov_fh.closed:
            self._cov_fh.close()
    
    def is_finished(self):
        return False
    def save_uwb_pos_to_csv(self, position: Optional[Position], tag_id: int, table: Table, timestamp: Optional[float] = None) -> bool:
        """Save a position reading to a table with tag ID."""

        def _safe_get(p: Optional[Position], attr, default=''):
            return getattr(p, attr) if p is not None else default

        ts = timestamp if timestamp is not None else time.time()
        row = UWB_POSITIONS_TABLE.build_row(
            timestamp=ts,
            tag_id=tag_id,
            x=_safe_get(position, 'x', ''),
            y=_safe_get(position, 'y', ''),
            z=_safe_get(position, 'z', ''),
            quality=_safe_get(position, 'quality', ''),
        )

        return self.db_manager.write_row(table, row)

    def save_orientation_to_csv(self, orientation, table: Table) -> bool:
        """Save IMU orientation reading to a table."""
        ts = getattr(orientation, 'timestamp', time.time())
        row = IMU_ORIENTATION_TABLE.build_row(
            timestamp=ts,
            yaw=getattr(orientation, 'yaw', ''),
            pitch=getattr(orientation, 'pitch', ''),
            roll=getattr(orientation, 'roll', ''),
            accel=getattr(orientation, 'accel', None),
            gyro=getattr(orientation, 'gyro', None),
            mag=getattr(orientation, 'mag', None),
        )

        return self.db_manager.write_row(table, row)

    def save_uwb_anchors_to_csv(self, anchors_info, table: Table, timestamp: Optional[float] = None) -> bool:
        """Save UWB anchor info into a table."""
        ts = timestamp if timestamp is not None else time.time()

        for port, anchors in anchors_info or []:
            if not anchors:
                row = UWB_ANCHORS_TABLE.build_row(ts, port, '', '', '', '', '', '')
                self.db_manager.write_row(table, row)
            else:
                for anchor in anchors:
                    pos = anchor.get('position', (None, None, None))
                    row = UWB_ANCHORS_TABLE.build_row(
                        timestamp=ts,
                        port=port,
                        name=anchor.get('name', ''),
                        anchor_id=anchor.get('id', ''),
                        x=pos[0] if pos and len(pos) > 0 else '',
                        y=pos[1] if pos and len(pos) > 1 else '',
                        z=pos[2] if pos and len(pos) > 2 else '',
                        range_value=anchor.get('range', ''),
                    )
                    self.db_manager.write_row(table, row)
        return True

    def save_state_to_csv(self, state, yaw: float, pitch: float, roll: float, table: Table, timestamp: Optional[float] = None) -> bool:
        """Save estimator state (pos, vel, euler) to a table."""
        ts = timestamp if timestamp is not None else time.time()
        px, py, pz = state.pos
        vx, vy, vz = state.vel

        row = STATE_ESTIMATOR_TABLE.build_row(ts, px, py, pz, vx, vy, vz, yaw, pitch, roll)
        return self.db_manager.write_row(table, row)

    def save_covariance_to_txt(self, P, filename: str, timestamp: Optional[float] = None) -> bool:
        """Save covariance matrix in a readable text file.

        The file will contain a timestamp header followed by the matrix rows.
        Multiple calls append additional blocks (so the file contains history).
        """
        filename = str(filename)
        file_exists = os.path.exists(filename)

        P_arr = np.asarray(P)
        if P_arr.ndim != 2 or P_arr.shape[0] != P_arr.shape[1]:
            print(f"Covariance must be a square 2D array, got shape {P_arr.shape}")
            return False

        ts = timestamp if timestamp is not None else time.time()

        with open(filename, 'a') as f:
            f.write(f"# timestamp: {ts}\n")
            for row in P_arr:
                f.write('  '.join(f"{val: .6e}" for val in row) + "\n")
            f.write("\n")

        if not file_exists:
            print(f"Created new covariance TXT file: {filename}")

        return True

    def save_encoder_to_csv(self, count: int, velocity: float, table: Table, timestamp: Optional[float] = None) -> bool:
        """Save encoder count and velocity to a table."""
        ts = timestamp if timestamp is not None else time.time()
        row = ENCODER_DATA_TABLE.build_row(ts, count, velocity)
        return self.db_manager.write_row(table, row)

    def save_control_inputs_to_csv(self, velocity: float, steering_angle: float, table: Table, timestamp: Optional[float] = None) -> bool:
        """Save control inputs (velocity and steering angle) to a table."""
        ts = timestamp if timestamp is not None else time.time()
        steering_angle_deg = float(steering_angle) * 180.0 / np.pi
        row = CONTROL_INPUTS_TABLE.build_row(ts, velocity, steering_angle, steering_angle_deg)
        return self.db_manager.write_row(table, row)

    def save_path_following_to_csv(self, motor_speed: float, steering_angle: float, table: Table, timestamp: Optional[float] = None) -> bool:
        """Save path following motor speed and steering angle to a table."""
        ts = timestamp if timestamp is not None else time.time()
        steering_angle_deg = float(steering_angle) * 180.0 / np.pi
        row = PATH_FOLLOWING_TABLE.build_row(ts, motor_speed, steering_angle, steering_angle_deg)
        return self.db_manager.write_row(table, row)