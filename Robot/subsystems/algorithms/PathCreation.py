import logging
import math
from pathlib import Path

from dataclasses import dataclass

from Robot.Constants import Constants
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from structure.Subsystem import Subsystem
from scipy.interpolate import splrep, splev
from scipy.signal import savgol_filter
import numpy as np
from helpers.dbConstants import PathPointsTable
from helpers.sqllib import ROBOT_DATA_DB_FILENAME, SQLiteFileManager

logger = logging.getLogger(f"{__name__}.PathFollowing")
logger.setLevel(logging.INFO)

@dataclass
class DataPoint:
    x: float
    y: float
    yaw: float

class PathCreation(Subsystem):
    def __init__(self):
        super().__init__()
        self.kf = KalmanStateEstimator()
        self.logs_dir = Constants.path_file_directory

    def start_path_creation(self):
        logger.info("Path creation started")
        
        self._path : list[DataPoint] = []

    def stop_path_creation(self):
        logger.info("Path creation stopped")
        if len(self._path) > 0:
            self.simplify_path()
            return self.current_path_number
        else:
            logger.error("No path points were recorded, nothing to save.")
            return None

        
    def save_path_to_db(self):
        if not self._path:
            return
        
        self.logs_dir.mkdir(parents=True, exist_ok=True)

        db_manager = SQLiteFileManager()
        try:
            next_number, table_key = db_manager.next_numeric_table_key(self.logs_dir)
            self.current_path_number = next_number
            path_table = PathPointsTable(name=str(next_number))
            db_manager.setup_file(path_table)
            rows = [path_table.build_row(point.x, point.y, point.yaw) for point in self._path]
            db_manager.write_rows(path_table, rows)
        finally:
            db_manager.close_all()

        self._saved_file_path = Path.cwd() / ROBOT_DATA_DB_FILENAME
        logger.info(f"Saved {len(self._path)} path points to SQLite table {table_key}")

    # Backward-compatible name for existing callers.
    def save_path_to_csv(self):
        self.save_path_to_db()

    def add_path_point(self):
        if self.kf.is_initialized:
            currentPos = DataPoint(x=0.0, y=0.0, yaw=0.0)
            state = self.kf.get_state()
            currentPos.x = float(state.pos[0])  # x position in meters
            currentPos.y = float(state.pos[1])  # y position in meters
            euler = self.kf.euler  # numpy array [roll, pitch, yaw] in radians
            currentPos.yaw = float(euler[2])
            self._path.append(currentPos)

    def simplify_path(
        self,
        group_distance: float = 0.10, # meters, distance threshold for grouping nearby points before smoothing
        smoothing: float = 0.5, # smoothing factor for B-spline fit (0 = interpolate, higher = smoother)
        num_samples: int = 100, # number of points to sample along the smoothed path
        savgol_window: int = 11, # window length for Savitzky-Golay filter (must be odd and >= polyorder+2)
        savgol_polyorder: int = 3, # polynomial order for Savitzky-Golay filter
        unwrap_yaw: bool = True, # whether to unwrap yaw angles to prevent discontinuities before smoothing
        wrap_yaw_to_pi: bool = True, # whether to wrap final yaw angles to [-pi, pi] after smoothing
        yaw_min_motion_distance: float = 1e-3, # minimum distance between points to consider for computing yaw (prevents unstable headings from tiny motions)
    ) -> None:
        """Read px/py/yaw samples, smooth with Savitzky-Golay,
        fit a B-spline, compute yaw in radians from path direction, then persist.
        Returns the smoothed list of points.
        """
        points = self._path
        if not points:
            logger.error(f"No points found in path data.")
            return
        
        original_count = len(points)
        if group_distance is not None and group_distance > 0:
            points = self.group_nearby_points(points, distance_threshold=group_distance)
            self._path = points  # Update the path with grouped points before smoothing

        # If too few points for a cubic spline, just write the original points
        if len(points) < 4:
            logger.info(
                f"Input points: {original_count}; Grouped points: {len(points)}; "
                f"Smoothed points: {len(points)};"
            )
            self.save_path_to_db()  # Save original points
            return

        # Apply Savitzky-Goyal smoothing before spline fit when possible
        window_length = savgol_window
        if window_length % 2 == 0:
            window_length += 1
        if window_length > len(points):
            window_length = len(points) if len(points) % 2 == 1 else len(points) - 1
        spline_input = points
        if window_length >= 3 and window_length > savgol_polyorder:
            spline_input = self.smooth_savgol(spline_input, window_length=window_length, polyorder=savgol_polyorder)

        smoothed = self.fit_bspline(spline_input, smoothing=smoothing, num_samples=num_samples)
        smoothed = [DataPoint(x=float(row[0]), y=float(row[1]), yaw=0.0) for row in smoothed]
        
        # Compute yaw from the smoothed path direction
        # Compute yaw from the smoothed path direction
        smoothed = self.compute_yaw_from_path(
            smoothed,
            unwrap=unwrap_yaw,
            wrap_to_pi=wrap_yaw_to_pi,
            min_motion_distance=yaw_min_motion_distance,
        )
        
        self._path = smoothed
        smoothed_count = len(smoothed)
        logger.info(
            f"Input points: {original_count}; Grouped points: {len(points)}; "
            f"Smoothed points: {smoothed_count};"
        )
        self.save_path_to_db()

        return
    
    def group_nearby_points(self, points : list[DataPoint], distance_threshold=0.10) -> list[DataPoint]:
        """Collapse consecutive points that stay within distance_threshold meters.

        The representative point for each cluster is the running centroid of the
        cluster, so short jitter around a location gets compressed into one point.
        """
        if not points:
            return []

        pts = np.asarray([(point.x, point.y) for point in points], dtype=float)
        grouped = []

        cluster_sum = pts[0].copy()
        cluster_count = 1
        cluster_center = cluster_sum / cluster_count

        for point in pts[1:]:
            if np.hypot(*(point - cluster_center)) <= distance_threshold:
                cluster_sum += point
                cluster_count += 1
                cluster_center = cluster_sum / cluster_count
            else:
                grouped.append(tuple(cluster_center))
                cluster_sum = point.copy()
                cluster_count = 1
                cluster_center = cluster_sum

        grouped.append(tuple(cluster_center))
        return [DataPoint(x=float(x), y=float(y), yaw=0.0) for x, y in grouped]
    
    def smooth_savgol(self, points: list[DataPoint], window_length: int = 11, polyorder: int = 3) -> list[DataPoint]:
        """
        window_length: Must be odd. Larger = smoother.
        polyorder: Polynomial order to fit in the window.
        """
        x = np.array([point.x for point in points], dtype=float)
        y = np.array([point.y for point in points], dtype=float)

        # Smooth x and y independently
        x_smooth = savgol_filter(x, window_length, polyorder)
        y_smooth = savgol_filter(y, window_length, polyorder)

        return [DataPoint(x=float(x_val), y=float(y_val), yaw=point.yaw)
                for x_val, y_val, point in zip(x_smooth, y_smooth, points)]

    def fit_bspline(self, points: list[DataPoint], smoothing: float = 0.5, num_samples: int = 100) -> list[tuple[float, float]]:
        """
        points: list of (x, y) tuples
        smoothing: The smoothing factor 's'. 
                0 = Interpolation (hits every point, noisy). 
                Higher = Smoother curve (loosely fits points).
        """
        points_array = np.array([(point.x, point.y) for point in points], dtype=float)        

        # 1. Parameterize by cumulative arc length so spacing reflects true travel distance.
        deltas = np.diff(points_array, axis=0)
        seg_len = np.sqrt(np.sum(deltas**2, axis=1))
        t = np.concatenate(([0.0], np.cumsum(seg_len)))

        # Remove duplicate-distance samples; splrep expects a strictly increasing parameter.
        keep = np.concatenate(([True], np.diff(t) > 1e-9))
        t_fit = t[keep]
        points_fit = points_array[keep]

        if len(points_fit) < 2:
            return [tuple(points_array[0])] * num_samples

        k = min(3, len(points_fit) - 1)

        # 2. Fit splines for each dimension independently against arc-length t.
        tcks = [splrep(t_fit, points_fit[:, i], s=smoothing, k=k) for i in range(points_fit.shape[1])]

        # 3. Generate new smooth points.
        t_new = np.linspace(t_fit.min(), t_fit.max(), num_samples)
        smoothed_columns = [splev(t_new, tck) for tck in tcks]

        return [tuple(row) for row in zip(*smoothed_columns)]
    
    def compute_yaw_from_path(self, points: list[DataPoint], unwrap: bool = True, wrap_to_pi: bool = True, min_motion_distance: float = 1e-3) -> list[DataPoint]:
        """Compute yaw as the direction of travel (angle) at each point.
        Returns list of DataPoint instances where yaw is atan2(dy, dx).
        """
        # Extract x, y from DataPoint instances for numpy conversion
        xy_array = np.array([(p.x, p.y) for p in points], dtype=float)
        x = xy_array[:, 0]
        y = xy_array[:, 1]

        if len(points) == 0:
            return []
        if len(points) == 1:
            return [DataPoint(x=float(x[0]), y=float(y[0]), yaw=0.0)]
        
        # Compute angle between consecutive points
        dx = np.diff(x)
        dy = np.diff(y)
        yaw = np.arctan2(dy, dx)
        motion = np.hypot(dx, dy)

        if unwrap:
            yaw = np.unwrap(yaw)

        # Ignore startup and intermittent tiny-motion samples that produce unstable headings.
        valid = motion >= min_motion_distance
        if np.any(valid):
            first_valid = int(np.argmax(valid))
            yaw[:first_valid] = yaw[first_valid]

            for i in range(first_valid + 1, len(yaw)):
                if not valid[i]:
                    yaw[i] = yaw[i - 1]
        else:
            yaw[:] = 0.0
        
        # Pad the last point (repeat final yaw since there's no "next" point)
        yaw = np.append(yaw, yaw[-1])

        if wrap_to_pi:
            yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        
        return [DataPoint(x=float(x_val), y=float(y_val), yaw=float(yaw_val)) for x_val, y_val, yaw_val in zip(x, y, yaw)]
