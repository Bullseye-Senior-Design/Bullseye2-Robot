import logging
import csv
import math
from pathlib import Path
from uuid import uuid4

from dataclasses import dataclass
import threading

from Robot.Constants import Constants
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from structure.Subsystem import Subsystem
from scipy.interpolate import splrep, splev
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
import numpy as np

logger = logging.getLogger(f"{__name__}.PathFollowing")
logger.setLevel(logging.INFO)


class PathCreation(Subsystem):
    PATH_FIELDNAMES = ['x', 'y', 'yaw']

    @dataclass
    class DataPoint:
        x: float
        y: float
        yaw: float

    def __init__(self):
        super().__init__()
        self.kf = KalmanStateEstimator()
        self.logs_dir = Constants.path_file_directory

    def start_path_creation(self):
        logger.info("Path creation started")
        
        self._path = []

    def stop_path_creation(self):
        logger.info("Path creation stopped")
        if len(self._path) > 0:
            self.simplify_path()
        else:
            logger.error("No path points were recorded, nothing to save.")
        

    def save_path_to_csv(self):
        if not self._path:
            return
        
        self.logs_dir.mkdir(parents=True, exist_ok=True)

        filename = f"{uuid4()}.csv"
        file_path = self.logs_dir / filename

        with file_path.open('w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.PATH_FIELDNAMES)
            writer.writeheader()
            for point in self._path:
                writer.writerow({'x': point.x, 'y': point.y, 'yaw': point.yaw})

        self._saved_file_path = file_path
        logger.info(f"Saved {len(self._path)} path points to {file_path}")

    def add_path_point(self):
        if self.kf.is_initialized:
            currentPos = self.DataPoint(x=0.0, y=0.0, yaw=0.0)
            state = self.kf.get_state()
            currentPos.x = float(state.pos[0])  # x position in meters
            currentPos.y = float(state.pos[1])  # y position in meters
            euler = self.kf.euler  # numpy array [roll, pitch, yaw] in radians
            currentPos.yaw = math.degrees(euler[2])
            self._path.append(currentPos)

    def simplify_path(
        self,
        smoothing=0.5,
        num_samples=100,
        savgol_window=11,
        savgol_polyorder=3,
    ):
        """Read px/py/yaw from input_csv, smooth with Savitzky-Golay,
        fit a B-spline, compute yaw from path direction, write smoothed path to output_csv.
        Returns the smoothed list of points.
        """
        points = self._path
        if not points:
            logger.error(f"No points found in path data.")
            return []

        # If too few points for a cubic spline, just write the original points
        if len(points) < 4:
            logger.info(f"Input points: {len(points)}; Smoothed points: {len(points)}")
            self.save_path_to_csv()  # Save original points
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
        
        # Compute yaw from the smoothed path direction
        smoothed = self.compute_yaw_from_path(smoothed)
        
        self._path = smoothed
        logger.info(f"Input points: {len(points)}; Smoothed points: {len(smoothed)}")
        self.save_path_to_csv()

        return
    
    def smooth_savgol(self, points: list[DataPoint], window_length=11, polyorder=3):
        """
        window_length: Must be odd. Larger = smoother.
        polyorder: Polynomial order to fit in the window.
        """
        x = np.array([point.x for point in points], dtype=float)
        y = np.array([point.y for point in points], dtype=float)

        # Smooth x and y independently
        x_smooth = savgol_filter(x, window_length, polyorder)
        y_smooth = savgol_filter(y, window_length, polyorder)

        return [self.DataPoint(x=float(x_val), y=float(y_val), yaw=point.yaw)
                for x_val, y_val, point in zip(x_smooth, y_smooth, points)]

    def fit_bspline(self, points: list[DataPoint], smoothing=0.5, num_samples=100):
        """
        points: list of DataPoint objects
        smoothing: The smoothing factor 's'. 
                0 = Interpolation (hits every point, noisy). 
                Higher = Smoother curve (loosely fits points).
        """
        x = np.array([point.x for point in points], dtype=float)
        y = np.array([point.y for point in points], dtype=float)

        # 1. Parameterize the curve based on the index (or distance)
        # We use a parameter 't' to handle loops and vertical lines correctly
        t = np.arange(len(points))

        # 2. Fit splines for x and y independently against t
        # k=3 implies a Cubic spline
        tck_x = splrep(t, x, s=smoothing, k=3) 
        tck_y = splrep(t, y, s=smoothing, k=3)

        # 3. Generate new smooth points
        t_new = np.linspace(t.min(), t.max(), num_samples)
        x_smooth = splev(t_new, tck_x)
        y_smooth = splev(t_new, tck_y)

        return [self.DataPoint(x=float(x_val), y=float(y_val), yaw=0.0) for x_val, y_val in zip(x_smooth, y_smooth)]
    
    def compute_yaw_from_path(self, points: list[DataPoint]):
        """Compute yaw from path direction and update each DataPoint in-place.
        Yaw is computed as atan2(dy, dx) in radians.
        """
        if not points:
            return points

        if len(points) == 1:
            points[0].yaw = 0.0
            return points

        for i in range(len(points) - 1):
            dx = points[i + 1].x - points[i].x
            dy = points[i + 1].y - points[i].y
            points[i].yaw = float(np.arctan2(dy, dx))

        points[-1].yaw = points[-2].yaw
        return points