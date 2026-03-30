import logging
import csv
from pathlib import Path
from uuid import uuid4

from dataclasses import dataclass
import threading

from Robot.Constants import Constants
from Robot.subsystems.KalmanStateEstimator import KalmanStateEstimator
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
            # In a real implementation, you'd want a more graceful shutdown mechanism
            # For this example, we'll just let the thread exit when the program ends
            self.save_path_to_csv()
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
        logger.info(f"CreatePathCmd: saved {len(self._path)} path points to {file_path}")

    def add_path_point(self):
        if self.kf.is_initialized:
            currentPos = self.DataPoint(x=0.0, y=0.0, yaw=0.0)
            state = self.kf.get_state()
            currentPos.x = float(state.pos[0])  # x position in meters
            currentPos.y = float(state.pos[1])  # y position in meters
            euler = self.kf.euler  # numpy array [roll, pitch, yaw] in radians
            currentPos.yaw = float(euler[2]) * 180.0 / 3.141592653589793  # Convert yaw to degrees
            self._path.append(currentPos)

    def simplify_path_file(
        self,
        input_csv='state_estimator.csv',
        output_csv='reduced_path.csv',
        smoothing=0.5,
        num_samples=100,
        savgol_window=11,
        savgol_polyorder=3,
    ):
        """Read px/py from input_csv, smooth with Savitzky-Golay, fit a B-spline, write smoothed path to output_csv.
        Returns the smoothed list of points.
        """
        points = self.read_px_py_from_csv(input_csv)
        if not points:
            print(f"No valid points found in {input_csv}.")
            return []

        # If too few points for a cubic spline, just write the original points
        if len(points) < 4:
            self.write_px_py_to_csv(output_csv, points)
            print(f"Input points: {len(points)}; Smoothed points: {len(points)}; Wrote to {output_csv}")
            return points

        # Apply Savitzky-Golay smoothing before spline fit when possible
        window_length = savgol_window
        if window_length % 2 == 0:
            window_length += 1
        if window_length > len(points):
            window_length = len(points) if len(points) % 2 == 1 else len(points) - 1
        if window_length >= 3 and window_length > savgol_polyorder:
            points = self.smooth_savgol(points, window_length=window_length, polyorder=savgol_polyorder)

        smoothed = self.fit_bspline(points, smoothing=smoothing, num_samples=num_samples)
        self.write_px_py_to_csv(output_csv, smoothed)
        print(f"Input points: {len(points)}; Smoothed points: {len(smoothed)}; Wrote to {output_csv}")
        return smoothed
    
    def smooth_savgol(self, points, window_length=11, polyorder=3):
        """
        window_length: Must be odd. Larger = smoother.
        polyorder: Polynomial order to fit in the window.
        """
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]

        # Smooth x and y independently
        x_smooth = savgol_filter(x, window_length, polyorder)
        y_smooth = savgol_filter(y, window_length, polyorder)

        # You can then pass (x_smooth, y_smooth) to RDP or CubicSpline
        return list(zip(x_smooth, y_smooth))

    def fit_bspline(self, points, smoothing=0.5, num_samples=100):
        """
        points: list of (x, y) tuples
        smoothing: The smoothing factor 's'. 
                0 = Interpolation (hits every point, noisy). 
                Higher = Smoother curve (loosely fits points).
        """
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]

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

        return list(zip(x_smooth, y_smooth))
    
    def perpendicular_distance(self, point, line_start, line_end):
        """Return the perpendicular distance from `point` to the line segment
        defined by `line_start` and `line_end`.
        point, line_start, line_end are (x, y) tuples.
        """
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end

        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            # line_start and line_end are the same point
            return ((px - x1) ** 2 + (py - y1) ** 2) ** 0.5

        # Project point onto the line segment, computing parameter t
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        # Clamp t to the segment [0,1]
        if t < 0:
            proj_x, proj_y = x1, y1
        elif t > 1:
            proj_x, proj_y = x2, y2
        else:
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy

        return ((px - proj_x) ** 2 + (py - proj_y) ** 2) ** 0.5


    def read_px_py_from_csv(self, path):
        """Read px,py columns (case-insensitive) from a CSV and return list of (x,y) tuples.
        Rows with missing or non-numeric px/py are skipped.
        """
        import csv

        points = []
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            # find px, py field names case-insensitively
            headers = reader.fieldnames or []
            lower_map = {h.lower(): h for h in headers}
            if 'px' not in lower_map or 'py' not in lower_map:
                raise ValueError(f"Input CSV must contain 'px' and 'py' columns. Found: {headers}")

            px_field = lower_map['px']
            py_field = lower_map['py']

            for row in reader:
                try:
                    x = float(row[px_field])
                    y = float(row[py_field])
                    points.append((x, y))
                except Exception:
                    # skip rows with missing/invalid numbers
                    continue

        return points


    def write_px_py_to_csv(self, path, points):
        """Write list of (x,y) tuples to CSV with header px,py."""
        import csv

        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['px', 'py'])
            for x, y in points:
                writer.writerow([f"{x:.6f}", f"{y:.6f}"])