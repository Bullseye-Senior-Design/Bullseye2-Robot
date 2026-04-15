import logging
import json
from pathlib import Path
from typing import Optional, List

from helpers.dbConstants import PATH_POINTS_TABLE
from helpers.HomePositionHelper import HomePositionManager
from helpers.sqllib import ROBOT_DATA_DB_FILENAME, SQLiteFileManager

logger = logging.getLogger(f"{__name__}.SavedPathsHelper")
logger.setLevel(logging.INFO)


class SavedPathsHelper:
    """Helper class for managing saved paths in the database.
    
    Provides methods to load, retrieve, delete, and manage saved paths
    stored in the SQLite database.
    """

    def __init__(self):
        
        self.db_manager = SQLiteFileManager()
        self.home_manager = HomePositionManager()

    def save_path(self, path_points: List[tuple[float, float, float]]) -> Optional[int]:
        """Save a path to the unified path_points table.

        Args:
            path_points: List of (x, y, yaw) tuples.

        Returns:
            New numeric path ID if save succeeded, otherwise None.
        """
        if not path_points:
            logger.error("Cannot save an empty path")
            return None

        try:
            self.db_manager.setup_file(PATH_POINTS_TABLE)
            existing_rows = self.db_manager.read_rows(PATH_POINTS_TABLE)

            existing_ids: list[int] = []
            for row in existing_rows:
                raw_id = row.get("id")
                if raw_id is None:
                    continue
                try:
                    existing_ids.append(int(raw_id))
                except (TypeError, ValueError):
                    continue

            next_number = (max(existing_ids) + 1) if existing_ids else 0

            points_json = json.dumps([
                {"x": x, "y": y, "yaw": yaw}
                for x, y, yaw in path_points
            ])

            home_position = self.home_manager.get_home_position()
            home_position_json = json.dumps({
                "x": home_position.x,
                "y": home_position.y,
                "yaw": home_position.yaw,
            }) if home_position is not None else json.dumps(None)

            row = PATH_POINTS_TABLE.build_row(
                path_id=next_number,
                points=points_json,
                home_position=home_position_json,
            )

            if not self.db_manager.write_row(PATH_POINTS_TABLE, row):
                logger.error("Failed to write path row to database")
                return None

            logger.info(f"Saved path {next_number} with {len(path_points)} points")
            return next_number
        except Exception as e:
            logger.error(f"Error saving path: {e}")
            return None
        finally:
            self.db_manager.close_all()

    def load_path_by_id(self, path_id: int) -> Optional[List[tuple]]:
        """Load a saved path from database by its numeric ID.
        
        Args:
            path_id: The numeric ID of the path to load
            
        Returns:
            List of (x, y, yaw) tuples representing the path, or None if not found
        """
        try:
            self.db_manager.setup_file(PATH_POINTS_TABLE)
            rows = self.db_manager.read_rows(PATH_POINTS_TABLE)

            for row in rows:
                raw_id = row.get("id")
                if raw_id is None:
                    continue
                try:
                    if int(raw_id) != int(path_id):
                        continue
                except (TypeError, ValueError):
                    continue

                points_raw = row.get("points", "")
                if not points_raw:
                    return []

                try:
                    points = json.loads(points_raw)
                except Exception:
                    logger.error(f"Path {path_id} contains invalid points JSON")
                    return None

                if not isinstance(points, list):
                    logger.error(f"Path {path_id} points payload is not a list")
                    return None

                parsed_points: list[tuple[float, float, float]] = []
                for point in points:
                    if not isinstance(point, dict):
                        continue
                    try:
                        parsed_points.append((
                            float(point.get("x", 0.0)),
                            float(point.get("y", 0.0)),
                            float(point.get("yaw", 0.0)),
                        ))
                    except (TypeError, ValueError):
                        continue

                logger.info(f"Loaded path {path_id} with {len(parsed_points)} points")
                return parsed_points

            logger.warning(f"Path {path_id} not found in database")
            return None
                
        except Exception as e:
            logger.error(f"Error loading path {path_id}: {e}")
            return None
        finally:
            self.db_manager.close_all()

    def get_all_saved_paths(self) -> List[int]:
        """Get all saved path IDs from the database.

        Returns:
            Sorted list of numeric path IDs.
        """
        try:
            self.db_manager.setup_file(PATH_POINTS_TABLE)
            rows = self.db_manager.read_rows(PATH_POINTS_TABLE)

            ids: list[int] = []
            for row in rows:
                raw_id = row.get("id")
                if raw_id is None:
                    continue
                try:
                    ids.append(int(raw_id))
                except (TypeError, ValueError):
                    continue

            return sorted(set(ids))
        except Exception as e:
            logger.error(f"Error retrieving saved path IDs: {e}")
            return []
        finally:
            self.db_manager.close_all()

    def get_path_metadata(self, path_id: int) -> Optional[dict]:
        """Get metadata about a saved path (point count, bounds, etc.).
        
        Args:
            path_id: The numeric ID of the path
            
        Returns:
            Dictionary with metadata, or None if path not found
        """
        try:
            path_data = self.load_path_by_id(path_id)
            if path_data is None:
                return None
            
            x_coords = [p[0] for p in path_data]
            y_coords = [p[1] for p in path_data]
            yaw_coords = [p[2] for p in path_data]
            
            metadata = {
                'path_id': path_id,
                'point_count': len(path_data),
                'x_min': min(x_coords),
                'x_max': max(x_coords),
                'y_min': min(y_coords),
                'y_max': max(y_coords),
                'yaw_min': min(yaw_coords),
                'yaw_max': max(yaw_coords),
            }
            
            logger.debug(f"Path {path_id} metadata: {metadata}")
            return metadata
            
        except Exception as e:
            logger.error(f"Error getting metadata for path {path_id}: {e}")
            return None

    def get_total_path_distance(self, path_id: int) -> Optional[float]:
        """Calculate the total distance traveled along a path.
        
        Args:
            path_id: The numeric ID of the path
            
        Returns:
            Total distance in the same units as path coordinates, or None if error
        """
        try:
            path_data = self.load_path_by_id(path_id)
            if path_data is None or len(path_data) < 2:
                return None
            
            total_distance = 0.0
            for i in range(len(path_data) - 1):
                x1, y1, _ = path_data[i]
                x2, y2, _ = path_data[i + 1]
                segment_distance = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
                total_distance += segment_distance
            
            logger.info(f"Path {path_id} total distance: {total_distance:.2f}")
            return total_distance
            
        except Exception as e:
            logger.error(f"Error calculating distance for path {path_id}: {e}")
            return None
