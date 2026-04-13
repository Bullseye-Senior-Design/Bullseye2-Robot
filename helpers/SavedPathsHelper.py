import logging
from pathlib import Path
from typing import Optional, List

from helpers.dbConstants import PathPointsTable
from helpers.sqllib import ROBOT_DATA_DB_FILENAME, SQLiteFileManager

logger = logging.getLogger(f"{__name__}.SavedPathsHelper")
logger.setLevel(logging.INFO)


class SavedPathsHelper:
    """Helper class for managing saved paths in the database.
    
    Provides methods to load, retrieve, delete, and manage saved paths
    stored in the SQLite database.
    """

    def __init__(self, db_path: Optional[Path] = None):
        """Initialize SavedPathsHelper.
        
        Args:
            db_path: Path to the database file. If None, uses current working directory.
        """
        if db_path is None:
            self.db_path = Path.cwd() / ROBOT_DATA_DB_FILENAME
        else:
            self.db_path = db_path
        
        self.db_manager = SQLiteFileManager()

    def load_path_by_id(self, path_id: int) -> Optional[List[tuple]]:
        """Load a saved path from database by its numeric ID.
        
        Args:
            path_id: The numeric ID of the path to load
            
        Returns:
            List of (x, y, yaw) tuples representing the path, or None if not found
        """
        try:
            path_table = PathPointsTable(name=str(path_id))
            self.db_manager.setup_file(path_table)
            
            rows = self.db_manager.read_rows(path_table)
            if rows:
                logger.info(f"Loaded path {path_id} with {len(rows)} points")
                return [(row['x'], row['y'], row['yaw']) for row in rows]
            else:
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
            List of numeric path IDs
        """
        try:
            path_ids = []
            self.db_manager.setup_file(PathPointsTable())
            
            # Query the database for all numeric table keys
            all_tables = self.db_manager.get_all_table_names()
            for table_name in all_tables:
                try:
                    # Try to convert to int - valid path IDs are numeric
                    path_id = int(table_name)
                    path_ids.append(path_id)
                except (ValueError, TypeError):
                    # Skip non-numeric table names
                    pass
            
            path_ids.sort()
            logger.info(f"Found {len(path_ids)} saved paths: {path_ids}")
            return path_ids
            
        except Exception as e:
            logger.error(f"Error retrieving saved paths: {e}")
            return []
        finally:
            self.db_manager.close_all()

    def delete_path(self, path_id: int) -> bool:
        """Delete a saved path from the database.
        
        Args:
            path_id: The numeric ID of the path to delete
            
        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            path_table = PathPointsTable(name=str(path_id))
            self.db_manager.setup_file(path_table)
            
            self.db_manager.delete_table(path_table)
            logger.info(f"Successfully deleted path {path_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error deleting path {path_id}: {e}")
            return False
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
