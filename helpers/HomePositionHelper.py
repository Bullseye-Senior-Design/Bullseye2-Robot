"""Helper for managing the robot's home position using SQLite."""

import logging
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from helpers.dbConstants import HOME_POSITION_TABLE
from helpers.sqllib import SQLiteFileManager

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)  # Set to DEBUG for detailed output


@dataclass
class HomePosition:
    """Represents the robot's home position."""
    x: float
    y: float
    yaw: float


class HomePositionManager:
    """Manages robot home position storage and retrieval using SQLite."""
    
    def set_home_position(self, x: float, y: float, yaw: float) -> bool:
        """
        Set the robot's home position. Overwrites any previous home position.
        
        Args:
            x: X coordinate (meters)
            y: Y coordinate (meters)
            yaw: Yaw angle (degrees or radians, depending on convention)
            
        Returns:
            True if successful, False otherwise
        """
        db_manager = SQLiteFileManager()
        try:
            home_data = HOME_POSITION_TABLE.build_row(x=x, y=y, yaw=yaw)
            success = db_manager.overwrite_with_row(HOME_POSITION_TABLE, home_data)
            
            if success:
                logger.info(f"Home position set to ({x}, {y}, {yaw})")
            else:
                logger.error("Failed to set home position")
            
            return success
        except Exception as e:
            logger.error(f"Error setting home position: {e}")
            return False
        finally:
            db_manager.close_all()
    
    def get_home_position(self) -> Optional[HomePosition]:
        """
        Retrieve the robot's home position.
        
        Returns:
            HomePosition object if found, None otherwise
        """
        db_manager = SQLiteFileManager()
        try:
            row = db_manager.read_last_row(HOME_POSITION_TABLE)
            
            if row is None:
                logger.warning("No home position set")
                return None
            
            try:
                home_pos = HomePosition(
                    x=float(row.get('x', 0.0)),
                    y=float(row.get('y', 0.0)),
                    yaw=float(row.get('yaw', 0.0))
                )
                logger.info(f"Home position retrieved: ({home_pos.x}, {home_pos.y}, {home_pos.yaw})")
                return home_pos
            except (ValueError, TypeError) as e:
                logger.error(f"Error parsing home position data: {e}")
                return None
        except Exception as e:
            logger.error(f"Error retrieving home position: {e}")
            return None
        finally:
            db_manager.close_all()
    
    def has_home_position(self) -> bool:
        """
        Check if a home position has been set.
        
        Returns:
            True if home position exists, False otherwise
        """
        return self.get_home_position() is not None
