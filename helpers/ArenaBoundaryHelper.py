"""Helper for managing the arena boundary using SQLite."""

import logging
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from helpers.sqllib import SQLiteFileManager

logger = logging.getLogger(__name__)


@dataclass
class ArenaBoundary:
    """Represents the arena boundary as 4 corner coordinates."""
    x1: float
    y1: float
    x2: float
    y2: float
    x3: float
    y3: float
    x4: float
    y4: float
    
    def get_corners(self) -> list[tuple[float, float]]:
        """Return boundary corners as list of (x, y) tuples."""
        return [
            (self.x1, self.y1),
            (self.x2, self.y2),
            (self.x3, self.y3),
            (self.x4, self.y4)
        ]


class ArenaBoundaryManager:
    """Manages arena boundary storage and retrieval using SQLite."""
    
    FIELDNAMES = ['x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'x4', 'y4']
    TABLE_NAME = 'arena_boundary'
    
    def __init__(self, db_dir: Path):
        """
        Initialize the ArenaBoundaryManager.
        
        Args:
            db_dir: Directory where robot_data.db will be stored
        """
        self.db_dir = Path(db_dir)
        self.db_dir.mkdir(parents=True, exist_ok=True)
        self.table_key = str(self.db_dir / self.TABLE_NAME)
    
    def set_arena_boundary(self, corners: list[tuple[float, float]]) -> bool:
        """
        Set the arena boundary. Overwrites any previous boundary.
        
        Args:
            corners: List of 4 (x, y) coordinate tuples representing the boundary corners
            
        Returns:
            True if successful, False otherwise
        """
        if len(corners) != 4:
            logger.error(f"Arena boundary requires exactly 4 corners, got {len(corners)}")
            return False
        
        db_manager = SQLiteFileManager()
        try:
            boundary_data = {
                'x1': str(corners[0][0]),
                'y1': str(corners[0][1]),
                'x2': str(corners[1][0]),
                'y2': str(corners[1][1]),
                'x3': str(corners[2][0]),
                'y3': str(corners[2][1]),
                'x4': str(corners[3][0]),
                'y4': str(corners[3][1])
            }
            success = db_manager.overwrite_with_row(self.table_key, self.FIELDNAMES, boundary_data)
            
            if success:
                logger.info(f"Arena boundary set with corners: {corners}")
            else:
                logger.error("Failed to set arena boundary")
            
            return success
        except Exception as e:
            logger.error(f"Error setting arena boundary: {e}")
            return False
        finally:
            db_manager.close_all()
    
    def set_arena_boundary_from_coords(self, x1: float, y1: float, x2: float, y2: float, 
                                      x3: float, y3: float, x4: float, y4: float) -> bool:
        """
        Set the arena boundary from individual coordinates.
        
        Args:
            x1, y1: First corner coordinates
            x2, y2: Second corner coordinates
            x3, y3: Third corner coordinates
            x4, y4: Fourth corner coordinates
            
        Returns:
            True if successful, False otherwise
        """
        corners = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        return self.set_arena_boundary(corners)
    
    def get_arena_boundary(self) -> Optional[ArenaBoundary]:
        """
        Retrieve the arena boundary.
        
        Returns:
            ArenaBoundary object if found, None otherwise
        """
        db_manager = SQLiteFileManager()
        try:
            row = db_manager.read_last_row(self.table_key)
            
            if row is None:
                logger.warning("No arena boundary set")
                return None
            
            try:
                boundary = ArenaBoundary(
                    x1=float(row.get('x1', 0.0)),
                    y1=float(row.get('y1', 0.0)),
                    x2=float(row.get('x2', 0.0)),
                    y2=float(row.get('y2', 0.0)),
                    x3=float(row.get('x3', 0.0)),
                    y3=float(row.get('y3', 0.0)),
                    x4=float(row.get('x4', 0.0)),
                    y4=float(row.get('y4', 0.0))
                )
                logger.info(f"Arena boundary retrieved: {boundary.get_corners()}")
                return boundary
            except (ValueError, TypeError) as e:
                logger.error(f"Error parsing arena boundary data: {e}")
                return None
        except Exception as e:
            logger.error(f"Error retrieving arena boundary: {e}")
            return None
        finally:
            db_manager.close_all()
    
    def has_arena_boundary(self) -> bool:
        """
        Check if an arena boundary has been set.
        
        Returns:
            True if arena boundary exists, False otherwise
        """
        return self.get_arena_boundary() is not None
