"""Helper for managing the arena boundary using SQLite."""

import logging
import math
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from helpers.dbConstants import ARENA_BOUNDARY_TABLE
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

    @staticmethod
    def _cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        """2D cross product of OA and OB vectors."""
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    @staticmethod
    def _distance_point_to_segment(
        px: float,
        py: float,
        ax: float,
        ay: float,
        bx: float,
        by: float,
    ) -> float:
        """Shortest Euclidean distance from a point to a line segment."""
        abx = bx - ax
        aby = by - ay
        apx = px - ax
        apy = py - ay

        ab_len_sq = abx * abx + aby * aby
        if ab_len_sq == 0.0:
            return math.hypot(apx, apy)

        t = (apx * abx + apy * aby) / ab_len_sq
        t = max(0.0, min(1.0, t))

        closest_x = ax + t * abx
        closest_y = ay + t * aby
        return math.hypot(px - closest_x, py - closest_y)

    def shortest_distance_to_edge(self, x: float, y: float) -> float:
        """
        Return shortest distance from a point to the nearest rectangle edge.

        If the point is outside the rectangle, returns 0.0.
        """
        corners = self.get_corners()
        p = (x, y)

        # For a convex quadrilateral with ordered corners, point is inside iff
        # all cross products have the same sign (or zero on edges).
        signs = [
            self._cross(corners[i], corners[(i + 1) % 4], p)
            for i in range(4)
        ]
        has_pos = any(v > 0 for v in signs)
        has_neg = any(v < 0 for v in signs)
        if has_pos and has_neg:
            return 0.0

        distances = [
            self._distance_point_to_segment(
                x,
                y,
                corners[i][0],
                corners[i][1],
                corners[(i + 1) % 4][0],
                corners[(i + 1) % 4][1],
            )
            for i in range(4)
        ]
        return min(distances)


class ArenaBoundaryManager:
    """Manages arena boundary storage and retrieval using SQLite."""
    
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
            boundary_data = ARENA_BOUNDARY_TABLE.build_row(
                x1=corners[0][0],
                y1=corners[0][1],
                x2=corners[1][0],
                y2=corners[1][1],
                x3=corners[2][0],
                y3=corners[2][1],
                x4=corners[3][0],
                y4=corners[3][1],
            )
            success = db_manager.overwrite_with_row(ARENA_BOUNDARY_TABLE, boundary_data)
            
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
            row = db_manager.read_last_row(ARENA_BOUNDARY_TABLE)
            
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
