from pydantic import BaseModel
from dataclasses import dataclass

@dataclass
class PathData(BaseModel):
    position_list: list[tuple[float, float]]
    id: int