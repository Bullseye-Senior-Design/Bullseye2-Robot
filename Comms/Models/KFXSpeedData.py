from pydantic import BaseModel

class KFXSpeedData(BaseModel):
    speed: float
