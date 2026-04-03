from pydantic import BaseModel


class KFXMapUpdate(BaseModel):
    """
    Sent from the controller GUI to the Pi to update button-to-action mappings.
    Only buttons 3-8 are configurable; buttons 1 (E-Stop) and 2 (Start) are hardcoded.

    Keys are button numbers as strings ("3" through "8").
    Values are route identifiers in the form "route:<id>" (e.g. "route:0").
    """
    mapping: dict[str, str]
