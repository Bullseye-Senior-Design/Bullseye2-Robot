from dataclasses import dataclass
@dataclass 
class BatteryData:
    voltage: float
    current: float
    power: float
    state_of_charge: float
    time_remaining: float

# test 