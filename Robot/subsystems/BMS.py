import serial
from Comms.BatteryData import BatteryData

# Update this if needed (check with: ls /dev/ttyUSB*)
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 19200

# Fields we care about
TARGET_FIELDS = {"V", "I", "P", "SOC", "TTG"}
PRINTDEBUG = False

def parse_value(key, value):
    """
    Convert Victron raw values to human-readable units
    """
    if key == "V":      # mV -> V
        return float(value) / 1000
    elif key == "I":    # mA -> A
        return float(value) / 1000
    elif key == "P":    # W
        return float(value)
    elif key == "SOC":  # 0.1% -> %
        return float(value) / 10
    elif key == "TTG":  # minutes
        num = float(value)
        return num  # Return as float for time_remaining
    else:
        return value

def read_smartshunt():
    data = {}

    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while True:
            line = ser.readline().decode(errors="ignore").strip()

            # VE.Direct packets end with a checksum
            if line.startswith("Checksum"):
                if data:
                    return BatteryData(
                        voltage=data.get('V', 0.0),
                        current=data.get('I', 0.0),
                        power=data.get('P', 0.0),
                        state_of_charge=data.get('SOC', 0.0),
                        time_remaining=data.get('TTG', 0.0)
                    )
                data = {}
                continue

            if "\t" in line:
                key, value = line.split("\t", 1)

                if key in TARGET_FIELDS:
                    data[key] = parse_value(key, value)

if __name__ == "__main__":
    while True:
        battery_data = read_smartshunt()

        if PRINTDEBUG:
            print("SmartShunt Data:")
            print(f" Voltage (V): {battery_data.voltage}")
            print(f" Current (A): {battery_data.current}")
            print(f" Power (W):   {battery_data.power}")
            print(f" SOC (%):     {battery_data.state_of_charge}")      #Update everytime SOC changes by 0.5%
            print(f" TTG (min):   {battery_data.time_remaining}")
            print("-" * 30)
