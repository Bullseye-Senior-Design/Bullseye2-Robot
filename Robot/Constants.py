from pathlib import Path
import threading
import math

class UWBTagInfo:
    def __init__(self, port, id, offset):
        self.port = port
        self.id = id
        self.offset = offset

class Constants:
    # Wheel base width in meters
    wheel_base_width = 0.25
    steering_angle_limit_rads = math.radians(60)  # degrees converted to radians, max steering angle for the front wheel

    # Bit Banging Constants
    bitbang_setup_delay = 0.0001
    bitbang_clock_delay = 0.0001
    bitbang_clock_pin = 11
    shifter_dir_pin = 21
    bitbang_MISO_pin = 9
    bitbang_MOSI_pin = 10
    bitbang_cs_DAC_pin = 7
    bitbang_cs_frontwheel_encoder_pin = 8
    bitbang_spi_lock = threading.Lock()
    
    # DAC Constants
    dac_backwheel_channel = 0
    dac_frontwheel_channel = 1
    dac_resolution = 12
    dac_max_value = (1 << dac_resolution) - 1  # 4095 for 12-bit DAC

    # Front Wheel Encoder Constants
    frontwheel_encoder_resolution = 14  # bits
    frontwheel_encoder_max_position = (1 << frontwheel_encoder_resolution) - 1
    frontwheel_encoder_zero_offset = 4.5  # calibrated for straight-ahead position
    
    # Back Wheel Encoder Constants
    back_right_encoder_pin = 5
    back_left_encoder_pin = 25
    wheel_circumference = 1.651  # meters
    counts_per_revolution = 16  # encoder pulses per wheel rotation
    min_swap_speed_threshold = 0.01  # m/s, below this we consider the robot stopped. This determines wether to allow swapping to direction or not.
    
    # Back Wheel Constants
    backwheel_forward_ssr_pin = 27
    backwheel_reverse_ssr_pin = 22
    backwheel_power_ssr_pin = 4
    rear_motor_top_speed = 0.13
    backwheel_power_scale_factor = 0.5  # Scale factor for back wheel speed control
    backwheel_accel_limit_per_sec = 2.0  # Max normalized speed change per second
    
    # Front Wheel Constants
    frontwheel_power_ssr_pin = 17
    
    # Clutches Constants
    left_clutch_pin = 26
    right_clutch_pin = 16
    
    # Header Healer Limit Switches Constants
    header_limit_switch_pin = 23
    healer_limit_switch_pin = 24
    
    # For the tag offsets:
    # +x : tag is forward of the robot center
    # +y : tag is to the robot's left side
    uwb_tag_data = [ UWBTagInfo(port="/dev/ttyACM0", id=0, offset=(0.0, -91.44 / 100, 0.0)), # back left tag in Meters
                     UWBTagInfo(port="/dev/ttyACM1", id=1, offset=(0.0, 22.86 / 100, 0.0)) ] # front right tag

    status_led_green = 13
    status_led_red = 19

    # Controller/BMS Serial
    serial_baud_rate = 19200
    controller_serial_port = "/dev/ttyUSB0"
    pi_serial_port = "/dev/ttyAMA1"
    bms_serial_port = "/dev/ttyUSB0"

    # Controller input handling
    controller_deadzone = 0.15
    controller_update_rate = 0.05 

    #BMS Constants
    bms_update_interval = 0.05 # seconds between reads

    # Path creation constants
    path_creation_update_interval = 0.1  # seconds between path point captures
    path_file_directory  = Path.cwd() / 'logs' / 'paths'

