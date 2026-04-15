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
    wheel_base_width = 1.0414 # updated for big bullseye
    steering_angle_limit_rads = math.radians(30)  # degrees converted to radians, max steering angle for the front wheel
    steering_mechanics_adjustment_factor = 1.72 # empirically determined factor to adjust for mechanical differences in steering response

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
    frontwheel_encoder_zero_offset = math.radians(248.72)  # calibrated for straight-ahead position
    
    # Back Wheel Encoder Constants
    back_right_encoder_pin = 5
    back_left_encoder_pin = 25
    wheel_circumference = 1.756  # meters
    counts_per_revolution = 16  # encoder pulses per wheel rotation
    min_swap_speed_threshold = 0.01  # m/s, below this we consider the robot stopped. This determines wether to allow swapping to direction or not.
    inside_clutch_angle_threshold_rads = math.radians(10)  # degrees converted to radians, threshold for engaging inside clutch
    
    # Back Wheel Constants
    backwheel_forward_ssr_pin = 27
    backwheel_reverse_ssr_pin = 22
    backwheel_power_ssr_pin = 4
    distance_to_stop_from_full_speed = 5 # meters, distance at which it take to be able to stop from max speed. Used for clamping out speed when we are close to the boundary.
    rear_motor_top_speed = 11.176 # 25 mph = 11.176 m/s
    backwheel_power_scale_factor = 0.5  # Scale factor for back wheel speed control
    
    # Front Wheel Constants
    frontwheel_power_ssr_pin = 17
    
    # Clutches Constants
    left_clutch_pin = 16
    right_clutch_pin = 26
    
    # Header Healer Limit Switches Constants
    header_limit_switch_pin = 23
    healer_limit_switch_pin = 24
    
    # For the tag offsets:
    # +x : tag is forward of the robot center
    # +y : tag is to the robot's left side
    uwb_tag_data = [ UWBTagInfo(port="/dev/ttyACM0", id=0, offset=(0.0, -91.44 / 100, 0.0)), # back left tag in Meters
                     UWBTagInfo(port="/dev/ttyACM1", id=1, offset=(0.0, 22.86 / 100, 0.0)) ] # front right tag

    # Status LED Constants
    status_led_green = 13
    status_led_red = 19

    # IMU Constants
    imu_serial_port = "/dev/ttyUSB1"
    imu_baud_rate = 115200
    imu_timeout = 0.5  # seconds
    imu_update_rate = 100  # hz

    # XBee Constants
    serial_baud_rate = 19200
    pi_serial_port = "/dev/ttyAMA1"
    controller_update_rate = 0.05  # seconds between sending StateData updates to the controller

    #BMS Constants
    bms_update_interval = 0.05 # seconds between reads
    bms_serial_port = "/dev/ttyUSB0"

    # Path Creation Constants
    path_creation_update_interval = 0.1  # seconds between path point captures
    # Log creation constants (mostly usless now)
    logs_directory = Path.cwd() / "logs" # not important
    path_file_directory = logs_directory / "paths"
    records_directory = Path.cwd() / "records"
    references_directory = Path.cwd() / "references" # not important
    
    # Home Position Constants
    distance_to_home_threshold = 1.5  # meters, how close we need to be to consider ourselves "at home"
    difference_in_heading_to_home_threshold = math.radians(15)  # radians, how close our heading needs to be to consider ourselves "at home"


    #test
