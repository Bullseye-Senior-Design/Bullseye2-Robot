from pathlib import Path


class UWBTagInfo:
    def __init__(self, port, id, offset):
        self.port = port
        self.id = id
        self.offset = offset

class Constants:
    # Wheel base width in meters
    wheel_base_width = 0.25
    
    # SPI Constants
    spi_bus = 0
    
    # DAC Constants
    dac_spi_device = 1
    dac_spi_mode = 0
    dac_max_freq_hz = 10000
    dac_backwheel_channel = 0
    dac_frontwheel_channel = 1
    dac_resolution = 12
    dac_max_value = (1 << dac_resolution) - 1  # 4095 for 12-bit DAC

    # Front Wheel Encoder Constants
    frontwheel_encoder_spi_device = 0
    frontwheel_encoder_spi_mode = 0
    frontwheel_encoder_max_freq_hz = 1000
    frontwheel_encoder_resolution = 12  # bits
    frontwheel_encoder_max_position = (1 << frontwheel_encoder_resolution) - 1
    
    # Back Wheel Encoder Constants
    back_right_encoder_pin = 5
    back_left_encoder_pin = 25
    wheel_circumference = 0.25  # meters
    counts_per_revolution = 6  # encoder pulses per wheel rotation
    
    # Back Wheel Constants
    backwheel_forward_ssr_pin = 27
    backwheel_reverse_ssr_pin = 22
    backwheel_power_ssr_pin = 4
    rear_motor_top_speed = 0.13
    backwheel_power_scale_factor = 0.3  # Scale factor for back wheel speed control
    
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
    uwb_tag_data = [ UWBTagInfo(port="/dev/ttyACM0", id=0, offset=(-24.77 / 2 / 100, 22.225 / 2 / 100, 0.0)), # back left tag
                     UWBTagInfo(port="/dev/ttyACM1", id=1, offset=(24.77 / 2 / 100, -22.225 / 2 / 100, 0.0)) ] # front right tag

    status_led_green = 13
    status_led_red = 19

    # Controller/BMS Serial
    serial_baud_rate = 19200
    controller_serial_port = "/dev/ttyUSB0"
    pi_serial_port = "/dev/ttyAMA1"
    bms_serial_port = "/dev/ttyUSB1"

    # Controller input handling
    controller_deadzone = 0.15
    controller_update_rate = 0.1  # seconds (20 Hz)

    #BMS Constants
    bms_update_interval = 0.05 # seconds between reads

    # Path creation constants
    path_creation_update_interval = 0.1  # seconds between path point captures
    path_file_directory  = Path.cwd() / 'logs' / 'paths'

