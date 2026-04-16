import logging

from Robot.Commands.StopMovementCmd import StopMovementCmd
from Robot.Commands.CreatePathCmd import CreatePathCmd
from Robot.Commands.DefaultMovementCmd import DefaultMovementCmd
from Robot.Constants import Constants
from Robot.subsystems.algorithms.PathCreation import PathCreation
from structure.Input.InputScheduler import InputScheduler
from structure.commands.InstantCommand import InstantCommand
from structure.commands.SequentialCommandGroup import SequentialCommandGroup

from Robot.subsystems.sensors.UWB import UWB
from Robot.subsystems.sensors.IMU import IMU
from Robot.subsystems.algorithms.PathFollowing import PathFollowing
from Robot.subsystems.DriveTrain import DriveTrain
from Robot.subsystems.BMS import BMS
from Robot.subsystems.PCBLEDs import PCBLEDs
from Robot.subsystems.algorithms.PathCreation import PathCreation

import time

from Robot.subsystems.sensors.BackWheelEncoder import BackWheelEncoder
from Robot.subsystems.Clutches import Clutches
from Robot.subsystems.HeaderHealerSwitches import HeaderHealerSwitches
from Robot.subsystems.algorithms.ParkingController import ParkingController

from Robot.Commands.LogDataCmd import LogDataCmd
from Robot.Commands.PlotStateCmd import PlotStateCmd
from Robot.Commands.MotorMovementExampleCmd import MotorMovementExampleCmd
from Robot.Commands.AlignIMUToWorldCmd import AlignIMUToWorldCmd
from Robot.Commands.ZeroIMUCmd import ZeroIMUCmd
from Robot.Commands.TestingFollowPathCmd import TestingFollowPathCmd
from Comms.PiCommThread import PiCommThread
from Robot.Commands.ParkingCmd import ParkingCmd
from Robot.Commands.DriveHomeCmd import DriveHomeCmd
from Robot.Commands.FollowPathCmd import FollowPathCmd

logger = logging.getLogger(f"{__name__}.RobotContainer")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output

class RobotContainer:
    def __init__(self):
        self.uwb = UWB()
        self.back_Wheel_encoder = BackWheelEncoder()
        self.imu = IMU()
        self.clutches = Clutches()
        self.path_following = PathFollowing()
        self.parking_controller = ParkingController()
        self.path_creation = PathCreation()
        self.drive_train = DriveTrain()
        self.header_healer_switches = HeaderHealerSwitches()
        self.pcb_leds = PCBLEDs()
        
        # Start subsystems
        self.bms = BMS()
        self.comm_thread = PiCommThread(bms=self.bms)
        self.comm_thread.start()
            
        # Start subsystems
        self.uwb.start(uwb_tag_data=Constants.uwb_tag_data, anchors_pos=None)
        self.back_Wheel_encoder.start()

        self.path_cmd = None  # To keep track of the path creation command for cancellation if needed
        
        # InputScheduler(lambda: self.comm_thread.get_controller_data().btn_A).on_true(
        #     CreatePathCmd(self.path_creation, lambda: self.comm_thread.get_controller_data().btn_B)
        # )
        
    
    def start_robot(self):
        logger.info("Running robot initialization")
        AlignIMUToWorldCmd(self.imu, self.uwb).schedule()  # Align IMU to UWB at startup
                    
    def begin_data_log(self):
        logger.info("Starting data logging")
        LogDataCmd(self.path_following).schedule()
        #MotorMovementExampleCmd(self.drive_train, self.clutches, self.header_healer_switches, self.pcb_leds).schedule()
        
    def start_parking(self):
        logger.info("Starting parking sequence")

        return_home_cmd = SequentialCommandGroup()
        return_home_cmd.add_commands(
            DriveHomeCmd(self.drive_train, self.path_following),
            ParkingCmd(self.drive_train, self.parking_controller)
        )
        return_home_cmd.schedule()
        
    def start_path_following(self):
        logger.info("Starting path following mode")
        self.path_following = PathFollowing.reset_instance()  # Get fresh instance
        TestingFollowPathCmd(self.drive_train, self.path_following).schedule()
        # FollowPathCmd(self.drive_train, self.path_following).schedule()

    def start_teleop(self):
        logger.info("Starting teleop control")

        DefaultMovementCmd(self.drive_train, self.clutches,
                            lambda: self.comm_thread.get_controller_data().left_y, 
                            lambda: self.comm_thread.get_controller_data().right_x).schedule()
        
        InputScheduler(lambda: self.comm_thread.get_controller_data().btn_Y).on_true(
            StopMovementCmd(self.drive_train)
        )

        InputScheduler(lambda: self.comm_thread.get_controller_data().btn_X).on_true(
            DefaultMovementCmd(self.drive_train, self.clutches,
                            lambda: self.comm_thread.get_controller_data().left_y, 
                            lambda: self.comm_thread.get_controller_data().right_x)
        )
    
    def start_path_creation(self):
        logger.info("Starting path creation mode")

        DefaultMovementCmd(self.drive_train, self.clutches,
                            lambda: self.comm_thread.get_controller_data().left_y, 
                            lambda: self.comm_thread.get_controller_data().right_x).schedule()

        self.path_cmd = CreatePathCmd(self.path_creation, self.comm_thread, lambda: self.comm_thread.get_controller_data().btn_B)
        self.path_cmd.schedule()

    
    def disabled_init(self):
        logger.info("Robot disabled, stopping all movement")
        if self.path_cmd is not None:
            self.path_cmd.cancel()
        StopMovementCmd(self.drive_train).schedule()

    def shutdown(self):
        self.clutches.close()
        self.uwb.close_all()
        self.drive_train.stop()
        self.drive_train.close()
        self.back_Wheel_encoder.close()
        self.header_healer_switches.close()
        self.bms.close()
        self.pcb_leds.close()
        self.comm_thread.close()
        self.imu.close()
