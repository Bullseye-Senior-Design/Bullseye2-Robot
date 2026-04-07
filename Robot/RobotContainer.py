from Robot.Commands.StopMovementCmd import StopMovementCmd
from Robot.Commands.CreatePathCmd import CreatePathCmd
from Robot.Commands.DefaultMovementCmd import DefaultMovementCmd
from Robot.Constants import Constants
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
from Robot.Commands.FollowPathCmd import FollowPathCmd
from Comms.PiCommThread import PiCommThread
from Robot.Commands.ParkingCmd import ParkingCmd
from Robot.Commands.DriveHomeCmd import DriveHomeCmd



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
        
        AlignIMUToWorldCmd(self.imu, self.uwb).schedule()  # Align IMU to UWB at startup
    
        
        # Start subsystems
        self.uwb.start(uwb_tag_data=Constants.uwb_tag_data, anchors_pos=None)
        self.back_Wheel_encoder.start()
        
        InputScheduler(lambda: self.comm_thread.get_controller_data().btn_A).on_true(
            CreatePathCmd(self.path_creation, lambda: self.comm_thread.get_controller_data().btn_B)
        )
        
        #self.path_following.default_command(FollowPathCmd(self.drive_train, self.path_following))
                    
    def begin_data_log(self):
        LogDataCmd(self.path_following).schedule()
        
    def start_parking(self):
        return_home_cmd = SequentialCommandGroup()
        return_home_cmd.add_commands(
            DriveHomeCmd(self.drive_train, self.path_following),
            ParkingCmd(self.drive_train, self.parking_controller)
        )
        return_home_cmd.schedule()


    def start_teleop(self):
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
