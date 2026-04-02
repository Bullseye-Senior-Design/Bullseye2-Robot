from Robot.Commands.DefaultMovementCmd import DefaultMovementCmd
from Robot.Constants import Constants
from structure.commands.InstantCommand import InstantCommand
from structure.commands.SequentialCommandGroup import SequentialCommandGroup
import time

from Robot.subsystems.sensors.UWB import UWB
from Robot.subsystems.sensors.IMU import IMU
from Robot.subsystems.PathFollowing import PathFollowing
from Robot.subsystems.DriveTrain import DriveTrain
from Robot.subsystems.BMS import BMS
from Robot.subsystems.PCBLEDs import PCBLEDs
import time

from Robot.subsystems.sensors.BackWheelEncoder import BackWheelEncoder
from Robot.subsystems.Clutches import Clutches
from Robot.subsystems.HeaderHealerSwitches import HeaderHealerSwitches

from Robot.Commands.LogDataCmd import LogDataCmd
from Robot.Commands.PlotStateCmd import PlotStateCmd
from Robot.Commands.MotorMovementExampleCmd import MotorMovementExampleCmd
from Robot.Commands.AlignIMUToWorldCmd import AlignIMUToWorldCmd
from Robot.Commands.ZeroIMUCmd import ZeroIMUCmd
from Robot.Commands.FollowPathCmd import FollowPathCmd
from Comms.PiCommThread import PiCommThread


class RobotContainer:
    def __init__(self):
        self.uwb = UWB()
        self.back_Wheel_encoder = BackWheelEncoder()
        self.imu = IMU()
        self.clutches = Clutches()
        self.path_following = PathFollowing()
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
        # self.drive_train.default_command(DefaultMovementCmd(self.drive_train, 
        #                                                     lambda: self.comm_thread.get_controller_data().left_y, 
        #                                                     lambda: self.comm_thread.get_controller_data().right_x))
        
        #self.path_following.default_command(FollowPathCmd(self.drive_train, self.path_following))
                    
    def begin_data_log(self):
        LogDataCmd(self.path_following).schedule()
        # ZeroIMUCmd(self.drive_train, self.path_following, schedule_followup=False).schedule()
        # PlotStateCmd().schedule()
        MotorMovementExampleCmd(self.drive_train, self.clutches, self.header_healer_switches, self.pcb_leds).schedule()
        
        # AlignIMUToWorldCmd(tau=0.5, duration=30.0).schedule()
                
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
