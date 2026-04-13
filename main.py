from Debug import Debug
from Robot.Robot import Robot
from structure.RobotState import RobotState
from Robot.subsystems.sensors.UWB import UWB
from Comms.PiCommThread import PiCommThread

import time
import logging
import sys

# Configure root logger for the entire application
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('robot_debug.log')
    ]
)

def main():
    robot = Robot()
    robot_state = RobotState()

    pi_comm_thread = PiCommThread()

    robot.robot_init()
    try: 
        while True:        
            # Run periodic functions
            robot.robot_periodic()
            
            # Teleop (human operated) mode
            if robot_state.should_init_teleop():
                print("Teleop mode enabled")
                robot.teleop_init()
            
            if robot_state.is_teleop_enabled():
                robot.teleop_periodic()

            if robot_state.should_init_path_creation():
                print("Path creation mode enabled")
                robot.path_creation_init()

            if robot_state.is_path_creation_enabled():
                robot.path_creation_periodic()

            if robot_state.should_init_return_to_home():
                print("Return to home mode enabled")
                robot.return_to_home_init()

            if robot_state.is_return_to_home_enabled():
                robot.return_to_home_periodic()

            # Autonomous mode
            if robot_state.should_init_autonomous():
                print("Autonomous mode enabled")
                robot.autonomous_init()

            if robot_state.is_autonomous_enabled():
                # print("Running autonomous periodic function")
                robot.autonomous_periodic()
            
            # Test mode
            if robot_state.should_init_test():
                print("Test mode enabled")
                robot.test_init()
            
            if robot_state.is_test_enabled():
                robot.test_periodic()
            
            # Disabled check
            if robot_state.should_init_disable():
                robot.disabled_init()

            pi_comm_thread.reset_kfx_button_data()  # Reset KFX button states after processing each loop
            
            # Add a small delay to prevent high CPU usage
            time.sleep(0.01)
    
    except Exception as e:
        print(f"An error occurred: {e}")
        robot.shutdown()
        

        
if __name__ == "__main__":
    main()