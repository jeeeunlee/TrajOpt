import setup_paths

from bindings import trajopt_planner
from simulator.utils.interface_wrapper import RobotInterface
import numpy as np


# wrapper for TrajoptInterface
class TrajoptPlanner(RobotInterface):
    def __init__(self, Config):
        self.Config = Config
        self.interface = trajopt_planner.NoColTestInterface(Config.ROBOT_FILE_NAME)
        self.sensor_data = trajopt_planner.SensorData(Config.ROBOTDOF)
        self.command = trajopt_planner.RobotCommand(Config.ROBOTDOF)
        self.planning_cmd = trajopt_planner.PLANNING_COMMAND()
        self.solution  = trajopt_planner.SOLUTION()
        self.traj_data = trajopt_planner.TRAJ_DATA()

    def get_command(self, 
                    t:float,  
                    sensor_data_dict):
        self.sensor_data.elapsed_time = t
        self.sensor_data.joint_positions = sensor_data_dict["joint_pos"]
        self.sensor_data.joint_velocities = sensor_data_dict["joint_vel"]
        self.interface.getCommand(self.sensor_data, self.command)
        return self.command.joint_positions, self.command.joint_velocities
    
    def get_exmple_planning_cmd(self, 
                    init_pos: np.ndarray, 
                    goal_pos: np.ndarray,
                    override: float):
        N = 10
        unif_path = np.linspace(init_pos, goal_pos, num=N)
        devi_path = np.array([a*np.sin( np.linspace(0, np.pi, num=N ) ) 
                              for a in np.random.normal(0.0,0.5,self.Config.ROBOTDOF)])
        devi_path = np.transpose(devi_path)
        self.planning_cmd.joint_path = unif_path + devi_path
        # self.planning_cmd.cartesian_path 
        self.planning_cmd.max_joint_speed = override*np.array(self.Config.VELOCITY_LIMITS)
        self.planning_cmd.max_joint_acceleration = override*np.array(self.Config.ACCLERATION_LIMITS)
        self.planning_cmd.max_joint_jerk = override*np.array(self.Config.JERK_LIMITS)
        return self.planning_cmd
    
    def get_trajopt_results(self, 
                            joint_path: np.ndarray,
                            override: float = 0.5, # 0.1~1.0
                            ):
        self.planning_cmd.joint_path = joint_path
        # self.planning_cmd.cartesian_path 
        self.planning_cmd.max_joint_speed = override*np.array(self.Config.VELOCITY_LIMITS)
        self.planning_cmd.max_joint_acceleration = override*np.array(self.Config.ACCLERATION_LIMITS)
        self.planning_cmd.max_joint_jerk = override*np.array(self.Config.JERK_LIMITS)

        self.interface.doPlanning(self.planning_cmd)

        self.interface.getPlannedResult(self.solution)
        if(self.Config.PRINT_RESULTS): self.printSolution()

        self.interface.getPlannedTrajectory(self.Config.CONTROLLER_DT, 
                                            self.traj_data)     

        return self.traj_data.qdata


    def doPlanning(self, 
                   planning_cmd: trajopt_planner.PLANNING_COMMAND):
        return self.interface.doPlanning(planning_cmd)
    
    def printSolution(self):
        print(self.solution.h)
        print('\n')
        print(self.solution.path)
        print('\n')
        print(self.solution.velocity)
        print('\n')
        print(self.solution.acceleration)
        print('\n')
        print(self.solution.jerk)