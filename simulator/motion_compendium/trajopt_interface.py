import setup_paths

from bindings import trajopt_planner
import numpy as np
from typing import Dict, List, Optional, Union

# wrapper for TrajoptInterface
class TrajoptInterface():
    def __init__(self, Config):
        self.Config = Config
        # self.interface = trajopt_planner.TestInterface(Config.ROBOT_FILE_NAME)
        self.interface = trajopt_planner.TestInterface(Config.ROBOT_NAME, Config.ASSETS_DIR)
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
    
    def build_obstacles(self, 
                        obstacle_dict: Dict, 
                        robot_base:np.ndarray):
        obstacles = []
        for k,v in obstacle_dict.items():
            obs_name = k
            obs = trajopt_planner.OBSTACLE()
            obs.dimension = v['info']['data']
            # do full transform later 
            # now just assume robot base orientation be identity
            p_wo = v['pose'] 
            obs.pose = [ p_wo[i]-robot_base[i] if i<3 else p_wo[i] for i in range(7) ]
            obstacles.append(obs)
        self.planning_cmd.obstacles = obstacles
        print( self.planning_cmd.obstacles )
    
    def get_sparse_joint_path(self, 
                              joint_path: np.ndarray, 
                              n_max: int = 20):
        n,d = joint_path.shape
        if(n < n_max): return joint_path
        m = (int)((n+n_max-1)/n_max)
        sparse_joint_path = []
        i = 0
        while (m*i < n-1-m*i - m):
            sparse_joint_path.insert(i, joint_path[m*i,:])
            sparse_joint_path.insert(i+1,joint_path[n-1-m*i,:])
            i = i + 1
        return np.array(sparse_joint_path)
    
    def get_trajopt_results(self, 
                            plan_data: Dict,                       
                            ):
        # set stuff
        joint_path = plan_data["joint_path"]
        obstacles = plan_data["obstacles"]
        max_joint_velocity = plan_data["max_joint_speed"]
        max_joint_acceleration = plan_data["max_joint_acceleration"]
        max_joint_jerk= plan_data["max_joint_jerk"]
        base_pose = plan_data['base_pose']
        # 
        self.planning_cmd.joint_path = self.get_sparse_joint_path(joint_path)
        # self.planning_cmd.cartesian_path         
        self.planning_cmd.max_joint_speed = max_joint_velocity        
        self.planning_cmd.max_joint_acceleration = max_joint_acceleration 
        self.planning_cmd.max_joint_jerk = max_joint_jerk    
        self.build_obstacles(obstacles, 
                             base_pose)
        print(self.planning_cmd.obstacles)
        self.planning_cmd.gripped_box.pose_from_ee =[ 0.0, 0.0, 0.2032, 1.0, 0.0, 0.0, 0.0]
        self.planning_cmd.gripped_box.dimension = [0.4064, 0.4064, 0.4064]
        self.interface.doPlanning(self.planning_cmd)

        self.interface.getPlannedResult(self.solution)
        if(self.Config.PRINT_RESULTS): self.printSolution()

        self.interface.getPlannedTrajectory(self.Config.CONTROLLER_DT, 
                                            self.traj_data)
        return self.traj_data


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