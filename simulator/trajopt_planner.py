import setup_paths

from trajopt_interface import TrajoptInterface
from configs.ra830a import Config as ra830a
from configs.ra830b import Config as ra830b

from typing import Dict, List, Optional, Union
import numpy as np
from dataclasses import dataclass

@dataclass(frozen=True)
class RobotTrajectoryData:
    """
    A dataclass containing information about a robot trajectory.
    """

    se3_trajectory: Optional[np.ndarray] = None
    cartesian_velocities: Optional[np.ndarray] = None
    joint_trajectory: Optional[np.ndarray] = None
    joint_velocities: Optional[np.ndarray] = None
    duration: float = float("NaN")


class TrajoptPlanner():
    def __init__(self):
        self.ra830a_planner = TrajoptInterface(ra830a)
        self.ra830b_planner = TrajoptInterface(ra830b)
    
    def compute_trajopt(self,
        plan_data: Dict,
        dt: Optional[float] = None,
    ) -> RobotTrajectoryData:
        
        if(plan_data["robot_name"] =='ra830a'):
            plan_data['base_pose'] = [0.0, 0.25, 0, 1, 0, 0, 0]
            traj_data = self.ra830a_planner.get_trajopt_results(plan_data)
        elif(plan_data["robot_name"]  =='ra830b'):
            plan_data['base_pose'] = [0.0, -0.25, 0, 1, 0, 0, 0],
            traj_data = self.ra830b_planner.get_trajopt_results(plan_data)
        else:
            raise Exception("unknown robot_name")

        return RobotTrajectoryData(joint_trajectory=np.array(traj_data.qdata[1:-1]), 
                                   joint_velocities=np.array(traj_data.dqdata[1:-1]), 
                                   duration=traj_data.tdata[-1])
    

    
    

    