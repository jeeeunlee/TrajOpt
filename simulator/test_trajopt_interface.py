
import os
import sys
sys.path.insert(-1, os.getcwd() + "/bazel-bin/")
from bindings import trajopt_interface

import numpy as np
from utils.pybullet_simulator import Simulator
import utils.pybullet_util as pybullet_util
# robot type
from simulator.configs.rl030a import Config
# from simulator.configs.ra830l import Config

from simulator.trajopt_interface import TrajoptInterface

def main():    
    interface = TrajoptInterface(Config)
    solution = trajopt_interface.SOLUTION()
    trajopt = interface.interface

    planning_cmd = interface.get_exmple_planning_cmd(
        init_pos=np.array(list(Config.INIT_JOINT_CONFIG.values())),
        goal_pos=np.array([0.7,0.7,0.7,0.7, 0.7,0.7,0.7,0.7]),
        override = 0.3,
    )

    trajopt.doPlanning(planning_cmd)
    trajopt.getPlannedResult(solution)

    print(solution.h)
    print('\n')
    print(solution.path)
    print('\n')
    print(solution.velocity)
    print('\n')
    print(solution.acceleration)
    print('\n')
    print(solution.jerk)

if __name__ == "__main__":
    main()
    
