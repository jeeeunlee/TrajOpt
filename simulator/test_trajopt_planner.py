
import setup_paths

from bindings import trajopt_planner
import numpy as np

# robot type
from simulator.configs.rl030a import Config as rl030a
from simulator.configs.rl030b import Config as rl030b
from simulator.configs.ra830a import Config as ra830a
from simulator.configs.ra830b import Config as ra830b
# from simulator.configs.ra830l import Config

from simulator.trajopt_interface import TrajoptPlanner


ra830a_planner = TrajoptPlanner(ra830a)
ra830b_planner = TrajoptPlanner(ra830b)


if __name__ == "__main__":
    ra830a_planner.get_trajopt_results(joint_path_a)
    ra830b_planner.get_trajopt_results(joint_path_b)
    
