
import setup_paths

# from bindings import trajopt_planner
import numpy as np

# robot type
from simulator.configs.rl030a import Config as rl030a
from simulator.configs.rl030b import Config as rl030b
from simulator.configs.ra830a import Config as ra830a
from simulator.configs.ra830b import Config as ra830b
# from simulator.configs.ra830l import Config

from simulator.trajopt_interface import TrajoptInterface
from simulator.motion_compendium.read_snapshots import read_case_data


ra830a_planner = TrajoptInterface(ra830a)
ra830b_planner = TrajoptInterface(ra830b)


if __name__ == "__main__":
    case = read_case_data('ra830a', 1)
    ra830a_planner.get_trajopt_results(case)

    case = read_case_data('ra830b', 1)
    ra830b_planner.get_trajopt_results(case)
    
