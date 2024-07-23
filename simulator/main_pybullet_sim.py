
import os
import sys
sys.path.insert(-1, os.getcwd() + "/bazel-bin/")
from bindings import trajopt_planner

import numpy as np
from utils.pybullet_simulator import Simulator
import utils.pybullet_util as pybullet_util
# robot type
from simulator.configs.rl030a import Config
# from simulator.configs.ra830l import Config

from simulator.trajopt_planner import TrajoptInterface



def main():    
    interface = TrajoptInterface(Config)
    
    sim = Simulator(Config, interface)
    sim.set_init_config(list(Config.INIT_JOINT_CONFIG.values()))

    # Run Sim
    count_planning = 0 
    b_planning = True
    
    while (1):
        # planning 
        keys = pybullet_util.get_key_pressed()
        # if(len(keys)>0):
        #     if(keys[0]==32 and b_planning):
        if(sim.count==500):              
                print("key pressed")
                planning_cmd = interface.get_exmple_planning_cmd(
                    init_pos=np.array(list(Config.INIT_JOINT_CONFIG.values())),
                    goal_pos=np.array([0.7,0.7,0.7,0.7, 0.7,0.7,0.7,0.7]),
                    override = 0.3,
                )
                print("doPlanning")
                ret = interface.doPlanning(planning_cmd)
                print(f"ret = {ret}")
                b_planning=False
                count_planning = sim.count
        sim.update_in_loop()
        
        
if __name__ == "__main__":
    main()
    
