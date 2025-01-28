
import os
import sys
sys.path.insert(-1, os.getcwd() + "/bazel-bin/")
sys.path.append(os.getcwd())

from bindings import trajopt_planner

assets_dir = "/home/jelee/my_ws/TrajOpt/rtcl/assets"
robot_name = "ra830a"
def main():
    # Construct Interface    
    print(robot_name)
    print(" test interface bindings ")
    interface = trajopt_planner.TestInterface(robot_name, assets_dir )
    sensor_data = trajopt_planner.SensorData(8)
    command = trajopt_planner.RobotCommand(8)


if __name__ == "__main__":
    main()
