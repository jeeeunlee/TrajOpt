
import os;

from bindings import trajopt_planner

panda_urdf = "/home/jelee/my_ws/TrajOpt/simulator/configs/urdf_files/franka_panda.urdf"
dex_hiwin_urdf = "/home/jelee/my_ws/TrajOpt/simulator/configs/urdf_files/dex/ra830_2475_gs_a_v4.urdf"
def main():
    # Construct Interface    
    print(dex_hiwin_urdf)
    print(" test interface bindings ")
    interface = trajopt_planner.TestInterface(dex_hiwin_urdf)
    sensor_data = trajopt_planner.SensorData(8)
    command = trajopt_planner.RobotCommand(8)


if __name__ == "__main__":
    main()
