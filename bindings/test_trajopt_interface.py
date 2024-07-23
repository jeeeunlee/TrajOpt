
import os;

from bindings import trajopt_interface

panda_urdf = "/home/jelee/my_ws/TrajOpt/config/urdf_files/franka_panda.urdf"
dex_hiwin_urdf = "/home/jelee/my_ws/TrajOpt/config/urdf_files/dex/hiwin-ra830-2475-gs-l.urdf"
def main():
    # Construct Interface    
    print(dex_hiwin_urdf)
    print(" test interface bindings ")
    interface = trajopt_interface.TestInterface(dex_hiwin_urdf)
    sensor_data = trajopt_interface.SensorData(8)
    command = trajopt_interface.RobotCommand(8)


if __name__ == "__main__":
    main()
