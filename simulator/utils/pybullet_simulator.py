import time, math
from collections import OrderedDict
import copy
import signal
import shutil
import cv2
import pybullet as p
import numpy as np

import os
import sys

sys.path.append(os.getcwd())
sys.path.append(os.getcwd() + 'simulator/utils')
sys.path.append(os.getcwd() + 'simulator')

np.set_printoptions(precision=2)
import utils.pybullet_util as pybullet_util
from utils.interface_wrapper import RobotInterface


class Simulator():
    def __init__(self, 
                 Config, 
                 robot_interface:RobotInterface):
        
        # Environment Setup
        p.connect(p.GUI,  options="--opengl2")
        p.resetDebugVisualizerCamera(cameraDistance=3.0,
                                    cameraYaw=-65,
                                    cameraPitch=-10,
                                    cameraTargetPosition=[0, 0.0, 1.0])
        p.setGravity(0, 0, -9.8)
        p.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                                    numSubSteps=Config.N_SUBSTEP)
        if Config.VIDEO_RECORD:
            if os.path.exists(Config.VIDEO_DIR):
                shutil.rmtree(Config.VIDEO_DIR)
            os.makedirs(Config.VIDEO_DIR)
            
        # Create Robot, Ground, Environments
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        self.robot = p.loadURDF(Config.ROBOT_FILE_NAME, useFixedBase=1)
        p.loadURDF(os.getcwd() + "/simulator/configs/urdf_files/ground/plane.urdf", 
                   [0, 0, 0], useFixedBase=1)
        
        _, _, _, self.joint_id, self.link_id = pybullet_util.get_robot_config(
            self.robot, Config.PRINT_ROBOT_INFO)
    
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # Link Damping & Joint Friction
        p.changeDynamics(self.robot, -1, linearDamping=0., angularDamping=0.)
        pybullet_util.set_link_damping(self.robot, self.link_id.values(), 0., 0.)        
        pybullet_util.set_joint_friction(self.robot, self.joint_id, 0)
        
        # Enable 
        pybullet_util.enable_trq_data(self.robot, self.joint_id)
        
        # draw
        for k, v in self.link_id.items():
            if(k == "wrist_3_link"):
                pybullet_util.draw_link_frame(self.robot, v, linewidth=5.0, text=None)

        self.Config = Config
        self.robot_interface = robot_interface
        self.count = 0
        self.t = 0
        self.dt = Config.CONTROLLER_DT
        
    def set_init_config(self, init_joint_pos):        
        init_config = pybullet_util.get_joint_dict(self.joint_id, init_joint_pos)        
        pybullet_util.set_config(self.robot, self.joint_id, init_config)  
        
    def update_in_loop(self):
        tstart = time.time()
        
        # Get SensorData
        sensor_data_dict = pybullet_util.get_sensor_data(self.robot, self.joint_id)        
        jp, jv = self.robot_interface.get_command(self.t, sensor_data_dict)
        
        # Apply cmd
        pybullet_util.set_motor_pos_vel(self.robot, self.joint_id, jp, jv)

        # Save Image
        if (self.Config.VIDEO_RECORD) and (self.count % self.Config.RECORD_FREQ == 0):
            frame = pybullet_util.get_camera_image([1.2, 0.5, 1.], 2.0, 120,
                                                   -15, 0, 60., 1920, 1080,
                                                   0.1, 100.)
            frame = frame[:, :, [2, 1, 0]]  # << RGB to BGR
            filename = self.Config.VIDEO_DIR + '/step%06d.jpg' % self.count
            cv2.imwrite(filename, frame)

        p.stepSimulation()

        # time.sleep(dt)
        tsleep = self.dt-(time.time()-tstart)
        if(tsleep>0):
            time.sleep(tsleep)
        if self.Config.PRINT_TIME:            
            print(time.time()-tstart)
            
        self.count += 1
        self.t += self.dt

        
