import numpy as np
import os
file_dir = os.path.dirname(__file__)
# print(file_dir)

class Config(object):          
    ROBOT_FILE_NAME = file_dir + "/urdf_files/dex/ra830_2475_gs_a_v4.urdf"
    PYTHON_RUN_SCRIPT = None
    ROBOTDOF = 8
    INIT_JOINT_CONFIG = {'j1': 0.0,
                        'j2': 0.0,
                        'j3': 0.0,
                        'j4': 0.0,
                        'j5': 0.0,
                        'j6': 0.0,
                        'j7': 0.0,
                        'j8': 0.0}
    
    VELOCITY_LIMITS = [2.13710, 3.20560, 2.92690, 3.23630,
                       2.87210, 5.81770, 7.11060, 7.19950]
    ACCLERATION_LIMITS = [10.]*ROBOTDOF
    JERK_LIMITS = [500.]*ROBOTDOF
            
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1
    CAMERA_DT = 0.05
    KP = 0.
    KD = 0.

    PRINT_TIME = False #True #False
    PRINT_ROBOT_INFO = False
    PRINT_RESULTS = True
    VIDEO_RECORD = False
    VIDEO_DIR = ''
    RECORD_FREQ = 50
    SIMULATE_CAMERA = False

    KP, KD = dict(), dict()

