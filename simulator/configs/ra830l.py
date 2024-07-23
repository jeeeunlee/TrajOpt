import numpy as np


class Config(object):  
        
    ROBOT_FILE_NAME = "/config/urdf_files/dex/hiwin-ra830-2475-gs-l.urdf"
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
            
    CONTROLLER_DT = 0.001
    N_SUBSTEP = 1
    CAMERA_DT = 0.05
    KP = 0.
    KD = 0.

    PRINT_TIME = False #True #False
    PRINT_ROBOT_INFO = False
    VIDEO_RECORD = False
    VIDEO_DIR = ''
    RECORD_FREQ = 50
    SIMULATE_CAMERA = False

    KP, KD = dict(), dict()

