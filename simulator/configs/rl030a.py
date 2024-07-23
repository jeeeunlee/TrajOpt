import numpy as np


class Config(object):  
        
    ROBOT_FILE_NAME = "/config/urdf_files/dex/khi-rl030n-a.urdf"
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
    
    VELOCITY_LIMITS = [1.57080, 3.14160, 1.95130, 1.83720, 1.65350, 3.49070, 3.49070, 3.49070]
    ACCLERATION_LIMITS = [10.]*ROBOTDOF
    JERK_LIMITS = [500.]*ROBOTDOF
            
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

