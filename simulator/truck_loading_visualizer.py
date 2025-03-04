import time
import pybullet as p
import numpy as np
import shutil
import os
import sys
import json
import random

np.set_printoptions(precision=2)

# Setup paths
robot_name = "ra830a_box"
cwd_path = os.getcwd()

# Fixing asset directory path
assets_dir = os.path.join(cwd_path, "rtcl", "assets")
robot_file_path = os.path.join(assets_dir, 'urdfs', robot_name + '.urdf')
json_dir = os.path.join(cwd_path, 'test', 'testdata', 'furniture_task')
json_file_path = os.path.join(json_dir, 'ra830a', 'case1.json')
traj_dir = os.path.join(cwd_path, "experiment_results", "results")

gripped_box_pose = [0.0, 0.0, 0.2032, 1.0, 0.0, 0.0, 0.0]
gripped_box_dimension = [0.4064, 0.4064, 0.4064]

# Assuming pybullet_util and RobotInterface are imported from external utilities
import utils.pybullet_util as pybullet_util

# Load the JSON file
def load_json(json_file: str)->dict:
    with open(json_file, 'r') as file:
        return json.load(file)
    
def load_traj_txt(traj_dir: str)->dict:
    traj = dict()
    traj['t'] = np.loadtxt(os.path.join(traj_dir, 'traj_t.txt'))
    traj['q'] = np.loadtxt(os.path.join(traj_dir, 'traj_q.txt'))
    traj['dq'] = np.loadtxt(os.path.join(traj_dir, 'traj_dq.txt'))
    return traj
        
class PyBulletMotionVisualizer:
    def __init__(self, 
                 robot_file: str = robot_file_path,
                 json_file: str = json_file_path,
                 traj_file: str = traj_dir):
        # Connect to PyBullet and configure settings
        self.count = 0
        self.t = 0
        self.dt = 0.001
        config = load_json(json_file)
        result = load_traj_txt(traj_file)
        self.init_pybullet()

        self.set_robot(robot_file, config["joint_path"][0])
        # self.set_gripped_box(gripped_box_pose, gripped_box_dimension)
        self.set_obstacles(config["obstacles"])
        while True:            
            path_ids = self.display_robot_path(config["joint_path"], dt=0.1)
            print('done')
            time.sleep(1)            
            traj_ids = self.display_robot_traj(result)
            time.sleep(5)   
            self.remove_items(path_ids)
            self.remove_items(traj_ids)

    # Function to move the robot according to the joint path
    def display_robot_path(self, 
                   joint_path: list[list],  
                   dt: float =0.01):
        path_lines = []
        current_link_pose = self.set_robot_pos(joint_path[0])
        for joint_pos in joint_path:
            # Set the joint positions
            next_link_pose = self.set_robot_pos(joint_pos)
            line_id = self.draw_line(current_link_pose, next_link_pose)
            current_link_pose = next_link_pose
            path_lines.append(line_id)
            time.sleep(dt)
        return path_lines
    
    def display_robot_traj(self, traj_result:dict):
        traj_lines = []
        current_link_pose = self.set_robot_pos(traj_result['q'][0])
        t_prev = 0
        for t,q,dq in zip(traj_result['t'],traj_result['q'],traj_result['dq']):
            # Set the joint positions
            next_link_pose = self.set_robot_pos_vel(q,dq)
            line_id = self.draw_line(current_link_pose, next_link_pose, color=[0,1,1])
            current_link_pose = next_link_pose
            traj_lines.append(line_id)
            time.sleep(t-t_prev)
            t_prev = t
        return traj_lines
    
    def remove_items(self, item_ids):
        for id in item_ids:
            p.removeUserDebugItem(id)

    def draw_line(self,
            start_pose,
            end_pose,
            color=[1, 1, 1], width=2):
        line_id = p.addUserDebugLine(start_pose[0], end_pose[0], lineColorRGB=color, lineWidth=width)
        return line_id

    def set_robot_pos(self, joint_pos: list[float]):        
        joint_pos_dict = pybullet_util.get_joint_dict(self.joint_id, joint_pos)        
        pybullet_util.set_config(self.robot, self.joint_id, joint_pos_dict) 
        return p.getLinkState(self.robot, self.end_effector_link_id)
    
    def set_robot_pos_vel(self, 
                          joint_pos: np.ndarray,
                          joint_vel: np.ndarray):
        for i, (q, v) in enumerate(zip(joint_pos, joint_vel)):
            p.resetJointState(self.robot, i, q, v)
        return p.getLinkState(self.robot, self.end_effector_link_id)


    
    # initialize setup

    def set_robot(self,
                  robot_urdf: str,
                  init_joint_pos: list[float]):
        # Load Robot and environment URDF files
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        self.robot = p.loadURDF(robot_urdf, useFixedBase=1)
        # Retrieve robot configuration (joint and link IDs)
        _, _, _, self.joint_id, self.link_id = pybullet_util.get_robot_config(self.robot)
        self.dim = len(self.joint_id)
        
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # Set link damping and joint friction
        p.changeDynamics(self.robot, -1, linearDamping=0., angularDamping=0.)
        pybullet_util.set_link_damping(self.robot, self.link_id.values(), 0., 0.)        
        pybullet_util.set_joint_friction(self.robot, self.joint_id, 0)
        
        # Enable torque data if needed
        pybullet_util.enable_trq_data(self.robot, self.joint_id)

        # Draw link frames for visualization (e.g., wrist_3)
        for k, v in self.link_id.items():
            if k == "wrist_3":
                pybullet_util.draw_link_frame(self.robot, v, linewidth=5.0, text=None)
                self.end_effector_link_id = v

        # set init config
        self.set_robot_pos(init_joint_pos)    


    def set_obstacles(self, obstacles:dict):
        # p.loadURDF(os.getcwd() + "/simulator/configs/urdf_files/ground/plane.urdf",
        #            [0, 0, 0], useFixedBase=1)
        
        self.obstacle_ids = []        
        for name, data in obstacles.items():
            # mesh_file = os.path.join(json_dir, data["info"]["data"])
            a = 0.5 if name in ["wallLeft", "wallRight", "wallBack", "floor", "ceiling"] else 1

            rgba_color = [0.5*random.random(), random.random(), random.random(), a]


            obj_file = json_dir + data["info"]["data"]
            if os.path.exists(obj_file):
                # Create Collision Shape from .obj
                collision_shape = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=obj_file)
                
                # Create MultiBody for the object with visual and collision information                
                visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=obj_file, rgbaColor=rgba_color)
                
                # You can set position, orientation, and mass for the obstacle
                obstacle_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape,
                                                baseVisualShapeIndex=visual_shape,
                                                basePosition=[0, 0, 0])  # Adjust position as needed
                self.obstacle_ids.append(obstacle_id)
            else:
                print(f"Warning: Obj file '{obj_file}' not found.")



    def init_pybullet(self):        
        # Initialize PyBullet GUI and set the camera view
        p.connect(p.GUI) #, options="--opengl2"
        p.resetSimulation()
        p.resetDebugVisualizerCamera(cameraDistance=3.0,
                                     cameraYaw=-65,
                                     cameraPitch=-10,
                                     cameraTargetPosition=[0, 0.0, 1.0])
        p.setGravity(0, 0, -9.8)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt,
                                    numSubSteps=1)


if __name__ == "__main__":
    # main()    
    sim = PyBulletMotionVisualizer()