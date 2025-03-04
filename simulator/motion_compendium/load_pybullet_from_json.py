import json
import pybullet as p
import time
import numpy as np
import os

# Setup paths
robot_name = "ra830a"
cwd_path = os.getcwd()

# Fixing asset directory path
assets_dir = os.path.join(cwd_path, "rtcl", "assets")
robot_file_path = os.path.join(assets_dir, 'urdfs', robot_name + '.urdf')
json_dir = os.path.join(cwd_path, 'test', 'testdata', 'furniture_task')
json_file_path = os.path.join(json_dir, 'ra830a', 'case1.json')

# Load the JSON file
def load_json(json_file):
    with open(json_file, 'r') as file:
        return json.load(file)

# Function to load obstacles
def load_obstacles(obstacles):
    obstacle_ids = {}
    for name, data in obstacles.items():
        # mesh_file = os.path.join(json_dir, data["info"]["data"])
        mesh_file = json_dir + data["info"]["data"]
        pose = data["pose"]
        
        # Check if the mesh file exists
        if not os.path.isfile(mesh_file):
            print(f"Warning: Mesh file {mesh_file} does not exist.")
            continue

        # Load the visual and collision shapes of the mesh
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                              fileName=mesh_file,
                                              rgbaColor=[0.8, 0.8, 0.8, 1],
                                              meshScale=[1, 1, 1])  # Adjust scale if necessary
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                                    fileName=mesh_file,
                                                    meshScale=[1, 1, 1])  # Adjust scale if necessary
        
        # Create a multi-body with the visual and collision shapes
        obstacle_id = p.createMultiBody(baseMass=0,  # Mass = 0 for static object
                                        baseVisualShapeIndex=visual_shape_id,
                                        baseCollisionShapeIndex=collision_shape_id,
                                        basePosition=[pose[0], pose[1], pose[2]],
                                        baseOrientation=[pose[3], pose[4], pose[5], pose[6]])
        
        obstacle_ids[name] = obstacle_id
    return obstacle_ids

# Function to move the robot according to the joint path
def move_robot(robot_id, joint_path, joint_ids, dt=0.01):
    for joint_pos in joint_path:
        joint_config = dict(zip(joint_ids, joint_pos))  # Map joint ids to joint positions
        # Set the joint positions
        for joint_id, pos in joint_config.items():
            p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, targetPosition=pos)
        p.stepSimulation()
        time.sleep(dt)

# Main simulation function
def main():    
    # Load JSON data
    config = load_json(json_file_path)
    
    # Connect to PyBullet (in GUI mode)
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=3.0,
                                cameraYaw=0,
                                cameraPitch=0,
                                cameraTargetPosition=[0, 0.0, 1.0])
    
    # Load the robot model (URDF)
    robot_id = p.loadURDF(robot_file_path, useFixedBase=True, globalScaling=0.001)
    
    # Get joint ids (assuming the robot has 8 joints)
    joint_ids = list(range(p.getNumJoints(robot_id)))  # Replace with correct joint IDs if necessary
    
    # Load obstacles
    obstacle_ids = load_obstacles(config["obstacles"])
    
    # Load the joint path
    joint_path = config["joint_path"]
    
    # Run the simulation and follow the joint path
    move_robot(robot_id, joint_path, joint_ids)
    
    # End simulation
    p.disconnect()

if __name__ == "__main__":
    main()
