import numpy as np
import os
import csv
import matplotlib.pyplot as plt

# import matplotlib
# print(matplotlib.rcsetup.all_backends)
# matplotlib.use('tkagg')  

# Load points from a CSV file using the csv module
def load_points_from_csv(file_path):
    points = {"x": [], "y": [], "z": []}
    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            # print(row)
            points["x"].append(float(row["Position x"]))
            points["y"].append(float(row["Position y"]))
            points["z"].append(float(row["Position z"]))
    return points

# Plot points in 3D
def plot_3d_points(points, file_path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Scatter plot
    ax.scatter(points["x"], points["y"], points["z"], c='blue', marker='.', label='Points')
    
    # Set labels
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.set_title('point clouds')
    # ax.legend()

    # Set axis equal and turn off axis lines and labels
    plt.axis('equal')  # Ensures equal scaling for both axes
    plt.axis('off')  # Turns off the entire axis
    plt.grid(False)  # Turns off the grid
    
    # Show plot
    # plt.show()
    plt.savefig(file_path)

# Main function
if __name__ == "__main__":
    # Replace 'points.csv' with the path to your CSV file
    assets_directory = "/home/dexterity/ambyld/TrajOpt/rtcl/assets"
    robot_name = "ra830a"
    point_cloud_resolution = "2.0cm"
    robot_directory = "{}/point_clouds/{}".format(assets_directory, robot_name)
    point_cloud_directory = "{}/point_clouds/{}/{}".format(assets_directory, robot_name, point_cloud_resolution)
    
    csv_files = os.listdir(point_cloud_directory)
    for csv_file in csv_files:
        csv_file_path = "{}/{}".format(point_cloud_directory,csv_file)
        plot_file_path = "{}/plots/{}.pdf".format(robot_directory,csv_file.split(".")[0])
        # Load and plot points
        points = load_points_from_csv(csv_file_path)
        plot_3d_points(points, plot_file_path)
    

