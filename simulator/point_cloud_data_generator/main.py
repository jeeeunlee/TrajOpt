# Example from https://www.fwilliams.info/point-cloud-utils/sections/mesh_sampling/
# require python library point cloud utils 
# pip install point_cloud_utils / conda install conda-forge::point_cloud_utils

import point_cloud_utils as pcu
import numpy as np
import os
import csv
file_dir = os.path.dirname(__file__)
### 1. generate watertight meshes
data_folder = file_dir + "/graphics-hiwin-ra830-2475-gs/0_raw_files"
scadj_folder = file_dir + "/graphics-hiwin-ra830-2475-gs/1_scale_adjustment"
wt_folder = file_dir + "/graphics-hiwin-ra830-2475-gs/2_watertight_10_000"
pc_folder = file_dir + "/graphics-hiwin-ra830-2475-gs/3_point_clouds/100"
# for robot link data (except gripper), they're scaled in [mm] so we rescale it to [m]
for file in os.listdir(data_folder):
    print(file)
    v, f, n = pcu.load_mesh_vfn(filename= os.path.join(data_folder,file ))
    if(np.max(v)-np.min(v) > 10.0):
        v = v * 0.001
        pcu.save_mesh_vfn(os.path.join(scadj_folder, file ), v, f, n)

    # v is a [n, 3] shaped NumPy array of vertices
    # f is a [m, 3] shaped integer NumPy array of indices into v
    # n is a [n, 3] shaped NumPy array of vertex normals
    resolution = 10_000
    vw, fw = pcu.make_mesh_watertight(v, f, resolution)
    nw = pcu.estimate_mesh_vertex_normals(vw, fw)

    pcu.save_mesh_vfn(os.path.join(wt_folder, file ), vw, fw, nw)

    # Generate barycentric coordinates of random samples
    fid, bc = pcu.sample_mesh_poisson_disk(vw, fw, num_samples=100)
    # fid, bc = pcu.sample_mesh_poisson_disk(vw, fw, num_samples=-1, radius=0.005)

    # Interpolate the vertex positions and normals using the returned barycentric coordinates
    # to get sample positions and normals
    rand_positions = pcu.interpolate_barycentric_coords(fw, fid, bc, vw)
    rand_normals = pcu.interpolate_barycentric_coords(fw, fid, bc, nw)

    file_name_base = os.path.splitext(file)[0]
    with open(os.path.join(pc_folder, file_name_base ) + ".csv", "w") as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=",")
        header_row = ["Position x", "Position y", "Position z", "Normal x", "Normal y", "Normal z"]
        csv_writer.writerow(header_row)
        for position, normal in zip(rand_positions, rand_normals):
            csv_writer.writerow(position.tolist() + normal.tolist())