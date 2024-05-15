import os
import numpy as np
import open3d as o3d
import time

def get_file_list (path, suffix=""):
  in_list = os.listdir(path)
  out_list = []

  for file in in_list:
    if ( file.split('.')[1] == suffix or suffix == "" ):
      out_list.append( f"{path}/{file}" )

  return out_list

def read_PCD_from_file (path):
  return o3d.io.read_point_cloud(path)


if __name__ == "__main__":
  geometry_list = get_file_list("./lidar_frames")
  # print(geometry_list)
  first_geometry = read_PCD_from_file (geometry_list[0])
  
  geometry = o3d.geometry.PointCloud()
  geometry.points = o3d.utility.Vector3dVector(first_geometry)
  o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

  vis = o3d.visualization.Visualizer()
  vis.create_window()
  vis.add_geometry([geometry])

  for geometry_file in geometry_list:
    pc_geometry = read_PCD_from_file (geometry_file)
    print(geometry_file)
    vis.update_geometry(pc_geometry)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(1000)
    
  # pc_geometry = read_PCD_from_file (geometry_list[0])
  # o3d.visualization.draw_geometries([pc_geometry])
  # time.sleep(1000)
  # time.sleep(1000)

  vis.destroy_window()

# center = np.array([0, 0, 0])
# r = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
# size = np.array([10, 5, 5])
# sample_3d = o3d.geometry.OrientedBoundingBox(center, r, size)

# o3d.visualization.draw_geometries([pc_pcd, sample_3d],
#                                   zoom=0.7,
#                                   front=[0.5439, -0.2333, -0.8060],
#                                   lookat=[2.4615, 2.1331, 1.338],
#                                   up=[-0.1781, -0.9708, 0.1608])

# voxel_grid = o3d.geometry.VoxelGrid
# create_from_point_cloud(pc_pcd,voxel_size=0.40)
# o3d.visualization.draw_geometries([voxel_grid])