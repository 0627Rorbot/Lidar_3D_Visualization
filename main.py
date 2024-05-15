import os
import open3d as o3d
from json import load
import numpy as np
import time


def get_file_list (path, suffix=""):
  in_list = os.listdir(path)
  out_list = []

  for file in in_list:
    if ( file.split('.')[1] == suffix or suffix == "" ):
      out_list.append( f"{path}/{file}" )

  return out_list

def roty(t):
    """
    Rotation about the y-axis.
    """
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


def box_center_to_corner(box):
        
    translation = box[3:6]
    h, w, l = box[0], box[1], box[2]
    #if the angle value is in radian then use below mentioned conversion
    # rotation_y = box[6]
    # rotation = rotation_y * (180/math.pi)                             #rad to degree
    rotation = box[6]

    # Create a bounding box outline if x,y,z is center point then use defination bounding_box as mentioned below
    # bounding_box = np.array([
    #     [-l/2, -l/2, l/2, l/2, -l/2, -l/2, l/2, l/2],
    #     [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2],
    #     [-h/2, -h/2, -h/2, -h/2, h/2, h/2, h/2, h/2]])
    
    # Create a bounding box outline if x,y,z is rear center point then use defination bounding_box as mentioned below
    bounding_box = np.array([
                [l,l,0,0,l,l,0,0],                      
                [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2],          
                [0,0,0,0,h,h,h,h]])                        

    # Standard 3x3 rotation matrix around the Z axis
    rotation_matrix = np.array([
        [np.cos(rotation), -np.sin(rotation), 0.0],
        [np.sin(rotation), np.cos(rotation), 0.0],
        [0.0, 0.0, 1.0]])

    # Standard 3x3 rotation matrix around the X axis
    # rotation_matrix = np.array([
    #     [1.0, 0.0, 0.0],
    #     [0.0, np.sin(rotation), np.cos(rotation)],
    #     [0.0, 0.0, 1.0]])

    #rotation_matrix = roty(rotation)

    # Repeat the [x, y, z] eight times
    eight_points = np.tile(translation, (8, 1))


    # Translate the rotated bounding box by the
    # original center position to obtain the final box
    corner_box = np.dot(
        rotation_matrix, bounding_box) + eight_points.transpose()

    return corner_box.transpose()

def get_boxes_from_files(path):
    out_list = []
    with open(path, 'r', encoding='utf-8') as f:
        json_data = load(f)
        #box = [h,w,l,x,y,z,rot]
        for data in json_data:
            # print(data)
            box = [
                data['dimensions']['y'],
                data['dimensions']['x'],
                data['dimensions']['z'],
                data['center']['x'],
                data['center']['y'],
                data['center']['z'],
                0.02
            ]
            out_list.append(box)

    return out_list

# pcd1 = o3d.io.read_point_cloud('./000.pcd')
# points_v = np.asarray(pcd1.points)
# mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0,0,0])
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points_v)
# entities_to_draw = [pcd, mesh_frame]
vis_transform = [[1,0,0,0],[0,0.5,0,0],[0,0,-1,0],[0,0,0,1]] 

vis = o3d.visualization.Visualizer()
vis.create_window(visible=True)

pcd = o3d.io.read_point_cloud('./000.pcd')
# pcd.transform(vis_transform)

vis.add_geometry(pcd)
#box = [h,w,l,x,y,z,rot]
file_list = get_file_list('./detections_per_frame')
i = 0

drawed_boxes = []

for file in file_list:
    i += 1
    i %= len(file_list)
    temp = ''
    if i < 10:
       temp = f"./lidar_frames/00{i}.pcd"
    elif i < 100:
       temp = f"./lidar_frames/0{i}.pcd"
    else:
       temp = f"./lidar_frames/{i}.pcd"

    pcd1 = o3d.io.read_point_cloud(temp)

    # vis.update_geometry(pcd1)

    vis.remove_geometry(pcd,False)
    pcd1.transform(vis_transform)
    vis.add_geometry(pcd1)

    vis.poll_events()
    vis.update_renderer()


    drawed_boxes = []
    # pcd1.transform(vis_transform)

    boxes = get_boxes_from_files(file)
    for box in boxes:    
        boxes3d_pts = box_center_to_corner(box)
        boxes3d_pts = boxes3d_pts.T
        boxes3d_pts = o3d.utility.Vector3dVector(boxes3d_pts.T)
        box = o3d.geometry.OrientedBoundingBox.create_from_points(boxes3d_pts)
        box.color = [1, 0, 0]           #Box color would be red box.color = [R,G,B]
        # entities_to_draw.append(box)
        vis.add_geometry(box)
        drawed_boxes.append(box)

    vis.update_renderer()
    time.sleep(1/20)

    # vis.remove_geometry(boxes)
    
    for box in drawed_boxes:
        print("deletee points")
        # vis.remove_geometry(box)
        # box.clear()
        print("deleted points")

    # for box in boxes:
    #     box.clear()
    

    vis.remove_geometry(pcd1)
    # Draw
    # o3d.visualization.draw_geometries(
    #     [*entities_to_draw],
    #     front = [-0.9945, 0.03873, 0.0970],
    #     lookat = [38.4120, 0.6139, 0.48500],
    #     up = [0.095457, -0.0421, 0.99453],
    #     zoom = 0.33799
    # )
vis.destroy_window()
# print('ddd')
# o3d.visualization.draw_geometries(
#     [*entities_to_draw],
#     front = [-0.9945, 0.03873, 0.0970],
#     lookat = [38.4120, 0.6139, 0.48500],
#     up = [0.095457, -0.0421, 0.99453],
#     zoom = 0.33799
# )