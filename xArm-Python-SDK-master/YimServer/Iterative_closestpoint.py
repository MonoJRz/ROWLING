"""ICP (Iterative Closest Point) registration algorithm"""
import json
import open3d as o3d
import numpy as np
import copy
import os
import matplotlib.pyplot as plt

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp])
    
def point_to_point_icp(source, target, threshold, trans_init):
    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    print(reg_p2p)
    
    draw_registration_result(source, target, reg_p2p.transformation)
    return reg_p2p.transformation

def Center(source):
        # Convert to numpy array
    points_source = np.asarray(source.points)

    # Calculate min and max values
    min_values = np.min(points_source, axis=0)
    max_values = np.max(points_source, axis=0)
    print(min_values)
    print(max_values)
    # Calculate the midpoint for each axis
    mid_point = (min_values + ((max_values - min_values) / 2))
    print(mid_point)
    # Center the point cloud at the origin
    centered_points = points_source - mid_point
    print(centered_points)
    centered_pcd = o3d.geometry.PointCloud()
    centered_pcd.points = o3d.utility.Vector3dVector(centered_points)
    return centered_pcd

def rough_ICP(source,target):
        # Convert to numpy array
    points_source = np.asarray(source.points)
    points_target = np.asarray(target.points)

    # Calculate min and max values
    points_source_mean = np.mean(points_source, axis=0)
    points_target_mean = np.mean(points_target, axis=0)
    range_to_move=points_source_mean-points_target_mean
    print(range_to_move)
    return range_to_move

if __name__ == "__main__":
    source_raw = o3d.io.read_point_cloud(os.path.join("Final\material\saved_point_cloud.pcd"))

    target = o3d.io.read_point_cloud(os.path.join("Final\material\gather_point.pcd"))
    

    # Get the number of points
    num_points = np.asarray(target.points).shape[0]
    print("Number of points in the point cloud:", num_points)
    centered_pcd=Center(source_raw)

    translation=rough_ICP(centered_pcd,target)

    threshold = 5
    trans_init = np.asarray([[0, 0, 1, -translation[0]],
                             [1, 0, 0, -translation[1]],
                             [0, 1, 0, -translation[2]], 
                             [0.0, 0.0, 0.0, 1.0]])
    trans_init_0 = np.asarray([[0, 0, 1, 0],
                             [1, 0, 0, 0],
                             [0, 1, 0, 0], 
                             [0.0, 0.0, 0.0, 1.0]])
    draw_registration_result(centered_pcd, target, trans_init_0)
    draw_registration_result(centered_pcd, target, trans_init)

    print("Initial alignment")
    evaluation = o3d.pipelines.registration.evaluate_registration(
        centered_pcd, target, threshold, trans_init)
    print(evaluation, "\n")

    Transformation_matrix=point_to_point_icp(centered_pcd, target, threshold, trans_init)
    print(Transformation_matrix)


    # Convert the NumPy array to a list
    array_list = Transformation_matrix.tolist()
    # Serialize the list to a JSON string
    json_str = json.dumps(array_list)
    # Specify the directory where the file will be saved
    directory = 'Final/material'
    # Ensure the directory exists
    if not os.path.exists(directory):
        os.makedirs(directory)
    # Specify the path to the file
    file_path = os.path.join(directory, 'transformation_after_ICP.json')

    # Write the JSON string to the file
    with open(file_path, 'w') as file:
        file.write(json_str)
