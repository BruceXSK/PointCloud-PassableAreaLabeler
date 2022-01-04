import os
from typing import List
import open3d as o3d
import numpy as np
import struct
from utils import transformations, point_cloud_ops as pc_ops
from dataset import parse_point_cloud


class Params:
    frame_num = 5
    t_thresh_temporal = 1.0
    t_thresh_data = 2.0


def matrix_to_rt(mat: np.ndarray):
    t = np.linalg.norm(mat[:2, -1])
    _, _, yaw = transformations.euler_from_matrix(mat)
    return yaw, t


def pose_to_matrix(pose: np.ndarray):
    # pose: [x, y, z, x, y, z, w]
    r = transformations.quaternion_matrix(pose[3:])
    t = transformations.translation_matrix(pose[:3])
    return t @ r


def cluster_data_from_pose_(pose_list: List[np.ndarray], cluster: List[int]):
    last_data_idx = cluster[-1]
    pose_last = pose_list[last_data_idx]
    mat_last = pose_to_matrix(pose_last)
    mat_last_inv = np.linalg.inv(mat_last)
    found = False
    for i in range(last_data_idx - 1, -1, -1):
        pose_curr = pose_list[i]
        mat_curr = pose_to_matrix(pose_curr)
        pose_mat_rel = mat_last_inv @ mat_curr
        yaw, t = matrix_to_rt(pose_mat_rel)
        if abs(t) > Params.t_thresh_temporal:
            cluster.append(i)
            found = True
            break
    if not found or len(cluster) == Params.frame_num:
        return
    cluster_data_from_pose_(pose_list, cluster)


def cluster_data_from_pose(pose_list: List[np.ndarray]):
    clusters = []
    for data_idx in range(len(pose_list) - 1, -1, -1):
        if len(clusters) != 0:
            mat_last = pose_to_matrix(pose_list[clusters[-1][0]])
            mat_curr = pose_to_matrix(pose_list[data_idx])
            pose_mat_rel = np.linalg.inv(mat_last) @ mat_curr
            yaw, t = matrix_to_rt(pose_mat_rel)
            if abs(t) < Params.t_thresh_data:
                continue
        cluster = [data_idx]
        cluster_data_from_pose_(pose_list, cluster)
        if len(cluster) == Params.frame_num:
            clusters.append(cluster)
        else:
            return clusters


def parse_pose(path: str) -> np.ndarray:
    f = open(path, 'rb')
    data = f.read()
    p = np.array(struct.unpack('7d', data))  # [position [x y z] orientation [x y z w]]
    f.close()
    return p


def save_clusters(path: str, clusters: List[List[int]]):
    f = open(path, 'w')
    for i, cluster in enumerate(clusters):
        s = str(cluster)[1:-1].replace(',', '')
        if i != len(clusters) - 1:
            s += '\n'
        f.write(s)
    f.close()


def main():
    data_series_name = 'IB-NorthEast'
    pcd_dir = f'data/{data_series_name}/pcd'
    pos_dir = f'data/{data_series_name}/ndt_pose'
    data_to_label_save_path = f'data/{data_series_name}/data_to_label.txt'
    pose_files = sorted(os.listdir(pos_dir))
    pose_list = []
    pc_path_list = []
    for pose_file in pose_files:
        pose_list.append(parse_pose(os.path.join(pos_dir, pose_file)))
        pc_path_list.append(os.path.join(pcd_dir, pose_file.split('.')[0] + '.pcd'))
    clusters = cluster_data_from_pose(pose_list)

    save_clusters(data_to_label_save_path, clusters)
    # for cluster in clusters:
    #     pc_o3d_list = []
    #     for i, idx in enumerate(cluster):
    #         pc_np = parse_point_cloud(pc_path_list[idx])
    #         pc_o3d = o3d.geometry.PointCloud()
    #         if i != 0:
    #             pc_np = pc_ops.transform_point_cloud(pc_np, pose_list[idx], pose_list[cluster[0]])
    #         pc_o3d.points = o3d.utility.Vector3dVector(pc_np)
    #         pc_o3d_list.append(pc_o3d)
    #     o3d.visualization.draw_geometries(pc_o3d_list)


if __name__ == '__main__':
    main()
