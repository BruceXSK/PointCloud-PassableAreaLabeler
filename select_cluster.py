import os
from typing import List
import open3d as o3d
import numpy as np
import struct
from utils import transformations, point_cloud_ops as pc_ops


class Params:
    frame_num = 5
    yaw_threshold = 3.14
    t_threshold = 0.5
    pass


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
        if abs(yaw) > Params.yaw_threshold or abs(t) > Params.t_threshold:
            cluster.append(i)
            found = True
            break
    if not found or len(cluster) == Params.frame_num:
        return
    cluster_data_from_pose_(pose_list, cluster)


def cluster_data_from_pose(pose_list: List[np.ndarray]):
    clusters = []
    for data_idx in range(len(pose_list) - 1, -1, -1):
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


def main():
    pcd_dir = 'data/IB-2L/pcd'
    pos_dir = 'data/IB-2L/pos'
    pose_files = sorted(os.listdir(pos_dir))
    pcd_files = sorted(os.listdir(pcd_dir))
    pose_list = []
    for pose_file in pose_files:
        pose_list.append(parse_pose(os.path.join(pos_dir, pose_file)))
    clusters = cluster_data_from_pose(pose_list)
    for cluster in clusters:
        pc_list = []
        for idx in cluster:
            pc_list.append()



if __name__ == '__main__':
    main()
