import os
import struct
from typing import List

import numpy as np
from tqdm import tqdm
from pyntcloud import PyntCloud


def parse_point_cloud(path: str) -> np.ndarray:
    pc = PyntCloud.from_file(path)  # type: PyntCloud
    return np.array(pc.points)[:, :3]


def filter_point_cloud(src_pc: np.ndarray) -> np.ndarray:
    mask = src_pc[:, 2] < 2
    return src_pc[mask]


def parse_pose(path: str) -> np.ndarray:
    f = open(path, 'rb')
    data = f.read()
    p = np.array(struct.unpack('7d', data))  # [position [x y z] orientation [x y z w]]
    f.close()
    return p


class Dataset:

    def __init__(self, map_path: str, pcd_dir: str, ndt_pose_dir: str, label_dir: str, pre_load: bool = True,
                 selected_label_indices: List[int] = None, start: float = 0, end: float = 1):
        if start < 0 or start > 1 or end < 0 or end > 1:
            raise RuntimeError('Parameters start and end should be a ratio between 0 and 1')

        if not os.path.exists(label_dir):
            os.mkdir(label_dir)

        self.pre_load = pre_load
        self.map = parse_point_cloud(map_path)
        self.selected_label_indices = selected_label_indices
        if selected_label_indices is not None:
            if start != 0 or end != 1:
                print('[Warning] start and end will not be used when selected_label_indices is specified')
        self.pc_list = []
        self.pose_list = []
        self.pc_path_list = []
        self.pose_path_list = []
        self.label_path_list = []
        self.file_name_list = []

        pcd_files = sorted(os.listdir(pcd_dir))
        if selected_label_indices is None:
            self.size = len(pcd_files)
            pcd_files = pcd_files[int(start * self.size):int(end * self.size)]
        else:
            self.size = len(selected_label_indices)
        print('Loading data...')
        for pcd_file in tqdm(pcd_files):
            file_name = pcd_file.split('.')[0]
            pose_file = file_name + '.data'

            pcd_file_path = os.path.join(pcd_dir, pcd_file)
            pose_file_path = os.path.join(ndt_pose_dir, pose_file)
            label_file_path = os.path.join(label_dir, pose_file)

            if pre_load:
                self.pc_list.append(filter_point_cloud(parse_point_cloud(pcd_file_path)))
                self.pose_list.append(parse_pose(pose_file_path))
            else:
                self.pc_path_list.append(pcd_file_path)
                self.pose_path_list.append(pose_file_path)
            self.label_path_list.append(label_file_path)
            self.file_name_list.append(file_name)

    def __getitem__(self, idx: int) -> (np.ndarray, np.ndarray):
        if self.selected_label_indices is not None:
            idx = self.selected_label_indices[idx]
        if self.pre_load:
            return (
                self.pc_list[idx],
                self.pose_list[idx],
                self.label_path_list[idx],
                self.file_name_list[idx]
            )
        else:
            return (
                filter_point_cloud(parse_point_cloud(self.pc_path_list[idx])),
                parse_pose(self.pose_path_list[idx]),
                self.label_path_list[idx],
                self.file_name_list[idx]
            )

    def __len__(self):
        return self.size
