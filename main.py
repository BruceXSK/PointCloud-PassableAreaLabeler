import rospy
import os
import pyqtgraph as pg
from dataset import Dataset
from labeler import MainWindow, Labeler, Params


def parse_data_to_label_file(path: str):
    indices = []
    f = open(path, 'r')
    lines = f.readlines()
    for line in lines:
        indices.append(int(line.split(' ')[0]))
    f.close()
    return indices[::-1]


def main():
    if Params.ros_tf_pub:
        rospy.init_node('labeler')

    data_root_path = '.'
    data_series_name = 'IB-NorthEast'
    dataset = Dataset(
        map_path=os.path.join(data_root_path, 'data', data_series_name, 'map_velodyne.pcd'),
        pcd_dir=os.path.join(data_root_path, 'data', data_series_name, 'pcd'),
        ndt_pose_dir=os.path.join(data_root_path, 'data', data_series_name, 'ndt_pose'),
        label_dir=os.path.join(data_root_path, 'data', data_series_name, 'label'),
        pre_load=False,
        selected_label_indices=parse_data_to_label_file(os.path.join('data', data_series_name, 'data_to_label.txt'))
    )
    app = pg.mkQApp('Labeler')

    labeler = Labeler(dataset, start_idx=20)
    window = MainWindow(labeler)
    window.show()

    pg.exec()


if __name__ == '__main__':
    main()
