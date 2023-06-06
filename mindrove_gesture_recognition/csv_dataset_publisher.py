import argparse

import rclpy
from rclpy.node import Node

import pandas as pd
import numpy as np

from roboasset_msgs.msg import MindroveEmg


class CsvDatasetPublisher(Node):

    def __init__(self,
                 dataset_file):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MindroveEmg, 'mindrove/emg', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1
        self.dataset = pd.read_csv(dataset_file)
        self.array = self.dataset.iloc[self.i,2:10].values
        self.gesture = self.dataset.iloc[self.i,1]

        # get names of all the columns
        columns = list(self.dataset.columns)
        # check if the column names have emg in them case insensitive
        self.emg_channels = [col for col in columns if 'emg' in col.lower()]

        self.max = np.ones(len(self.emg_channels)) * -np.inf
        self.min = np.ones(len(self.emg_channels)) * np.inf

    def timer_callback(self):
        msg = MindroveEmg()
        #print(self.array)
        msg.data = self.array.tolist()
        self.max = np.maximum(self.max, self.array)
        self.min = np.minimum(self.min, self.array)
        msg.max = self.max.tolist()
        msg.min = self.min.tolist()

        self.publisher_.publish(msg)
        #self.get_logger().info(f"Publishing: {msg.data}")
        self.get_logger().info(f"Gesture: {self.gesture}")
        self.i += 1
        self.array = self.dataset.iloc[self.i,2:10].values
        self.gesture = self.dataset.iloc[self.i,1]
        
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset_file', type=str,
                        help='Path to the dataset file')
    return parser.parse_known_args()

def main(args=None):
    args, ros2_args = parse_args()
    rclpy.init(args=ros2_args)

    minimal_publisher = CsvDatasetPublisher(
        dataset_file=args.dataset_file
    )

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()