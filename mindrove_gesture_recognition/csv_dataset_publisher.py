import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import pandas as pd
import numpy as np

class CsvDatasetPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'topic', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 12000
        self.dataset = pd.read_csv("data/EMG/subject_3_EMG.csv")
        self.array = self.dataset.iloc[self.i,2:10].values
        self.gesture = self.dataset.iloc[self.i,1]


    def timer_callback(self):
        msg = Float64MultiArray()
        #print(self.array)
        msg.data = self.array.tolist()
        self.publisher_.publish(msg)
        #self.get_logger().info(f"Publishing: {msg.data}")
        self.get_logger().info(f"Gesture: {self.gesture}")
        self.i += 1
        self.array = self.dataset.iloc[self.i,2:10].values
        self.gesture = self.dataset.iloc[self.i,1]
        
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CsvDatasetPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()