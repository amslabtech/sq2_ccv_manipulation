import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class CollisionDetector(Node):

    def __init__(self):
        super().__init__("collision_detection")
        self.sub_scan1 = self.create_subscription(LaserScan, '/scan1', self.scan_sub, 10)
        self.sub_scan2 = self.create_subscription(LaserScan, '/scan2', self.scan_sub, 10)
        self.sub_scan3 = self.create_subscription(LaserScan, '/scan3', self.scan_sub, 10)

        self.fig = plt.figure(figsize=(15,5))
        self.axes = []
        for i in range(3):
            self.axes.append(self.fig.add_subplot(1, 3, i+1))

    def scan_sub(self, oscan):
        print(oscan.header)
        print(oscan.ranges)
        label = oscan.header.frame_id
        scan_id = int(label[5]) - 1
        self.axes[scan_id].set_title(label)
        self.axes[scan_id].cla()
        self.axes[scan_id].plot(np.array(oscan.ranges))
        plt.pause(0.1)

    def show_graph(self):
        pass

def main(args=None):

    rclpy.init(args=args)
    node = CollisionDetector()

    try:
        rclpy.spin(node)

    finally:
        if node not in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
