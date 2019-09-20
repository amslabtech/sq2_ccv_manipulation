import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
class CollisionDetector(Node):

    def __init__(self):
        super().__init__("collision_detection")
        self.sub_scan1 = self.create_subscription(String, '/scan1', self.scan_sub, 10)
        self.sub_scan2 = self.create_subscription(String, '/scan2', self.scan_sub, 10)
        self.sub_scan3 = self.create_subscription(String, '/scan3', self.scan_sub, 10)

    def scan_sub(self, oscan):
        print(oscan.data)

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
