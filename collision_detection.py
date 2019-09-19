import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
class CollisionDetector(Node):

    def __init__(self):
        self.sub1 = self.create_subscription(String, '/scan1', 10)
        self.sub2 = self.create_subscription(String, '/scan2', 10)
        self.sub3 = self.create_subscription(String, '/scan3', 10)

def main(args=None):
    rclpy.init(args=args)
    detector = CollisionDetector()
    try:
        rclpy.spin(detector)
    finally:
        if detector not in locals():
            detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
