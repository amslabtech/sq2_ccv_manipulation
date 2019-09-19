import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pynput.keyboard import Key, Listener
import time
 
class CollisionDetector(Node):

    def __init__(self):
        pass

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
