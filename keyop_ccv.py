import rclpy
from rclpy.node import Node
from geometry_msgs import Twist
from std_msgs.msg import String
from pynput.keyboard import Key, Listener
import time
 
class Keyboard(Node):

    def __init__(self):
        super().__init__('text')
        self.pub = self.create_publisher(String, '/demo/keyboard')
        self.pub_twist = self.create_publisher(Twist, '/sq2_ccv/diff_drive_steering_controller/cmd_vel', 10)
        self.actions = dict()

        self.__init_action()
        self.check()

    def __init_action(self):
        self.move_keys = ["stop", "forward", "backward", "left", "right"]
        self.op_keys = "swxad"
        xs = [0.0, 1.0, -1.0, 0.0, 0.0]
        zs = [0.0, 0.0, 0.0, 1.0, -1.0]

        for i in range(len(self.move_keys)):
            action = dict()
            action = {
                    "twist_linear" : (xs[i], 0.0, 0.0),
                    "twist_angular" : (0.0, 0.0, zs[i])
                }

            self.actions[self.move_keys[i]] = action

    def _send_twist_by_key(self, key_str):
        move_key = self.move_keys[self.op_keys.find(key_str)]
        linear = self.actions[move_key]["twist_linear"]
        angular = self.actions[move_key]["twist_angular"]
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = linear
        twist.angular.x, twist.angular.y, twist.angular.z = angular
        self.pub_twist.publish(twist)

    def on_press(self, key):
        pub_str = String()
        key_str = str(key)
        pub_str.data = key_str
        self.pub.publish(pub_str)
        if key_str in self.op_keys:
            self._send_twist_by_key(key_str)

    def on_release(self, key):
        pub_str = String()
        pub_str.data = 'None'
        self.pub.publish(pub_str)
        self._send_twist_by_key('s')
        if key == Key.esc:
            # Stop listener
            return False

    def check(self):
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()


def main(args=None):

    rclpy.init(args=args)
    text = Keyboard()
    try:
        rclpy.spin(text)
    finally:
        if text not in locals():
            text.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
