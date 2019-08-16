import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist


class Agent(Node):

    def __init__(self):
        super().__init__('sq2_ccv')
        self.pub = self.create_publisher(Twist, '/sq2_ccv/diff_drive_steering_controller/cmd_vel', 10)
        self.reset_sim = self.create_client(Empty, '/reset_simulation')
        self.time_period = 0.1
        self.tmr = self.create_timer(self.time_period, self.step)

    def reset(self):
        while not self.reset_sim.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/reset_simulation service not available, waiting again...')

        reset_future = self.reset_sim.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, reset_future)

    def _send_twist(self, x_linear, z_angular):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = x_linear
        twist.angular.x, twist.angular.y, twist.angular.z = z_angular
        self.pub.publish(twist)

    def step(self):
        self._send_twist(
            (1.0, 2.0, 0.0),
            (0.0, 0.0, 0.0)
            )

def main(args=None):

    rclpy.init(args=args)
    agent = Agent()

    try:
        rclpy.spin(agent)

    except KeyboardInterrupt:
        if agent not in locals():
            agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
