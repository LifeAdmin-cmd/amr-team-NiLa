import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircularMotionPublisher(Node):

    def __init__(self):
        super().__init__('circular_motion_publisher')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()

        msg.linear.x = -0.5      

        msg.angular.z = -0.5    

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CircularMotionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Stop the robot before shutting down
    stop = Twist()
    node.publisher.publish(stop)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
