import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__("publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.callback)
        self.counter = 0
    
    def callback(self):
        msg = String()
        msg.data = 'Packet: %d' % self.counter
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
