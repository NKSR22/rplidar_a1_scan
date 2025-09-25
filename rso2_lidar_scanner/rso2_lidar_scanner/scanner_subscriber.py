import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScannerSubscriber(Node):
    """
    A simple ROS 2 node that subscribes to the /scan topic and prints a message upon receiving data.
    """
    def __init__(self):
        super().__init__('scanner_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.get_logger().info('Scanner subscriber node has been started. Waiting for /scan topic...')

    def listener_callback(self, msg):
        """
        Callback function for the /scan topic subscriber.
        """
        num_points = len(msg.ranges)
        self.get_logger().info(f'Received a LaserScan message with {num_points} points.')

def main(args=None):
    rclpy.init(args=args)
    scanner_subscriber = ScannerSubscriber()
    try:
        rclpy.spin(scanner_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        scanner_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()