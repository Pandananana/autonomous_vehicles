import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32


class ScanSubscribe(Node):
    def __init__(self):
        super().__init__('lidar_converter')
        self.subscription = self.create_subscription(LaserScan,'scan',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(PointCloud, 'cloud', 10)

    def listener_callback(self, msg):
        # Initialize variables
        out_msg = PointCloud()

        for i in range(len(msg.ranges)):
            point = Point32()
            # Skip invalid measurements
            if (msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max):
                continue
            else:
                # Convert from polar to cartesian coordinates
                point.x = msg.ranges[i] * math.cos(msg.angle_min + i * msg.angle_increment)
                point.y = msg.ranges[i] * math.sin(msg.angle_min + i * msg.angle_increment)
                point.z = 0.0

                # Add point and to message
                out_msg.points.append(point)
                
        # Add header to messagepoint = Point32()
        out_msg.header = msg.header

        # Publish message
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    scan_subscriber = ScanSubscribe()

    rclpy.spin(scan_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()