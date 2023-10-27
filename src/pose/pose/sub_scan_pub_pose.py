import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class GPS_ODO_Subscribe(Node):
    def __init__(self):
        super().__init__('pose_converter')
        self.gps_subscription = self.create_subscription(NavSatFix,'gps',self.gps_callback,10)
        self.odo_subscription = self.create_subscription(Odometry, "odometry", self.odo_callback, 10)

        self.gps_subscription
        self.odo_subscription

        self.publisher = self.create_publisher(PoseStamped, 'vehicle_pose', 10)

    def gps_callback(self, msg):
        with open("gps.txt", "a") as myfile:
            myfile.write(str(msg.latitude) + " " + str(msg.longitude) + "\n")




    def odo_callback(self, msg):
        pass





def main(args=None):
    rclpy.init(args=args)

    gps_odo_subscriber = GPS_ODO_Subscribe()

    rclpy.spin(gps_odo_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_odo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()