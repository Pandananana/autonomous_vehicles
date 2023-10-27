import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import utm

class GPS_ODO_Subscribe(Node):
    def __init__(self):
        super().__init__('pose_converter')
        self.gps_subscription = self.create_subscription(NavSatFix,'gps',self.gps_callback,10)
        self.odo_subscription = self.create_subscription(Odometry, "odometry", self.odo_callback, 10)

        self.gps_subscription
        self.odo_subscription

        self.publisher = self.create_publisher(PoseStamped, 'vehicle_pose', 10)

        self.pose_msg = PoseStamped()

    def gps_callback(self, msg):
        with open("gps.txt", "a") as gps_file:
            gps_file.write(str(msg.latitude) + " " + str(msg.longitude) + "\n")

        x, y, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
        
        self.pose_msg.pose.position.x = x
        self.pose_msg.pose.position.y = y

        self.publisher.publish(self.pose_msg)

    def odo_callback(self, msg):
        theta = 2*math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        with open("odo.txt", "a") as odo_file:
            odo_file.write(str(msg.pose.pose.orientation.x) + " | " + str(msg.pose.pose.orientation.y) + " | " + str(msg.pose.pose.orientation.z) + " | " + str(msg.pose.pose.orientation.w) + "\n")
            odo_file.write("Angle: " + str(theta) + "\n")





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