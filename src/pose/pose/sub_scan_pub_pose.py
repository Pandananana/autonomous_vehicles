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

        # Reset the files
        open("gps_data.txt", "w").close()
        open("angle_data.txt", "w").close()

        self.gps_subscription = self.create_subscription(NavSatFix,'gps',self.gps_callback,10)
        self.odo_subscription = self.create_subscription(Odometry, "odometry", self.odo_callback, 10)

        self.gps_subscription
        self.odo_subscription

        self.publisher = self.create_publisher(PoseStamped, 'vehicle_pose', 10)

        self.x = 0.0
        self.y = 0.0
        self.first_val = True
        self.offset = [0.0, 0.0]

    def gps_callback(self, msg):
        with open("gps_data.txt", "a") as gps_file:
            gps_file.write(str(msg.latitude) + " " + str(msg.longitude) + "\n")

        self.x, self.y, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)

    def odo_callback(self, msg):
        if self.first_val and self.x != 0.0 and self.y != 0.0:
            self.offset[0] = self.x
            self.offset[1] = self.y
            self.first_val = False

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "pose"
        pose_msg.pose.position.x = self.x - self.offset[0]
        pose_msg.pose.position.y = self.y - self.offset[1]

        theta = 2*math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        with open("angle_data.txt", "a") as odo_file:
            odo_file.write(str(msg.pose.pose.orientation.x) + " | " + str(msg.pose.pose.orientation.y) + " | " + str(msg.pose.pose.orientation.z) + " | " + str(msg.pose.pose.orientation.w) + "\n")
            odo_file.write("Angle: " + str(theta) + "\n")

        pose_msg.pose.orientation.z = msg.pose.pose.orientation.z
        pose_msg.pose.orientation.w = msg.pose.pose.orientation.w
        
        self.publisher.publish(pose_msg)


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