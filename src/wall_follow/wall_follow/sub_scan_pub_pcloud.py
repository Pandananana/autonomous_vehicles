import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped


class ScanSubscribe(Node):
    def __init__(self):
        super().__init__('wall_follow_converter')
        self.lidar_subscription = self.create_subscription(LaserScan,'scan',self.lidar_callback,10)
        self.joy_subscription = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.lidar_subscription  # prevent unused variable warning
        self.joy_subscription

        self.publisher = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

        # initialize some globals, don't need to edit these
        self.go = False
        self.buttonOn = False
        self.dwall = 0 
        self.error_sum = 0
        self.last_error = 0

        # these are the parameters we can play around with
        self.kp = 0.5
        self.ki = 0
        self.kd = 0.3
        self.timer_period = 0.05    # seconds
        self.vehicle_speed = 0.7
        self.desired_distance = 1   # meters

        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # self.timer # prevent unused variable warning

    def lidar_callback(self, msg):
        # Initialize variables
        # Keep value of d consistently updated
        
        # NEW: take average over a 45 degree range (starts 45-90, ends 90-135)
        points_to_test = int((math.pi/4)/msg.angle_increment)


        front_angle = 45
        right_angle = 90

        front_index = int((front_angle/180) * math.pi / msg.angle_increment) 
        right_index = int((right_angle/180) * math.pi / msg.angle_increment)

        distance_sum = 0
        points_skipped = 0

        for i in range(0, points_to_test - 1): 

            if (msg.ranges[front_index] < msg.range_min or msg.ranges[front_index] > msg.range_max or msg.ranges[right_index] < msg.range_min or msg.ranges[right_index] > msg.range_max):
                points_skipped += 1
                continue
            
            d2 = msg.ranges[front_index]
            d1 = msg.ranges[right_index]

            x2 = d2 * math.cos(front_angle/180 * math.pi)
            y2 = d2 * math.sin(front_angle/180 * math.pi)

            x1 = d1 * math.cos(right_angle/180 * math.pi)
            y1 = d1 * math.sin(right_angle/180 * math.pi)

            # Distance between the two points
            d3 = math.sqrt((x1-x2)**2 + (y1-y2)**2)

            distance_sum += d1 * (x2 - x1) / d3
            front_index += 1
            right_index += 1

        self.dwall = distance_sum / (points_to_test - points_skipped)

        self.get_logger().info('Dwall: "%f"' % self.dwall)


        # PID starts here
        ack = AckermannDriveStamped()
        ack.drive.speed = self.vehicle_speed

        error = self.desired_distance - self.dwall   # negative means turn right, positive means turn left
        self.error_sum += error

        control_effort = self.kp*error + self.ki*self.error_sum*msg.scan_time + self.kd*(error-self.last_error)/msg.scan_time
        # timer period included in calcs so we can vary it without butchering Ks

        self.get_logger().info(f'Error: {error} m    Control effort: {control_effort} rad')

        self.last_error = error

        ack.drive.steering_angle = (abs(error)/error) * min(abs(control_effort), math.pi/4)
        # make sure the control effort does not exceed pi/4, and point it in the proper direction 

        ack.drive.speed = self.go * self.vehicle_speed

        self.publisher.publish(ack)


    def joy_callback(self, msg):

        # SPECIFY A BUTTON HERE
        if bool(msg.buttons[2]) and not self.buttonOn:
            self.go = not self.go
        self.buttonOn = bool(msg.buttons[2])





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