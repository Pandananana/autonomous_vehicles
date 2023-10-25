import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class MapJoyToAck(Node):

    def __init__(self):
        super().__init__('map_joy_to_ack')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

    def listener_callback(self, msg):
        # Throttle is axis 5 (-1 pressed, 1 not pressed)
        # X is button 2 (1 pressed, 0 not pressed)
        # Steering is axis 0 (1 left, -1 right)

        ack = AckermannDriveStamped()

        if msg.buttons[2]:
            direction = -1
        else:
            direction = 1
        

        # Map steering angle between -45 degrees and +45 degrees (in radians)
        ack.drive.steering_angle = msg.axes[0] * math.pi/4.0
        ack.drive.speed = direction*(1-msg.axes[5])*0.5

        # Log output
        self.publisher.publish(ack)


def main(args=None):
    rclpy.init(args=args)

    point_subscriber = MapJoyToAck()

    rclpy.spin(point_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()