import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleController(Node):
    def __init__(self):
        # create publisher
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # sets up callback
        self.time = 0

    def create_twist(self, linear_x, angular_z):
        # set linear and angular rates
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        # get linear and angular rates based on time
        if self.time < 10:
            msg = self.create_twist(1.0, 0.0) # straight
        elif 10 <= self.time < 12:
            msg = self.create_twist(0.0, 1.6) # turn left
        elif 12 <= self.time < 17:
            msg = self.create_twist(1.0, 0.0) # straight
        elif 17 <= self.time < 19:
            msg = self.create_twist(0.0, 1.6) # turn left
        elif 19 <= self.time < 29:
            msg = self.create_twist(1.0, 0.0) # straight
        elif 29 <= self.time < 31:
            msg = self.create_twist(0.0, 1.6) # turn left
        elif 31 <= self.time < 36:
            msg = self.create_twist(1.0, 0.0) # straight
        else:
            msg = self.create_twist(0.0, 0.0) # stop
        return msg

    def timer_callback(self):
        # get and publish rates, increment time
        msg = self.get_twist_msg()
        self.publisher.publish(msg)
        self.time += 1
        self.get_logger().info(f"time: {self.time}")


def main(args=None):
    # run program
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller) # start controller, blocking function
    turtle_controller.destroy_node() # destructor
    rclpy.shutdown()


if __name__ == '__main__':
    main()
