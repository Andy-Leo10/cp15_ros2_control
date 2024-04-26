import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

class ElevatorCommandPublisher(Node):
    def __init__(self):
        super().__init__('elevator_command_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/elevator_controller/commands', 10)
        self.timer_period = 0.1  # seconds
        self.i = 0
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        if self.i % 2 == 0:
            msg.data = [0.6]
            self.get_logger().info('Moving Up')
        else:
            msg.data = [0.0]
            self.get_logger().info('Moving Down')
        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    elevator_command_publisher = ElevatorCommandPublisher()
    rclpy.spin(elevator_command_publisher)
    elevator_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()