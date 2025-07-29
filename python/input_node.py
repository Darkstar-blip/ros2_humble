import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(String, 'task_conditions', 10)
        self.timer = self.create_timer(5.0, self.publish_input)

    def publish_input(self):
        msg = String()
        msg.data = 'morning,delivery,medium'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published conditions: {msg.data}')

def main():
    rclpy.init()
    node = InputNode()
    rclpy.spin(node)
    rclpy.shutdown()
