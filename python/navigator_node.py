import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.navigator = BasicNavigator()
        self.subscriber = self.create_subscription(String, 'target_room', self.navigate, 10)
        self.room_goals = {
            'kitchen': [1.5, 0.5, 0.0],
            'bedroom': [-3.0, -2.0, 0.0],
            'living_room': [0.0, -3.5, 0.0]
        }

    def navigate(self, msg):
        room = msg.data
        if room not in self.room_goals:
            self.get_logger().error(f"Unknown room: {room}")
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        x, y, _ = self.room_goals[room]
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.navigator.goToPose(goal)
        self.get_logger().info(f"Navigating to {room}...")

def main():
    rclpy.init()
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()
