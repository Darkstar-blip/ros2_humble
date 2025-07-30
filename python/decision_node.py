import rclpy
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import String
import joblib
import os
from pathlib import Path

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        home = str(Path.home())
        model_path = os.path.join(home, 'ros2_ws', 'src', 'smart_nav', 'smart_nav', 'room_predictor.joblib')
        self.get_logger().info(f'Loading model from: {model_path}')
        self.model = joblib.load(model_path)

        self.subscriber = self.create_subscription(String, 'task_conditions', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, 'target_room', 10)

    def listener_callback(self, msg):
        time, task, dirt = msg.data.split(',')
        features = pd.DataFrame([[time, task, dirt]], columns=['Time of Day', 'Task Type', 'Dirt Level'])
        room = self.model.predict(features)[0]
        self.publisher.publish(String(data=room))
        self.get_logger().info(f'Predicted room: {room}')

def main():
    rclpy.init()
    node = DecisionNode()
    rclpy.spin(node)
    rclpy.shutdown()
