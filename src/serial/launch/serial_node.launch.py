from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    serial_node = Node(package="serial", executable="serial_node", name="serial_twist")
    return LaunchDescription([serial_node])