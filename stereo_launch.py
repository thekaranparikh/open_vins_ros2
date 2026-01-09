from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    common_params = {"width": 640, "height": 480, "format": "XRGB8888"}
    
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='left_camera',
            parameters=[common_params, {"camera": "/base/axi/pcie@120000/rp1/i2c@88000/imx219@10"}],
            remappings=[("~/image_raw", "/left/image_raw"), ("~/camera_info", "/left/camera_info")]
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='right_camera',
            parameters=[common_params, {"camera": "/base/axi/pcie@120000/rp1/i2c@80000/imx219@10"}],
            remappings=[("~/image_raw", "/right/image_raw"), ("~/camera_info", "/right/camera_info")]
        )
    ])
