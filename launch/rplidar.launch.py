import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                #'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'serial_baudrate': 115200,
                #'serial_baudrate': 256000,
                'angle_compensate':True,
                'inverted':False,
                'scan_mode': 'Standard'
                }])
    ])
