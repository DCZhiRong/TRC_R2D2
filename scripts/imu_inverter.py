#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
from copy import deepcopy

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    #minimal_publisher = MinimalPublisher()

    #rclpy.spin(minimal_publisher)
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('imu_inverter')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.subscription = self.create_subscription(Imu,'/camera/imu',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        imu = deepcopy(msg)
        temp = imu.linear_acceleration.x
        imu.linear_acceleration.x = imu.linear_acceleration.z
        imu.linear_acceleration.z = -imu.linear_acceleration.y
        imu.linear_acceleration.y = -temp
        temp = imu.angular_velocity.x
        imu.angular_velocity.x = imu.angular_velocity.z
        imu.angular_velocity.z = -imu.angular_velocity.y
        imu.angular_velocity.y = -temp
        #imu.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(imu)



if __name__ == '__main__':
    main()