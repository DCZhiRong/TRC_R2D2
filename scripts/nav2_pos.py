#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com and then Dan haha
 
import time  # Time library
import os
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from std_msgs.msg import String
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
from threading import *
import queue
goals = []
goals_name = []
goals_name_reached = []
commands = []
bringup_dir = get_package_share_directory('r2d2')
'''
Navigates a robot from an initial pose to a goal pose.
'''


class Subscriber(Node):

    def __init__(self):
        super().__init__('_node')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 1.32
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.position.z = 0.00
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0
        #self.start_app = Thread(target=ros_nav, args=(self.navigator, goals,commands,))
        #start_app.daemon = True
        #self.start_app.start()
        timer_period = 1
        self.i = 0
        self.a = 0
        self.pause_at_goal = False
        self.timer_start = False
        self.starttime = 0
        self.curtime = 0
        self.destination_name = ""
        self.topic2 = self.create_timer(timer_period, self.ros_nav)
        self.subscription1 = self.create_subscription(String, 'nav_commands', self.listener_callback1, 10)
        self.subscription2 = self.create_subscription(PoseStamped, 'tk_destinations', self.listener_callback2, 10)
        self.subscription3 = self.create_subscription(PoseStamped, 'tk_destinations_names', self.listener_callback3, 10)
        self.publisher1 = self.create_publisher(String, 'tk_destinations_reached', 10)
        #self.topic3 = self.create_timer(timer_period, self.publish_reached)
        

    def listener_callback1(self, msg):
        words = msg.data.lower()
        commands.append(words)
        print(words)
    
    def listener_callback2(self, msg):
        goal = msg
        goals.append(goal)

    def listener_callback3(self, msg):
        goal_name = msg.data
        goals_name.append(goal_name)

    def publish_reached(self):
      msg = String()
      if goals_name_reached:
        msg.data = goals_name_reached.pop()
        self.publisher1.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def ros_nav(self):
      delay_mins = 5
      #navigator = BasicNavigator()
      #navigator.waitUntilNav2Active()
      if self.curtime - self.starttime < delay_mins*60:
        if commands:
          item = commands.pop()
          if "stop" in item:
            self.navigator.cancelTask()
            self.starttime = time.time()
            self.timer_start = True
        if goals:
          item = goals.pop()
          self.goal_pose = item
          self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
          self.navigator.cancelTask()
          self.pause_at_goal = True
          self.navigator.goToPose(self.goal_pose)
          self.curtime = 0
          self.start_time = 0
          self.timer_start = False
        if goals_name:
          self.destination_name = goals_name.pop()
      else:
        self.curtime = 0
        self.start_time = 0
        self.timer_start = False
      if self.timer_start:
        self.curtime = time.time()
        return
      if self.navigator.isTaskComplete():
        print('Goal succeeded!')
        if self.pause_at_goal:
          goals_name_reached.append(self.destination_name)
          self.destination_name = ""
          print(goals_name_reached[0])
          print("pausing")
          msg = String()
          if goals_name_reached:
            msg.data = goals_name_reached.pop()
            self.publisher1.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
          self.starttime = time.time()
          self.timer_start = True
          self.pause_at_goal = False
          self.a ^= 1
        if self.a == 0:
          self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
          self.goal_pose.pose.position.x = 66.6
          self.goal_pose.pose.position.y = 6.65
          self.goal_pose.pose.position.z = 0.0
          self.goal_pose.pose.orientation.x = 0.0
          self.goal_pose.pose.orientation.y = 0.0
          self.goal_pose.pose.orientation.z = 0.0
          self.goal_pose.pose.orientation.w = 1.0
        elif self.a == 1:
          self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
          self.goal_pose.pose.position.x = 1.320
          self.goal_pose.pose.position.y = 0.0
          self.goal_pose.pose.position.z = 0.00
          self.goal_pose.pose.orientation.x = 0.0
          self.goal_pose.pose.orientation.y = 0.0
          self.goal_pose.pose.orientation.z = 0.0
          self.goal_pose.pose.orientation.w = 1.0
        self.a ^= 1
        self.navigator.goToPose(self.goal_pose)
      #result = navigator.getResult()
      #if result == TaskResult.CANCELED:
      #    print('Goal was canceled!')
      #elif result == TaskResult.FAILED:
      #    print('Goal failed!')
      #else:
      #    print('Goal has an invalid return status!')

    # Shut down the ROS 2 Navigation Stack
      #self.navigator.lifecycleShutdown()
        


      

def main(args=None):
  rclpy.init(args=args)
  nav_subscriber = Subscriber()
  rclpy.spin(nav_subscriber)
  nav_subscriber.destroy_node()
  rclpy.shutdown()
  exit(0)
  

if __name__ == '__main__':
  main()