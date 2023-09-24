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
# Modified by AutomaticAddison.com
 
import time  # Time library
import os
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module

import tkinter as tk
import speech_recognition as sr
import threading
from PIL import Image, ImageTk

bringup_dir = get_package_share_directory('r2d2')
'''
Navigates a robot from an initial pose to a goal pose.
'''

class Location():
  def __init__(self):
    self.name = ""
    self.goal_pose = PoseStamped()
    self.goal_pose.header.frame_id = 'map'
    self.goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    self.goal_pose.pose.position.x = 0.0
    self.goal_pose.pose.position.y = 0.0
    self.goal_pose.pose.position.z = 0.0
    self.goal_pose.pose.orientation.x = 0.0
    self.goal_pose.pose.orientation.y = 0.0
    self.goal_pose.pose.orientation.z = 0.0
    self.goal_pose.pose.orientation.w = 1.0

class PageManager(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.title("R2D2 Interface")
        self.geometry("1024x600")
        self.keyword_detected = threading.Event()  # Shared event for communication
        self.start_voice_assistant()  # Start the voice assistant in the background
        # Create a container frame to hold the pages
        self.container = tk.Frame(self)
        self.container.pack(side="top", fill="both", expand=True)
        
        # Dictionary to store all pages
        self.pages = {}
        
        # Initialize the pages
        for PageClass in (Page1, Page2, Page3):
            page_name = PageClass.__name__
            page = PageClass(self.container, self)
            self.pages[page_name] = page
            page.grid(row=0, column=0, sticky="nsew")
        
        self.show_page("Page1")
    
    def show_page(self, page_name):
        # Show the requested page
        page = self.pages.get(page_name)
        if page:
            page.tkraise()
    
    def start_voice_assistant(self):
      assistant = VoiceAssistant(self)
      background_listener = threading.Thread(target=assistant.listen_for_keyword)
      background_listener.start()

class PageFeatures(tk.Frame):
  def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        self.tk = controller.tk
        
  def show(self):
      self.tkraise()
  
  def start_drag(self, event):
        self.last_x = event.x
        self.last_y = event.y

  def dragging(self, event, image):
      dx = event.x - self.last_x
      dy = event.y - self.last_y

      self.zoom_factor += dy * 0.01  # Adjust zoom factor based on vertical movement

      if self.zoom_factor < 0.5:
          self.zoom_factor = 0.5
      elif self.zoom_factor > 2.0:
          self.zoom_factor = 2.0

      self.image = self.image.resize((int(200 * self.zoom_factor), int(200 * self.zoom_factor)))
      self.image_photo = ImageTk.PhotoImage(self.image)
      self.image_label.config(image=self.image_photo)

      self.last_x = event.x
      self.last_y = event.y

  def stop_drag(self, event):
      self.last_x = 0
      self.last_y = 0
    
class Page1(PageFeatures):
  def __init__(self, parent, controller):
    PageFeatures.__init__(self, parent, controller)
    self.label = tk.Label(self, text="Home Page")
    self.label.pack(pady=50, padx=100)
    
    self.text_button_1 = tk.Label(self, text="Looking for somewhere?", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=50, pady=50, bg="lightblue")
    self.text_button_1.bind("<Button-1>", lambda event: controller.show_page("Page2"))
    self.text_button_1.pack()

    self.text_button_2 = tk.Label(self, text="Are you lost?", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=50, pady=50, bg="lightgreen")
    self.text_button_2.bind("<Button-1>", lambda event: controller.show_page("Page3"))  # Change the command to switchToPage3
    self.text_button_2.pack()

class Page2(PageFeatures):
  def __init__(self, parent, controller):
    PageFeatures.__init__(self, parent, controller)
    # Create widgets for page 2
    self.label = tk.Label(self, text="Mall Map")
    self.label.pack(pady=50)

            # Load the original image
    self.image = Image.open(os.path.join(bringup_dir, 'maps', 'New_ob.pgm'))
    self.image = self.image.resize((400, 400))  # Resize for display
    self.image_photo = ImageTk.PhotoImage(self.image)

    # Create a label with the PhotoImage
    self.image_label = tk.Label(self, image=self.image_photo)
    self.image_label.pack()

    self.text_button = tk.Label(self, text="Go Back to Main Page", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=50, pady=50, bg="lightblue")
    self.text_button.bind("<Button-1>", lambda event: controller.show_page("Page1"))
    self.text_button.pack()

    # Bind mouse events
    self.image_label.bind("<Button-1>", self.start_drag)
    self.image_label.bind("<B1-Motion>", self.dragging)
    self.image_label.bind("<ButtonRelease-1>", self.stop_drag)

    self.last_x = 0
    self.last_y = 0
    self.zoom_factor = 1.0  # Initial zoom factor

class Page3(PageFeatures):
  def __init__(self, parent, controller):
    PageFeatures.__init__(self, parent, controller)
    # Create widgets for page 3
    self.label = tk.Label(self, text="Notifying mall staff...")
    self.label.pack(pady=50)

    self.text_button = tk.Label(self, text="Go Back to Main Page", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=50, pady=50, bg="lightgreen")
    self.text_button.bind("<Button-1>", lambda event: controller.show_page("Page1"))
    self.text_button.pack()



class VoiceAssistant:
    def __init__(self, interface_instance):
        self.interface = interface_instance
        self.recognizer = sr.Recognizer()

    def listen_for_keyword(self):
        while True:
            with sr.Microphone() as source:
                print("Listening for keyword...")
                audio = self.recognizer.listen(source)

                try:
                    detected_text = self.recognizer.recognize_google(audio).lower()
                    print(detected_text)
                    if "i am lost" in detected_text:
                        self.interface.show_page("Page3")
                        print("Notifying mall staff")
                    # else: print(detected_text)
                except sr.UnknownValueError:
                    pass
                except sr.RequestError as e:
                    print("Could not request results; {0}".format(e))

    def voice_input(self):
        print("Help me triggered.")
        self.interface.show_page("Page3")

def main():
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
 
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  #Set the robot's initial pose if necessary
  #initial_pose = PoseStamped()
  #initial_pose.header.frame_id = 'map'
  #initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  #initial_pose.pose.position.x = 0.0
  #initial_pose.pose.position.y = 0.0
  #initial_pose.pose.position.z = 0.0
  #initial_pose.pose.orientation.x = 0.0
  #initial_pose.pose.orientation.y = 0.0
  #initial_pose.pose.orientation.z = 0.0
  #initial_pose.pose.orientation.w = 1.0
  #navigator.setInitialPose(initial_pose)
 
  # Activate navigation, if not autostarted. This should be called after setInitialPose()
  # or this will initialize at the origin of the map and update the costmap with bogus readings.
  # If autostart, you should `waitUntilNav2Active()` instead.
  # navigator.lifecycleStartup()
 
  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()
 
  # If desired, you can change or load the map as well
  # navigator.changeMap('/path/to/map.yaml')
 
  # You may use the navigator to clear or obtain costmaps
  # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  # global_costmap = navigator.getGlobalCostmap()
  # local_costmap = navigator.getLocalCostmap()
 
  # Set the robot's goal pose
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 10.0
  goal_pose.pose.position.y = 2.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
 
  # sanity check a valid path exists
  # path = navigator.getPath(initial_pose, goal_pose)
  start_app = threading.Thread(target=run_app)
  start_app.start()
  
  a = 0
  while True:
    # Go to the goal pose
    navigator.goToPose(goal_pose)
    i = 0
  
    # Keep doing stuff as long as the robot is moving towards the goal
    while not navigator.isTaskComplete():
      ################################################
      #
      # Implement some code here for your application!
      #
      ################################################
  
      # Do something with the feedback
      i = i + 1
      feedback = navigator.getFeedback()
      if feedback and i % 5 == 0:
        print('Distance remaining: ' + '{:.2f}'.format(
              feedback.distance_remaining) + ' meters.')
  
        # Some navigation timeout to demo cancellation
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
          navigator.cancelNav()
  
        # Some navigation request change to demo preemption
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
          goal_pose.pose.position.x = -3.0
          navigator.goToPose(goal_pose)
  
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        if a == 0:
          goal_pose.header.stamp = navigator.get_clock().now().to_msg()
          goal_pose.pose.position.x = 0.0
          goal_pose.pose.position.y = -7.0
          goal_pose.pose.position.z = 0.0
          goal_pose.pose.orientation.x = 0.0
          goal_pose.pose.orientation.y = 0.0
          goal_pose.pose.orientation.z = 0.0
          goal_pose.pose.orientation.w = 1.0
          a ^= 1
        elif a == 1:
          goal_pose.header.stamp = navigator.get_clock().now().to_msg()
          goal_pose.pose.position.x = 10.0
          goal_pose.pose.position.y = 3.0
          goal_pose.pose.position.z = 0.0
          goal_pose.pose.orientation.x = 0.0
          goal_pose.pose.orientation.y = 0.0
          goal_pose.pose.orientation.z = 0.0
          goal_pose.pose.orientation.w = 1.0
          a ^= 1
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
 
  # Shut down the ROS 2 Navigation Stack
  navigator.lifecycleShutdown()
 
  exit(0)
 
def run_app():
  app = PageManager()
  while True:
    app.update_idletasks()
    app.update()
  

if __name__ == '__main__':
  main()