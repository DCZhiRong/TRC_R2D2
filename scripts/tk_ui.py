#!/usr/bin/env python3

import time  # Time library
import os
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from std_msgs.msg import String
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import copy
import tkinter as tk
from PIL import Image, ImageTk

commands = []

goals = []
bringup_dir = get_package_share_directory('r2d2')

class Location():
  def __init__(self, posx, posy, posz):
    self.goal_pose = PoseStamped()
    self.goal_pose.header.frame_id = 'map'
    self.goal_pose.pose.position.x = posx
    self.goal_pose.pose.position.y = posy
    self.goal_pose.pose.position.z = posz
    self.goal_pose.pose.orientation.x = 0.0
    self.goal_pose.pose.orientation.y = 0.0
    self.goal_pose.pose.orientation.z = 0.0
    self.goal_pose.pose.orientation.w = 1.0

locations = {"CnC lab": Location(-14.8, 1.290, 0.0),
            "3d printing lab":Location(-4.35, 0.603, 0.0),
            }

class PageManager(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.title("R2D2 Interface")
        self.geometry("800x480")
        self.container = tk.Frame(self)
        self.container.pack(pady=0, padx=0, expand=True)
        
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

  def dragging(self, event):
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

  def change_page(self, page_name):
        self.controller.after(0, self.controller.show_page, page_name)

  def multi_action(self, location, page):
    end_goal = copy.deepcopy(location.goal_pose)
    goals.append(end_goal)
    self.change_page(page)

    
class Page1(PageFeatures):
  def __init__(self, parent, controller):
    PageFeatures.__init__(self, parent, controller)
    self.label = tk.Label(self, text="Home Page")
    self.label.pack(pady=50, padx=100)
    
    self.text_button_1 = tk.Label(self, text="Looking for somewhere?", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=50, pady=50, bg="lightblue")
    self.text_button_1.bind("<Button-1>", lambda event: self.change_page("Page2"))
    self.text_button_1.pack()

    self.text_button_2 = tk.Label(self, text="Are you lost?", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=50, pady=50, bg="lightgreen")
    self.text_button_2.bind("<Button-1>", lambda event: self.change_page("Page3"))  # Change the command to switchToPage3
    self.text_button_2.pack()

class Page2(PageFeatures):
  def __init__(self, parent, controller):
    PageFeatures.__init__(self, parent, controller)
    # Create widgets for page 2
    self.label = tk.Label(self, text="Mall Map")
    self.label.pack(pady=50)
            # Load the original image
    self.image = Image.open(os.path.join(bringup_dir, 'maps', 'New_ob.pgm'))
    self.image = self.image.resize((200, 200))  # Resize for display
    self.image_photo = ImageTk.PhotoImage(self.image)

    # Create a label with the PhotoImage
    self.image_label = tk.Label(self, image=self.image_photo)
    self.image_label.pack()
    self.location_button1 = tk.Label(self, text="Go to CnC Lab", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=5, pady=5, bg="lightblue")
    self.location_button1.bind("<Button-1>", lambda event: self.multi_action(locations["CnC lab"], "Page1"))
    self.location_button1.pack()
    self.location_button2 = tk.Label(self, text="Go to 3d printing Lab", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=5, pady=5, bg="lightblue")
    self.location_button2.bind("<Button-1>", lambda event: self.multi_action(locations["3d printing lab"], "Page1"))
    self.location_button2.pack()
    self.return_button = tk.Label(self, text="Go Back to Main Page", font=("Helvetica", 16, "bold"), relief="raised", bd=5, padx=5, pady=5, bg="lightblue")
    self.return_button.bind("<Button-1>", lambda event: self.change_page("Page1"))
    self.return_button.pack()

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
    self.text_button.bind("<Button-1>", lambda event: self.change_page("Page1"))
    self.text_button.pack()

class Publisher(Node):

    def __init__(self):
        self.app = PageManager()
        super().__init__('tk_destinations_node')
        self.publisher1 = self.create_publisher(PoseStamped, 'tk_destinations', 10)
        self.subscription = self.create_subscription(String, 'result', self.listener_callback, 10)
        self.publisher2 = self.create_publisher(String, 'nav_commands', 10)
        timer_period = 0.02  # seconds
        self.topic1 = self.create_timer(timer_period, self.timer_callback)
        self.topic2 = self.create_timer(timer_period, self.timer2_callback)
        self.i = 0

    def listener_callback(self, msg):
        words = msg.data.lower()
        if "i am lost" in words:
          self.app.show_page("Page3")
        if "stop" in words or "excuse me" in words or "help" in words:
          commands.append('stop')

    def timer2_callback(self):
        msg = String()
        if commands:
          msg.data = commands.pop()
          self.publisher2.publish(msg)
          self.get_logger().info('Publishing: "%s"' % msg)

    def timer_callback(self):
        self.app.update_idletasks()
        self.app.update()
        msg = PoseStamped()
        if goals:
          msg = goals.pop()
          self.publisher1.publish(msg)
          self.get_logger().info('Publishing: "%s"' % msg)
        self.i +=1


def main(args=None):
  rclpy.init(args=args)
  goal_publisher = Publisher()
  rclpy.spin(goal_publisher)
  minimal_publisher.destroy_node()
  rclpy.shutdown()
  exit(0)
  

if __name__ == '__main__':
  main()