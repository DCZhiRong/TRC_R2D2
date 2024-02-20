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
import pyttsx3
from PIL import Image, ImageTk

commands = []

goals = []
goal_names = []
bringup_dir = get_package_share_directory('r2d2')

class Location():
  def __init__(self, name, posx, posy, posz):
    self.name = name
    self.goal_pose = PoseStamped()
    self.goal_pose.header.frame_id = 'map'
    self.goal_pose.pose.position.x = posx
    self.goal_pose.pose.position.y = posy
    self.goal_pose.pose.position.z = posz
    self.goal_pose.pose.orientation.x = 0.0
    self.goal_pose.pose.orientation.y = 0.0
    self.goal_pose.pose.orientation.z = 0.0
    self.goal_pose.pose.orientation.w = 1.0

locations = {"CnC lab": Location("CnC lab",29.1, 1.6, 0.0),
            "3d printing lab":Location("3d printing lab",5.25, 1.57, 0.0),
            "Chemistry lab":Location("Chemistry lab",21.0, 3.54, 0.0),
            "Woodworking lab":Location("Woodworking lab",-4.35, 0.603, 0.0),
            "Vortex lab":Location("Vortex lab",16.3, 0.43, 0.0),
            "Mechanical lab":Location("Mechanical lab",-4.35, 0.603, 0.0),
            "Workshop":Location("Workshop",1.63, -1.44, 0.0),
            "Power and machine lab":Location("Power and machine lab",38.3, 3.0, 0.0),}

class PageManager(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.title("R2D2 Interface")
        self.geometry("800x480")
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # self container changed to main_frame grid
        self.main_frame = tk.Frame(self)
        self.main_frame.grid(row=0, column=0, sticky="nsew")
        self.main_frame.grid_rowconfigure(0, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=1)
        
        # Dictionary to store all pages
        self.pages = {}
        
        # Initialize the pages
        for PageClass in (Page1, Page2, Page3):
            page_name = PageClass.__name__
            page = PageClass(self.main_frame, self)
            self.pages[page_name] = page
            page.grid(row=0, column=0, sticky="nsew")
            page.grid_rowconfigure(0, weight=1)
            page.grid_columnconfigure(0, weight=1)
        
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
        self.controller.after(200, self.controller.show_page, page_name)

  def multi_action(self, location, page):
    end_goal = copy.deepcopy(location)
    goals.append(end_goal.goal_pose)
    goal_names.append(end_goal.name)
    self.change_page(page)

class button_base(tk.Label):
    def __init__(self, master=None, **kwargs):
        tk.Label.__init__(self, master, font=("Helvetica World", 14, "bold"),
                           relief="ridge", bd=5, padx=20, pady=15, bg="#123F71", 
                           fg="#FFFFFF", width=15, **kwargs)
    
class Page1(PageFeatures):
  def __init__(self, parent, controller):
    PageFeatures.__init__(self, parent, controller)

    # -- SETTING BACKGROUND IMAGE
    self.bg_image = Image.open(os.path.join(bringup_dir, 'maps', 'New_ob.pgm'))  # PATH TO IMAGE
    self.bg_image = self.bg_image.convert("RGB")
    self.bg_image = self.bg_image.resize((800, 480), Image.LANCZOS)
    self.bg_photo = ImageTk.PhotoImage(self.bg_image)

    bg_label = tk.Label(self, image=self.bg_photo)
    bg_label.img = self.bg_photo
    bg_label.place(x=0, y=0, relheight=1, relwidth=1)

    # -- CUTTING OUT THE HEADING
    # self.label = tk.Label(self, text="Home Page", font=("Helvetical",16,"bold"))
    # self.label.pack(pady=(60,30), padx=10) 
    # manually vertically centered bc its a pain
    
    self.text_button_1 = button_base(self, text="TAKE ME TO...")
    self.text_button_1.bind("<Button-1>", lambda event: self.change_page("Page2"))
    self.text_button_1.pack(pady=(150,30), padx=10)

    self.text_button_2 = button_base(self, text="I'M LOST!")
    self.text_button_2.bind("<Button-1>", lambda event: self.change_page("Page3"))
    self.text_button_2.pack(pady=5, padx=10)


class Page2(PageFeatures):
    def __init__(self, parent, controller):
        PageFeatures.__init__(self, parent, controller)
        # -- SET BACKGROUND IMAGE
        self.bg_image = Image.open(os.path.join(bringup_dir, 'maps', 'New_ob.pgm'))  # PATH TO IMAGE
        self.bg_image = self.bg_image.convert("RGB")
        self.bg_image = self.bg_image.resize((800, 480), Image.LANCZOS)
        self.bg_photo = ImageTk.PhotoImage(self.bg_image)

        bg_label = tk.Label(self, image=self.bg_photo)
        bg_label.img = self.bg_photo
        bg_label.place(x=0, y=0, relheight=1, relwidth=1)

    # -- CREATE A SCROLLABLE CANVAS --
        canvas = tk.Canvas(self)
        canvas.grid(row=0, column=0, sticky="nsew")

        # # Configure column and row weights -- SUPER IMPORTANT!!
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        canvas.grid_columnconfigure(0, weight=1)
        canvas.grid_rowconfigure(0, weight=1)

        # Create a frame inside the canvas to hold the widgets
        # manually mathematically fitted into center T-T
        content_frame = tk.Frame(canvas)
        x0 = canvas.winfo_screenwidth()/2
        y0 = canvas.winfo_screenheight()/2
        canvas.create_window((x0,y0), window=content_frame, anchor="nw", tags="content_frame")

        # # Configure column and row weights
        content_frame.grid_columnconfigure(0, weight=1)
        content_frame.grid_rowconfigure(0, weight=1)

        # # Create a vertical scrollbar
        scrollbar = tk.Scrollbar(self, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")

        # # Configure the canvas to use the scrollbar
        canvas.configure(yscrollcommand=scrollbar.set)

        # # Bind the canvas to update the scroll region when the size changes
        canvas.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))



        # Create widgets for page 2
        self.label = tk.Label(content_frame, text="MALL MAP",font=("Helvetical",16,"bold"),pady=15)
        self.label.grid(row=0, column=0, columnspan=3, pady=20)
                # Load the original image
        self.image = Image.open(os.path.join(bringup_dir, 'maps', 'New_ob.pgm'))
        self.image = self.image.resize((200, 200))  # Resize for display
        self.image_photo = ImageTk.PhotoImage(self.image)

        # Create a label with the PhotoImage
        self.image_label = tk.Label(content_frame, image=self.image_photo)
        self.image_label.grid(row=1, column=0, columnspan=3)

        # -- CREATE A GRID FOR ORGANISATION --
        self.button_widget(content_frame)

        # Bind mouse events
        self.image_label.bind("<Button-1>", self.start_drag)
        self.image_label.bind("<B1-Motion>", self.dragging)
        self.image_label.bind("<ButtonRelease-1>", self.stop_drag)

        self.last_x = 0
        self.last_y = 0
        self.zoom_factor = 1.0  # Initial zoom factor

    def button_widget(self, content_frame):
        buttons = [
            ("CnC Lab", lambda event: self.multi_action(locations["CnC lab"], "Page1")),
            ("3D Printing Lab", lambda event: self.multi_action(locations["3d printing lab"], "Page1")),
            ("Chemistry lab", lambda event: self.multi_action(locations["Chemistry lab"], "Page1")),
            ("Woodworking Lab", lambda event: self.multi_action(locations["Woodworking lab"], "Page1")),
            ("Vortex Lab", lambda event: self.multi_action(locations["Vortex lab"], "Page1")),
            ("Mechanical Lab", lambda event: self.multi_action(locations["Mechanical lab"], "Page1")),
            ("Workshop", lambda event: self.multi_action(locations["Workshop"], "Page1")),
            ("Power & Machine Lab", lambda event: self.multi_action(locations["Power and machine lab"], "Page1")),
            ("Back to Main", lambda event: self.change_page("Page1")),
        ]

        row_num = 2
        col_num = 0

        for text, callback in buttons:
            label = button_base(content_frame, text=text) 
            label.bind("<Button-1>", callback)  # Bind the event 
            label.grid(row=row_num, column=col_num, padx=10, pady=10)
            
            col_num += 1
            if col_num > 2:
                col_num = 0
                row_num += 1



class Page3(PageFeatures):
    def __init__(self, parent, controller):
        PageFeatures.__init__(self, parent, controller)
        # -- SET BACKGROUND IMAGE
        self.bg_image = Image.open(os.path.join(bringup_dir, 'maps', 'New_ob.pgm'))  # PATH TO IMAGE
        self.bg_image = self.bg_image.convert("RGB")
        self.bg_image = self.bg_image.resize((800, 480), Image.LANCZOS)
        self.bg_photo = ImageTk.PhotoImage(self.bg_image)

        bg_label = tk.Label(self, image=self.bg_photo)
        bg_label.img = self.bg_photo
        bg_label.place(x=0, y=0, relheight=1, relwidth=1)

        # Create widgets for page 3
        self.label = tk.Label(self, text="Notifying mall staff...",font=("Helvetical",16,"bold"), bg="#e2e2e2")
        self.label.pack(pady=(180,20))

        self.text_button = button_base(self, text="BACK TO MAIN")
        self.text_button.bind("<Button-1>", lambda event: self.change_page("Page1"))
        self.text_button.pack()

class Publisher(Node):

    def __init__(self):
        self.app = PageManager()
        super().__init__('tk_destinations_node')
        self.publisher1 = self.create_publisher(PoseStamped, 'tk_destinations', 10)
        self.publisher3 = self.create_publisher(String, 'tk_destinations_names', 10)
        self.subscription = self.create_subscription(String, 'result', self.listener_callback, 10)
        self.subscription1 = self.create_subscription(String, 'tk_destinations_reached', self.listener_callback1, 10)
        self.publisher2 = self.create_publisher(String, 'nav_commands', 10)
        timer_period = 0.02  # seconds
        self.topic1 = self.create_timer(timer_period, self.timer_callback)
        self.topic2 = self.create_timer(timer_period, self.timer2_callback)
        self.i = 0
        self.engine = pyttsx3.init()

    def listener_callback(self, msg):
        words = msg.data.lower()
        if "i am lost" in words:
          self.app.show_page("Page3")
        if "stop" in words or "excuse me" in words or "help" in words:
          commands.append('stop')

    def listener_callback1(self, msg):
        words = msg.data.lower()
        print(words)
        self.engine.say("You have arrived at " + words)
        self.engine.runAndWait()

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
        name = String()
        if goals:
          msg = goals.pop()
          name.data = goal_names.pop()
          self.publisher1.publish(msg)
          self.publisher3.publish(name)
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
