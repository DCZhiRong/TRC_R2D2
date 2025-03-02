# TRC_R2D2
The ros2 humble packages used for the trc r2d2 v2

In case you have not installed Ros2 humble and setup your workspace: 

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html


## Some details about the package
This package relies heavily on ros2 control to handle interfacing with the robot and its hardware.

The two command interfaces correspond to the left and right wheel velocity repectively, while the state interfaces keep track of the wheel's velocity and postion

The ros2_control node in this package takes in commands from the diff_cont/cmd_vel_unstamped topic in order to control the motors


### Setup
Firstly run the following code to ensure that there's no funny business going on with your serial ports(We are disabling the brltty udev rules here instead of deleting them outright just incase)

```
for f in /usr/lib/udev/rules.d/*brltty*.rules; do
  sudo ln -s /dev/null "/etc/udev/rules.d/$(basename "$f")"
done
sudo udevadm control --reload-rules
sudo systemctl mask brltty.path
```

Next you need to add yourself to the dialout group in ubuntu as this ros package uses a lot of serial devices such as arduinos and lidars

```
sudo usermod -a -G dialout $USER
```
This will add you to the dialout group but will only take into affect once you reboot, so do that:

```
reboot
```
### Ros dependencies
Now that you have done the initial setup, its time to install some of the ros dependencies needed for this ros package to run

```
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-teleop-twist* ros-humble-gazebo* ros-humble-ros2-control* ros-humble-twist-mux ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-robot-localization libserial-dev ros-humble-v4l2-camera ros-humble-image-transport-plugins
```

### Hardware drivers
Next are the device drivers, currently the project uses an intel realsense d-series camera along with a rplidar a1, so if you are not using the same components you can skip this step

Lidar:
```
sudo apt install ros-humble-rplidar-ros
```
Realsense(instructions taken from intel's repo):

https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages


### Software drivers
Even more than that are the dependencies used for some of the software stuff
Voice recog:
https://gitlab.com/bob-ros2/voskros


## How to run the robot in simulation:
* Start the simulation launch file:
```
#Replace $ros_workspace$ with your ros workspace
cd 
ros2 launch r2d2 launch_sim.launch.py world:=$ros_workspace$/src/r2d2/worlds/obstacles.world
```

You should see gazebo pop up with the robot in a simulated gazebo environment and a bunch of messages in the terminal saying that the controller successfully spawned.

You can now plug in a controller to check if the robot can move around:
* For xbox style controllers 
  * "X" - to enable movement(held)
  * "Left joy" - move around
  * "Right bumper" - enable turbo(held)

* For playstation controllers
  * "Triangle" to enable movement(held)
  * "Left joy" - move around
  * "Right bumper" - enable turbo(held)

Next to view the robot in rviz
```
rviz2
```
You need to subsribe to the /scan and /robot_description through laserscan and RobotModel respectively
Set the fixed frame to odom and the rivz viewer would be done


## How to run the robot in real world:
Start the real world launch file(This just launches the robot and connects to the interfaces):
```
ros2 launch r2d2 launch_robot.launch.py
```
Next to view the robot in rviz
```
rviz2
```
You need to subsribe to the /scan and /robot_description through laserscan and RobotModel respectively
Set the fixed frame to odom and the rivz viewer would be done


## How to bring up the mapper(works in both simulation and real world):
This here brings up the slammer for the robot
```
#sim time depends on if you are running it in a simulation or irl
ros2 launch r2d2 slam_launch.py use_sim_time:=true/false
```

Move the robot using
```
ros2 run teleop_twist_joy teleop_twist_joy
```

## Localization
Whether you are in simulation or in real life the instructions here are the same(You only need to make sure the lidar is running if in real life)
#sim time depends on if you are irl or simulation
Sensor fusion
```
ros2 launch r2d2 ekf.launch.py use_sim_time:=true/false
```

Open another terminal
localization
```
ros2 launch r2d2 localization_launch.py use_sim_time:=true/false map:="PATH TO YOUR MAP.yaml"
```

Open another terminal
Navigation
```
ros2 launch r2d2 navigation_launch.py use_sim_time:=true/false
```


## Ui stuff
When you wanna run the ui programs use these commands
```
ros2 run r2d2 tk_ui.py
```
This launches the ui, which is, for now, a requirement for voice commands to work(my bad)
Commands so far:
  I am lost: Triggers emergency message broadcast
  Stop/Excuse me/Help: Stops the robot for 5 mins

'''
ros2 run r2d2 nav2_pos.py
'''
This launches the automatic navigation stuff 
