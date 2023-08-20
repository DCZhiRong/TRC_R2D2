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
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-teleop-twist* ros-humble-gazebo* ros-humble-ros2-control* ros-humble-twist-mux ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

### Hardware drivers
Next are the device drivers, currently the project uses an intel realsense d-series camera along with a rplidar a1, so if you are not using the same components you can skip this step

Lidar:
```
sudo apt install ros-humble-rplidar-ros
```
Realsense(instructions taken from intel's repo):

https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages



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

## How to run the robot in real world(Not working yet):
Start the real world launch file:(Not working yet)
```
ros2 launch r2d2 launch_robot.launch.py
```
