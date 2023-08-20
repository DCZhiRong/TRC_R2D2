# TRC_R2D2
The ros2 humble packages used for the trc r2d2 v2

In case you have not installed Ros2 humble and setup your workspace: 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

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

Now that you have done the initial setup, its time to install some of the ros dependencies needed for this ros package to run

```
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-teleop-twist* ros-humble-gazebo* ros-humble-rplidar-ros ros-humble-ros2-control*
```