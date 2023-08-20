# TRC_R2D2
The ros2 packages used for the trc r2d2 v2

Think of this as the "src" of your ros workspace, so all the packages used are going to go here

Firstly run the following code to ensure that there's no funny business going on with your serial ports
```
for f in /usr/lib/udev/rules.d/*brltty*.rules; do
  sudo ln -s /dev/null "/etc/udev/rules.d/$(basename "$f")"
done
sudo udevadm control --reload-rules
sudo systemctl mask brltty.path
```

