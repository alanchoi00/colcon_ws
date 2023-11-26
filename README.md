# Steps to compile
(assumed src.tgz is exported to ~/colcown_ws/src)
1. Head into your colcon_ws
```
cd ~/colcon_ws
```
2.Build the message interface package for wall-follower
```
colcon build --packages-select wall_follower_msgs
```
3. Build the rest of the packages
```
colcon build
```
