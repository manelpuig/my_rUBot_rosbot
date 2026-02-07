# Rosbot

Original rosbot_ros is container at /rosbot_ros

You work on /root

you clone your `my_rUBot_rosbot` in /root/

you create 2 new packages:
- my_rosbot_bringup
- my_rosbot_control

To simulate:
```bash
ros2 launch rosbot_gazebo simulation.launch.py
```

For our custom:
```bash
ros2 launch my_rosbot_bringup bringup_custom_sw.launch.py
ros2 run my_rosbot_control teleop_simple 
```