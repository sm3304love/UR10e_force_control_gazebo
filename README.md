# UR10e_force_control_gazebo
Force control of UR10e in gazebo based on Modern Robotics


## ModernRoboticCpp git
Modern Robotics c++ library

https://github.com/Le0nX/ModernRoboticsCpp


Clone it to your home directory and follow the installation instructions.

## Gravity compensation

(By default, gazebo is in a paused state.)

`roslaunch ur10e_description gravity_gazebo.launch`


![Peek 2022-12-06 13-29](https://user-images.githubusercontent.com/57741032/205815092-3b3ba3c1-dec6-4042-951e-4a322d875f69.gif)


## Force control

(By default, gazebo is in a paused state.)

When a target joint position command and required time are sent, the trajectory and torque to the target position are generated.

### Launch gazebo

`roslaunch ur10e_description gazebo_force_control.launch`

### Send target joint position and time to reach the target position

`rosrun ur10e_force_control force_command 0 -1 0 0 0 0 5`

![Peek 2022-12-06 13-38](https://user-images.githubusercontent.com/57741032/205816901-23ff4868-2643-45e4-b8b8-9e94fca66880.gif)

