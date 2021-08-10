## OpenBot navigation stack

Link for documentation of move_base navigation package: http://wiki.ros.org/navigation/Tutorials/RobotSetup

Include TSDF surface cloud as here: https://answers.ros.org/question/231595/3d-pointcloud-to-2d-costmap-layer-projection-github-repo/

and this could also help: https://answers.ros.org/question/239190/point-clouds-not-generating-obstacles-in-costmap/


### Dependencies


### Scripts


### Launch pipeline


### Running the Twist commands

Install twist teleoperation packeg

```sudo apt-get install ros-melodic-teleop-twist-keyboard```

Launch the teleoperation listener that will subscribe to '/<robot_name>/cmd_vel'

```roslaunch openbot_navigation openbot_twist.launch robot_name:=hercules```

Launch remotely the teleop node with

```rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/<robot_name>/cmd_vel```


Additional info at: https://github.com/ros-teleop/teleop_twist_keyboard

