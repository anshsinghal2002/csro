# csro
An augmented reality based robot first person shooter utilising a ROS based messaging architecture. HUD implementation, OpenCV based hit detection, joystick control, unique game-modes and weapon classes.


## code to copy robot launch files
scp -r robot_launch_files/ ubuntu@TURTLEBOT3_03.union.edu:~/catkin_ws/src/turtlebot3/turtlebot3_bringup/launch
roscd turtlebot3_bringup/launch/robot_launch_files
chmod +x csro_camera.launch csro_core.launch csro_robot.launch csro_lidar.launch

## code to launch robot
roslaunch turtlebot3_bringup csro_robot.launch player_id:=player_01

## code to launch player
roslaunch csro player.launch player_id:=player_01