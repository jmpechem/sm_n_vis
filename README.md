# ROS Development Storage (THORMANG)
This works has done by jimin lee and suwan park.
All systems running in Ubuntu 14.04 LTS ROS indigo system.

Here is some guide line to use these packages.

1. Required Device
  (1) Hokuyo UTM-30LX-EW
  (2) Dynamixel MX-28(RS485 version)
  (3) USB2Dynamixel

2. Required Package and installation
  (1) SMACH state machine package
	$ sudo apt-get install ros-indigo-smach
  (2) smach_viewer
	$ sudo apt-get install ros-indigo-smach-viewer
  (2) urg_node
	$ sudo apt-get install ros-indigo-urg-node
  (3) dynamixel motor
	$ sudo apt-get install ros-indigo-dynamixel-motor

3. How to use?
  (1) rot_lidar package
      - sudo chmod a+rw /dev/ttyUSBX (X means number of ttyUSB number)
      - in launch directory, do this command for start package
	$ roslaunch start_system.launch
      - publish topic
        cloud(sensor_msgs::PointCloud2) : one layer point cloud
        assembled_cloud(sensor_msgs::PointCloud2) : multi layer point cloud when lidar in rotating
      - subscribe topic
	sweep_cmd(std_msgs::Int32) : 1 means sweep other is nothing
  (2) smach_usercase package
      - now under construction
  (3) thormang_biped package
      - in launch directory, do this command for start package
        $ roslaunch display.launch
  (4) move_model package
      - test package for moving thormang model in rviz

##SMACH
State machine
##Vision
Vision
##Dynamixel PRO(Realtime)
Dynamixel Pro node (xenomai)
##User Interface
Qt based user interface

thanks for prepared package company ^.^
