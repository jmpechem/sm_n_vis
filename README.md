# ROS Development Storage (THORMANG)
This works has done by jimin lee and suwan park.
All systems running in Ubuntu 14.04 LTS ROS indigo system.

Here is some guide line to use these packages.

1. Required Device <br />
  (1) Hokuyo UTM-30LX-EW<br />
  (2) Dynamixel MX-28(RS485 version)<br />
  (3) USB2Dynamixel<br />

2. Required Package and installation<br />
  (1) SMACH state machine package<br />
	$ sudo apt-get install ros-indigo-smach<br />
  (2) smach_viewer<br />
	$ sudo apt-get install ros-indigo-smach-viewer<br />
  (2) urg_node<br />
	$ sudo apt-get install ros-indigo-urg-node<br />
  (3) dynamixel motor<br />
	$ sudo apt-get install ros-indigo-dynamixel-motor<br />

3. How to use?<br />
  (1) rot_lidar package<br />
      - sudo chmod a+rw /dev/ttyUSBX (X means number of ttyUSB number)<br />
      - in launch directory, do this command for start package<br />
	$ roslaunch start_system.launch<br />
      - publish topic<br />
        cloud(sensor_msgs::PointCloud2) : one layer point cloud <br />
        assembled_cloud(sensor_msgs::PointCloud2) : multi layer point cloud when lidar in rotating <br />
      - subscribe topic <br />
	sweep_cmd(std_msgs::Int32) : 1 means sweep other is nothing <br />
  (2) smach_usercase package <br />
      - now under construction <br />
  (3) thormang_biped package <br />
      - in launch directory, do this command for start package <br />
        $ roslaunch display.launch <br />
  (4) move_model package <br />
      - test package for moving thormang model in rviz <br />

##SMACH
State machine
##Vision
Vision
##Dynamixel PRO(Realtime)
Dynamixel Pro node (xenomai)
##User Interface
Qt based user interface

thanks for prepared package company ^.^
