# ROS Development Storage (THORMANG)
This works has done by jimin lee and suhan park.
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

1. Required Device <br />
  (1) DS-MPE-SER4M<br />
  (2) Dynamixel PRO H54, H42 Series<br />

2. Required Package and installation<br />
  (1) Xenomai 2.4.6 kernel and libraries<br />
  (2) rosrt<br />
	$ sudo apt-get install ros-indigo-rosrt<br />
  (3) rt_dynamixel_msgs (included)<br />

3. How to use?<br />
  - Service name
  	- Mode Setting (Server): 	/rt_dynamixel/mode
  	- Motor Setting (Server):	/rt_dynamixel/motor_set
  - Topic name
  	- Joint State (Publish): 	/rt_dynamixel/joint_state
  	- Joint Set (Subscribe): 	/rt_dynamixel/joint_set

  (1) Setting Mode
	- Call a ros service to change the mode (rt_dynamixel_msgs::ModeSettingRequest::SETTING)
	- Call a ros service to set a motor (rt_dynamixel_msgs::MotorSetting)
	
	- Modes
		mode = rt_dynamixel_msgs::MotorSettingRequest::SET_GOAL_POSITION
			id = target motor id
			fvalue = radian
		mode = rt_dynamixel_msgs::MotorSettingRequest::SET_TORQUE_ENABLE
			value = 1 or 0 (torque ON = 1, OFF = 0)

  (2) Freerun Mode (400 Hz)
  	- Call a ros service to change the mode (rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN)
  	- Publish target angle datas(Radian) to joint_set
  
  (3) Joint Data
	- Current joint state datas are published via /rt_dynamixel/joint_state (400 Hz)
	- If motor set service is called, data updating is stopped shortly.

##User Interface
Qt based user interface

thanks for prepared package company ^.^
