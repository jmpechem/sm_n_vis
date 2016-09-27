#include <ros/ros.h>
#include <cstdio>
#include <boost/asio.hpp>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "face_ctrl");
  	
	boost::asio::io_service io;
	boost::asio::serial_port serial(io, "/dev/ttyACM0"); // 여기 수정
	

        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

	char ch = 0;

	while(true)
	{
		ch = getchar();
		ROS_INFO("char = %c",ch);
        	boost::asio::write(serial,boost::asio::buffer(&ch,1));
	}
}

