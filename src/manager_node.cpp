#include <dynamixel_group_interface/interface_manager.h>
#include <ros/ros.h>

int main( int argc, char** argv ) {
	ros::init( argc, argv, "dynamixel_group_interface" );
	DynamixelGroupInterface::InterfaceManager id;

	ros::spin();

	return 0;
}
