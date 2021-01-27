//standard libs
#include "ros/ros.h"

//custom libs
#include "sr_challenge/atc/atc.h"
#include "sr_challenge/plane/plane.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv) {

	//declare asynchronous instance of ATC class with intrinsics and generate runways and holding sectors pragmatically
	float zoneDiameter = 10;
	ATC baseATC = ATC(zoneDiameter);

	baseATC.generate_runway(0, 0, .05, .1);
	baseATC.generate_holding_sectors();

	//spawn a single plane, initialize pubsub for plane object
	Plane plane1 = Plane(baseATC.get_zone_dia(), 0);

	//start synchronous functionality
	ros::init(argc, argv, "base");
	ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
	ros::Rate loop_rate(10); // 10 Hz

	//primary thread spinner - control node is shared with pubsub threads in ATC/Plane instances
	while (ros::ok())
	{
		loop_rate.sleep();
		// process any incoming messages in this thread
		ros::spinOnce();
	}

	return 0;
}