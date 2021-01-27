//standard libs
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

//custom libs
#include "sr_challenge/plane/plane.h"

void Plane::initialize_pubsub() {
	//initialize publishing thread
	int publish_rate = 10;
	boost::thread plane_pubsub(run_pubsub, &publish_rate);
}

void Plane::destination_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	std::vector<float> destination_id = msg->data; 

	//if msg id matches plane id, recieve new destination and update unitVec
	if (destination_id[2] == id) {
		destination = Sector(destination_id[0], destination_id[1]);
		generate_unitVec();
	}
}

void Plane::run_pubsub(int* publish_rate) {

	//initialize shared control node
	ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

	//initialize pubsub members to send position/id and recieve destination/id
	ros::Publisher pos_publisher = node->advertise<std_msgs::Float32MultiArray>("position", 10);
	ros::Subscriber dest_sub = node->subscribe("destination", 100, &Plane::destination_callback, this);

	std_msgs::Float32MultiArray msg;
	ros::Rate loop_rate(*publish_rate);
	while (ros::ok())
	{
		//generate position + id vector, and then publish and update at 10Hz
		update_position();
		std::vector<float> pub_vec = { position.x, position.y, (float)id };
		msg.data.clear();
		msg.data.insert(msg.data.end(), pub_vec.begin(), pub_vec.end());
		pos_publisher.publish(msg);

		loop_rate.sleep();
	}
}

//generate unit vector in direction of destination for looping movement
void Plane::generate_unitVec() {
	float delta_x = destination.x - position.x; float delta_y = destination.y - position.y;
	float magnitude = sqrtf(pow(delta_x, 2) + pow(delta_y, 2));

	unit_vec.x = magnitude * delta_x; unit_vec.y = magnitude * delta_y;
}

//update position with assumed constant 10Hz update rate 
void Plane::update_position() {
	//if plane has reached destination, stop moving (representing a "holding" state)
	if (!(get_dist(destination, position))) {
		unit_vec.x = 0; unit_vec.y = 0;
		return;
	}

	//if not, proceed along straight-line path with constant velocity
	float vel_magnitude = velocity * .1; //velocity * .1s
	position.x += unit_vec.x * vel_magnitude;
	position.y += unit_vec.y * vel_magnitude;
}