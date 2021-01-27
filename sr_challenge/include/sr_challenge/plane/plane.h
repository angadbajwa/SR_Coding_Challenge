#pragma once

//custom libs
#include "sr_challenge/atc/atc.h"
#include "sr_challenge/utils.h"

//standard libs
#include "boost/thread/thread.hpp"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

class Plane {
public:
	//parametric constructor
	Plane(float zone_diameter, int id_) {
		//upon plane generation, make arbitrary spawn point on circle edge
		float rand_theta = rand() % 360;
		float x = zone_diameter * cosf(rand_theta);
		float y = zone_diameter * sinf(rand_theta);

		position = Sector(x, y);

		//initialize ID
		id = id_;

		//initialize internal velocity
		velocity = .14; //.14 km/s = 140 m/s

		//initialize unit vector to nil
		unit_vec = Sector(0, 0);

		//initialize pubsub
		initialize_pubsub();
	}

	//pubsub management
	void run_pubsub(int* publish_rate);
	void initialize_pubsub();
	void destination_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

	//unit vector generation and position update
	void generate_unitVec();
	void update_position();
private:
	Sector position; 
	Sector destination;
	Sector unit_vec;
	float velocity;
	int id;
};