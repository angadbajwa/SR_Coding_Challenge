#pragma once

//standard libs
#include "boost/thread/thread.hpp"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

//generation constants
#define SECTOR_STEP 1.1

//plane state enum
enum class PlaneState {
	TRAVELLING,
	HOLDING,
	LANDED
};

//generalized sector class
class Sector {
public:
	//parametric constructor
	Sector(float x_, float y_) :
		x(x_), y(y_) {}
	//default constructor
	Sector() :
		x(0), y(0) {}
	float x, y;

	float get_x() { return x; }
	float get_y() { return y; }
};

//node class for node positioning
class PlaneNode : public Sector {
public:
	//parametric constructor
	PlaneNode(bool occupied_, float x_, float y_) :Sector(x_, y_) {
		occupied = occupied_;
	}
	bool occupied;
	int occupant_id;
};

//management class to hold Plane object 
class PlaneControl {
public:
	//parametric constructor with XY pos, id, and current state (travelling by default on spawn)
	PlaneControl(float x_pos_, float y_pos_, int id_) {
		id = id_;
		x_pos = x_pos_; y_pos = y_pos_;
		state = PlaneState::TRAVELLING;
	}
	int id; 
	float x_pos, y_pos;
	enum PlaneState state;
};

class ATC {
private:
	//traffic control intrinsics
	float zone_diameter; //km
	float runway_num;
	std::vector<Sector> runway_dims; //corresponding dims to runway locations
	std::vector<Sector> runway_loc; //runway locations
	int airplane_num;

	//sector management
	std::vector<PlaneNode> sectorQueue;
	std::vector<PlaneNode> runwayNodes;

	//plane management
	std::vector<PlaneControl> planeVector;

	//pubsub
	ros::Publisher pos_publisher;
	ros::Subscriber dest_sub;

public:
	//parametric constructor
	ATC(float zone_diameter_) {
		zone_diameter = zone_diameter_;

		initialize_pubsub();
	}

	//runway management - asynchronous functions for initial ATC startup
	void generate_runway(float x, float y, float width, float length);
	void generate_holding_sectors();

	//airplane localization on spawn
	Sector find_arrival_node(Sector plane_pos, int plane_id);

	//update plane state
	void update_plane_state(int plane_id, PlaneState state);

	//looping node assignments
	void update_node_assignments();

	//looping node states
	void update_node_states();

	//get current plane position (as known to ATC) by id
	Sector pos_from_id(int plane_id);

	float get_zone_dia() {
		return zone_diameter;
	}

	void print_runway_num() {
		std::cout << runway_num << std::endl;
	}

	//pubsub management
	void run_pubsub(int* publish_rate);
	void initialize_pubsub();
	void position_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
};