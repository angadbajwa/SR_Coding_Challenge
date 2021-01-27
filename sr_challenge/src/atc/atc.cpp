//standard libs
#include "ros/ros.h"

//custom libs
#include "sr_challenge/atc/atc.h"
#include "sr_challenge/utils.h"

//generate and place runway in ATC zone
void ATC::generate_runway(float x, float y, float width, float length) {
	//add to runway_num and push location and dims
	runway_num++;
	runway_dims.push_back(Sector(width, length));
	runway_loc.push_back(Sector(x, y));

	//add runway profile (end of runway) to runwayNodes
	runwayNodes.push_back(PlaneNode(false, x, y + (length / 2)));
}

//pragmatically generate holding sectors (center of zone is (0, 0))
void ATC::generate_holding_sectors() {
	float center_distance = ((runway_dims[0].get_y()) / 2) + 1.1;
	float section_num = floor((2 * M_PI) / (2 * asinf(2.2 / (2 * zone_diameter))));
	float theta = ((2 * M_PI) / section_num);
	
	//build sectors from center outwards
	for (float i = center_distance; i < zone_diameter; i += SECTOR_STEP) {
		//generate points on outside of sector circle, starting from innermost points
		float section_num = floor((2 * M_PI) / (2 * asinf(2.2 / (2 * i))));
		float theta = ((2 * M_PI) / section_num);

		for (int theta_step = 0; theta_step < section_num; theta_step++) {
			float x = i * cosf(theta_step * theta);
			float y = i * sinf(theta_step * theta);

			//after generating sector center, push unoccupied to sectorQueue vector - will carry sector coords from highest priority (runway entry) to lowest
			sectorQueue.push_back(PlaneNode(false, x, y));
		}
	}
}

//finds an appropriate arrival node for the airplane to avoid conflicts in node assignment - assumes that node generation has taken place
Sector ATC::find_arrival_node(Sector plane_pos, int plane_id) {
	//iterate through runwayNodes to find available node
	for (auto node : runwayNodes) {
		if (!(node.occupied)) {
			node.occupied = true;
			node.occupant_id = plane_id;
			return Sector(node.x, node.y);
		}
	}

	//iterate through generated sectors to find an available node with highest priority
	for (int i = 0; i < sectorQueue.size(); i++) {
		if (!(sectorQueue[i].occupied)) {
			sectorQueue[i].occupied = true;
			sectorQueue[i].occupant_id = plane_id;
			return sectorQueue[i];
		}
	}
}

void ATC::initialize_pubsub() {
	//initialize publishing thread
	int publish_rate = 10;
	boost::thread plane_pubsub(run_pubsub, &publish_rate);
}

void ATC::position_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	std::vector<float> position_message = msg->data;

	//check if plane is within traffic control system - if present, update position, and if absent, generate arrival node and publish
	for (auto e : planeVector) {
		//if plane is present in control vector, update position from sent message and exit
		if (e.id == position_message[2]) {
			e.x_pos = position_message[0];
			e.y_pos = position_message[1];

			return;
		}
	}
	
	//if plane is not present, add to controlVector and publish arrival node location
	Sector positionSector = Sector(position_message[0], position_message[1]);
	planeVector.push_back(PlaneControl(positionSector.x, positionSector.y, position_message[2]));

	//generate arrival sector and publish with plane id
	Sector arrivalSector = find_arrival_node(positionSector, position_message[2]);

	//generate multiarray msg
	std_msgs::Float32MultiArray arrival_msg;
	std::vector<float> pub_vec = { arrivalSector.x, arrivalSector.y, (float)position_message[2] };

	//publish
	arrival_msg.data.insert(arrival_msg.data.end(), pub_vec.begin(), pub_vec.end());
	pos_publisher.publish(arrival_msg);
}

void ATC::run_pubsub(int* publish_rate) {

	//initialize shared control node
	ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

	//initialize pubsub members to send position/id and recieve destination/id
	ros::Publisher pos_publisher = node->advertise<std_msgs::Float32MultiArray>("destination", 10);
	ros::Subscriber dest_sub = node->subscribe("position", 100, &ATC::position_callback, this);

	std_msgs::Float32MultiArray msg;
	ros::Rate loop_rate(*publish_rate);
	while (ros::ok())
	{
		//generate and update node assignments at same frequency of message retrieval
		update_node_states();
		update_node_assignments();

		loop_rate.sleep();
	}
}

//from the plane_id, get the position as known to the ATC (from the planeControl vec)
Sector ATC::pos_from_id(int plane_id) {
	for (auto planeNode : planeVector) {
		if (planeNode.id = plane_id) {
			return Sector(planeNode.x_pos, planeNode.y_pos);
		}
	}

	//if illegal id, return invalid coords
	return Sector(-1, -1);
}

//update plane state, if arrived/travelling/holding
void ATC::update_plane_state(int plane_id, PlaneState state) {
	for (auto plane : planeVector) {
		if (plane.id == plane_id) {
			plane.state = state;
		}
	}
}

// starting from runway node, check for corresponding airplane position and update state between TRAVELLING and HOLDING
// for runway nodes, clear node if plane has arrived and set status to arrived
void ATC::update_node_states() {
	//iterate through runwayNodes - check if plane has arrived
	for (auto node : runwayNodes) {
		//if node is occupied, check distance from node location to plane location
		if ((node.occupied)) {
			//if arrived, change status and clear
			if (!(get_dist(node, pos_from_id(node.occupant_id)))) {
				node.occupied = false;
				update_plane_state(node.occupant_id, PlaneState::LANDED);
				node.occupant_id = -1;
			}
		}
	}

	//iterate through remainder of nodes and set internal states
	for (auto node : sectorQueue) {
		//if node is occupied, check distance from node location to plane location
		if ((node.occupied)) {
			//if arrived, change status
			if (!(get_dist(node, pos_from_id(node.occupant_id)))) {
				update_plane_state(node.occupant_id, PlaneState::HOLDING);
			}
		}
	}
}

// starting from runway node, if previous node has an arrival in the TRAVELLING or HOLDING state and current node is 
// unoccupied, set destination of previous node ID to current node - runway managed separately
void ATC::update_node_assignments() {
	//for simplicity, nodes are assigned in ascending priority on airplane spawn, so as soon as an unoccupied node is found, stop iterating

	//iterate through runwayNodes - check if plane has arrived and node is clear
	for (auto node : runwayNodes) {
		if (!(node.occupied)) {
			//if runway is clear, allow highest priority node occupant to land (as long as highest priority node is occupied)
			if (sectorQueue[0].occupied) {
				//publish landing message and clear sector

				//generate multiarray msg
				std_msgs::Float32MultiArray landing_msg;
				std::vector<float> pub_vec = { node.x, node.y, (float)sectorQueue[0].occupant_id };
				//publish
				landing_msg.data.insert(landing_msg.data.end(), pub_vec.begin(), pub_vec.end());
				pos_publisher.publish(landing_msg);

				//clear sector
				sectorQueue[0].occupied = false;
				sectorQueue[0].occupant_id = -1;
			}
			//if not occupied, exit
			else return;
		}
	}

	//iterate through sector nodes
	for (int i = 0; i < sectorQueue.size(); i++) {
		//if sector node is not occupied, check for next node occupany and publish destination message 
		if (!(sectorQueue[i].occupied)) {
			if (sectorQueue[i + 1].occupied) {
				//publish landing message and clear sector

				//generate multiarray msg
				std_msgs::Float32MultiArray landing_msg;
				std::vector<float> pub_vec = { sectorQueue[i].x, sectorQueue[i].y, (float)sectorQueue[i + 1].occupant_id };
				//publish
				landing_msg.data.insert(landing_msg.data.end(), pub_vec.begin(), pub_vec.end());
				pos_publisher.publish(landing_msg);

				//add occupant to new sector and remove from old sector
				sectorQueue[i].occupied = true;
				sectorQueue[i].occupant_id = sectorQueue[i + 1].occupant_id;

				sectorQueue[i + 1].occupied = false;
				sectorQueue[i + 1].occupant_id = -1;
			}
			//if no more current planes, exit 
			else return;
		}
	}
}

