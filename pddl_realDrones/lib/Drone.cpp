/**
 * @file   Drone.cpp
 * @Author Enrico BORELLO (enrico.borello@yahoo.it)
 * @Author Louise BRASSEUR (louise.brasseur@etu.utc.fr)
 * @date   March, 2017
 * @brief  Library of our class drone.
 *
 * Here are the functions used to control the drones
 */

#include "Drone.h"

void feedbackCB(const asctec_hl_comm::WaypointFeedbackConstPtr & fb)
{
	const geometry_msgs::Point32 & wp = fb->current_pos;
	ROS_INFO("got feedback: %fm %fm %fm %f° ", wp.x, wp.y, wp.z, fb->current_yaw*180/M_PI);
}

void activeCb()
{
	ROS_INFO("Goal just went active");
}

void doneCb(const actionlib::SimpleClientGoalState& state, const asctec_hl_comm::WaypointResultConstPtr & result)
{
	if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
	}
  	else
	{
		ROS_WARN("Finished in state [%s]", state.toString().c_str());
	}
  	const geometry_msgs::Point32 & wp = result->result_pos;
  	ROS_INFO("Reached waypoint: %fm %fm %fm %f°",wp.x, wp.y, wp.z, result->result_yaw*180/M_PI);
}


void Drone::setDrone (int number, ros::NodeHandle nh)
{
    	std::string numberStr = std::to_string(number);
    	name = "drone" + numberStr;
	///Activate communication with the drone
    	ac = new actionlib::SimpleActionClient<asctec_hl_comm::WaypointAction>(nh, "fcu/waypoint", true);
  	ROS_INFO("Waiting for action server to start.");
  	ac->waitForServer(); //will wait for infinite time
  	ROS_INFO("Action server started, sending goal.");
	
	std::cout << "the drone " <<number<< " is set."<<std::endl;
}

Drone::~Drone()
{
}

Drone::Drone()
{
}

bool Drone::move(float X, float Y, float Z)
{
	///Message creation to the format asctec_hl_comm::WaypointGoal for the new goal
	asctec_hl_comm::WaypointGoal goal;
	goal.goal_pos.x = X;
  	goal.goal_pos.y = Y;
  	goal.goal_pos.z = Z;

	goal.max_speed.x = 0.5;
	goal.max_speed.y = 0.5;
	goal.max_speed.z = maxSpeed;

	goal.goal_yaw = 0;

	goal.accuracy_position = accuracy;
	goal.accuracy_orientation = 0;
	
	///New goal sent to the drone
	ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCB);

        ros::Duration timeout(15);
	bool test= ac->waitForResult(timeout);
        std::cout << "test " << test << std::endl; 
        return test;
}

std::string Drone::getName()
{
	return name;
}

void Drone::setCurrentPosition(const geometry_msgs::PoseStamped position)
{
        currentPosition=position;
	std::cout<<"position"<<"  "<<getName()<< "  " <<this->getCurrentPosition().pose.position.x<<"  "<<this->getCurrentPosition().pose.position.y<<"  "<<this->getCurrentPosition().pose.position.z<<"  "<<std::endl;
}

geometry_msgs::PoseStamped Drone::getCurrentPosition()
{
        return currentPosition;
}

void Drone::setNextPosition(geometry_msgs::PoseStamped position)
{
        nextPosition=position;
}

geometry_msgs::PoseStamped Drone::getNextPosition()
{
        return nextPosition;
}
