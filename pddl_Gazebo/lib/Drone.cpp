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

Drone::~Drone()
{
}

Drone::Drone()
{
}

bool Drone::move(float X, float Y, float Z)
{
	///Message creation to the format trajectory_msgs::MultiDOFJointTrajectory for the new goal
	trajectory_msgs::MultiDOFJointTrajectory poseMsg;
        poseMsg.header.frame_id = "world";
	poseMsg.header.stamp = ros::Time::now();
        poseMsg.points.resize(1);
        poseMsg.points[0].transforms.resize(1);
        poseMsg.points[0].transforms[0].translation.x=X;
        poseMsg.points[0].transforms[0].translation.y=Y;
	poseMsg.points[0].transforms[0].translation.z=Z;
	///New goal sent to the drone
	getPublisher().publish(poseMsg);
	ros::spinOnce();

	///test that the drone has reached the goal
	float error=1;
	if ( (this->getCurrentPosition().pose.position.x-error)>=X || (this->getCurrentPosition().pose.position.x+error)<=X || (this->getCurrentPosition().pose.position.y-error)>=Y || (this->getCurrentPosition().pose.position.y+error)<=Y || (this->getCurrentPosition().pose.position.z-error)>=Z || (this->getCurrentPosition().pose.position.z+error)<=Z)
		{	
			return true;
		}
	else {return false;}    
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

void Drone::setPublisher(ros::NodeHandle nh)
{
	pub=nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+path+"/command/trajectory",1);
}    

ros::Publisher Drone::getPublisher()
{
	return pub;
}

void Drone::setDrone (int number, std::string path, ros::NodeHandle nh)
{
	this->path = path;
	std::string numberStr = std::to_string(number);
	name = "drone" + numberStr;
	///Set the subscriber
	poseSub= nh.subscribe<geometry_msgs::PoseStamped>("/"+path+"/odometry_sensor1/pose",1,&Drone::setCurrentPosition,this);
	///Set the publisher
	setPublisher(nh);
	std::cout << "the drone " <<number<< " is set."<<std::endl;
}

