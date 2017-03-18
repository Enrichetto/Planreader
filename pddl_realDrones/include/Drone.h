/**
 * @file   Drone.h
 * @Author Enrico BORELLO (enrico.borello@yahoo.it)
 * @Author Louise BRASSEUR (louise.brasseur@etu.utc.fr)
 * @date   March, 2017
 * @brief  Declaration of class drone.
 *
 * Here are the declaration of all the functions and parameters used to control the drones
 */

#ifndef DRONE_H
#define DRONE_H

//[C++ Header files]
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <asctec_hl_comm/WaypointAction.h>

/** 
* @class Drone
*
* @authors BORELLO - BRASSEUR 
* 
* $Header $
*/
class Drone {

protected:
    std::string name; /*!< Name of the drone */
    geometry_msgs::PoseStamped currentPosition; /*!< Current position of the drone */
    geometry_msgs::PoseStamped nextPosition; /*!< Next position of the drone */
    actionlib::SimpleActionClient<asctec_hl_comm::WaypointAction> *ac; /*!< how to communicate with the drone */
    float maxSpeed=0.3; /*!< maximum speed of the drone */
    float accuracy=0.3; /*!< accuracy of the drone */

public:
    Drone();
    ~Drone();
    std::string getName();
    void setCurrentPosition(const geometry_msgs::PoseStamped position);
    geometry_msgs::PoseStamped getCurrentPosition();
    void setNextPosition(geometry_msgs::PoseStamped position);
    geometry_msgs::PoseStamped getNextPosition();

    /**
    * This function gives the order to the drone to move to a position.
    * @param the parameters are the cartesian coordinates where the drone has to move.
    * @return the boolean return is true if the drone has reached the position and false otherwise
    */
    bool move(float X,float Y, float Z);

    /**
    * This function initialises the parameters of the drones
    * @param number : the number of the drone to create its name
    * @param path : the path with which we can communicate with the drone
    */
    void setDrone(int number, ros::NodeHandle nh);
};

#endif //DRONE_DRONE_H

