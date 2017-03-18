/**
 * @file   Drone.h
 * @Author Enrico BORELLO (enrico.borello@yahoo.it)
 * @Author Louise BRASSEUR (louise.brasseur@etu.utc.fr)
 * @date   March, 2017
 * @brief  Library of our class drone.
 *
 * Here are the functions used to control the drones
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include "Drone.h"
#include <iostream>
#include <fstream>
#define numDrones 3 /** @def Number of drones piloted */

struct Point /**< points of the map */
{
	std::string node; /**< name of the area */
	double x; 
	double y;  
	double z;  
};

int main(int argc, char * argv[])       
{
   
    Point *point;
    std::string node;
    std::string action;
    std::string name;
    std::string movefrom;
    std::string moveto;
    double xmove;
    double ymove;
    double zmove;
    int i=0;
    int k=0; 
    int position [numDrones]={440, 464, 472}; /**< first position of each drone */
    int move=0;
    Drone drones [numDrones];
    ros::init (argc, argv, "pddl");
    ros::NodeHandle nh("~");

    std::string path[numDrones]={"firefly", "hummingbird", "pelican"}; /**< path name of each drone */

    ///Initialisation of the drones
    for (int i=0; i< numDrones; i++)
    {
        drones[i].setDrone(i, path[i], nh);
    }

    ///Initialisation of the map
    ///Counting the number of points
    /*TODO use function count line who return k*/
    std::ifstream file("/home/enrico/catkin_ws/src/pddl/plan/map_coordinates.pddl", std::ios_base::in);
    if (file.is_open()) 
    {
	while (std::getline(file, node))
	{
      	 	k++;
	}
        file.close();
    }
    else 
    {
    	std::cout << "Error opening file map_coordinates\n";
    }
	 
    ///Creating each point of the map  
    point = new Point[k];
    std::ifstream myfile("/home/enrico/catkin_ws/src/pddl/plan/map_coordinates.pddl", std::ios_base::in);   
    if (myfile.is_open()) 
    {
    	while (myfile >> node >>xmove >> ymove>>zmove )
       	{
	        point[i].node=node;
	        point[i].x=xmove;
	        point[i].y=ymove;
	        point[i].z=zmove;
	        i++;   
		std::cout << "the point " <<i<< " of the map is set."<<std::endl;
	}
	std::cout << "the map is set."<<std::endl;
	myfile.close() ;
    }
    else 
    {
    	std::cout << "Error opening file map_coordinates\n";
    }
	
    ///Reading the plan
    std::ifstream pfile("/home/enrico/catkin_ws/src/pddl/plan/plan.pddl", std::ios_base::in);     
    if (pfile.is_open()) 
    {
	///Positioning each drone to its first position
	std::cout<<"Positionning the drones."<<std::endl;
	for (int i=0; i<numDrones; i++)
	{
		while(drones[i].move(point[position[i]].x, point[position[i]].y, point[position[i]].z))
		{
			ros::spinOnce();
		}
        }
	std::cout<< "All drones are positioned."<<std::endl;
	int l=0;
	///Sending the position read from the plan
        while (ros::ok())
	{
		while (pfile >> action >>name >> movefrom>>moveto )
		{
			std::cout <<"Next action is read."<<std::endl;
			int position =0;
			while(moveto!=point[position].node)position++; 
			for (int i=0; i<numDrones; i++)
			{
				std::cout<< "drones[i].getName()/"<<drones[i].getName()<<"/"<<std::endl;
				std::cout<< "name/"<<name<<"/"<<std::endl;
				if (drones[i].getName()==name)
				{
					while(drones[i].move(point[position].x, point[position].y, point[position].z))
					{
						std::cout << "the drone " <<i<< "is asked to move to "<<position<<std::endl;
						ros::spinOnce();
					}
				}
			}
		}	
	}
    }
    else 
    {
    	std::cout << "Error opening file plan\n";
    }
    getchar();
    return 0;

}
