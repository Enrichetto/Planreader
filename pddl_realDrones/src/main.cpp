/**
 * @file   main.cpp
 * @Author Enrico BORELLO (enrico.borello@yahoo.it)
 * @Author Louise BRASSEUR (louise.brasseur@etu.utc.fr)
 * @date   March, 2017
 * @brief  Main function to control the drones.
 *
 * Here is the procedure used to control the drones
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include "Drone.h"
#include <iostream>
#include <fstream>

#define numDrones 1 /** @def Number of drones piloted */

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
    bool check=0;
    double xmove;
    double ymove;
    double zmove;
    int i=0;
    int k=0; 
    int position [numDrones]={20/*, 315, 512*/}; /**< first position of each drone */
    int move=0;
    Drone drones [numDrones];
    ros::init (argc, argv, "pddl");
    ros::NodeHandle nh;
    ros::Duration timeout(1);

    ///Initialisation of the drones
    for (int i=0; i< numDrones; i++)
    {
        drones[i].setDrone(i, nh);
    }

    ///Initialisation of the map
    ///Counting the number of points
    /*TODO use function count line who return k*/
    std::ifstream file("/home/carabidouil/catkin_ws/src/pddl/plan/map_coordinates.pddl", std::ios_base::in);
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
    std::ifstream myfile("/home/carabidouil/catkin_ws/src/pddl/plan/map_coordinates.pddl", std::ios_base::in);   
    if (myfile.is_open()) 
    {
    	while (myfile >> node >>xmove >> ymove>>zmove )
    	{
		point[i].node=node;
	        point[i].x=xmove;
	        point[i].y=ymove;
	        point[i].z=zmove;
	        i++;   
	}
        std::cout << "the map is set."<<std::endl;
	myfile.close() ; 
    }
    else 
    {
    	std::cout << "Error opening file map_coordinates\n";
    }
	
    ///Reading the plan
    std::ifstream pfile("/home/carabidouil/catkin_ws/src/pddl/plan/plan.pddl", std::ios_base::in);     
    if (pfile.is_open()) 
    {
	///Positioning each drone to its first position
	std::cout<<"Positioning the drones."<<std::endl;
	for (int i=0; i<numDrones; i++)
	{
		while(!check)
		{
			ros::spinOnce();
			check=drones[i].move(point[position[i]].x, point[position[i]].y, point[position[i]].z);
			sleep(15);
		}
		check=0;	
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
				if (drones[i].getName()==name)
				{
					while(!check)
					{
						std::cout << "the drone " <<i<< "is asked to move to "<<position<<std::endl;
						ros::spinOnce();
						check=drones[i].move(point[position].x, point[position].y, point[position].z);
						sleep(5);
					}
					check=0;
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
