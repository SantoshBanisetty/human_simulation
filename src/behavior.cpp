/*
 * behavior.cpp
 *
 *  Created on: Feb 23, 2017
 *      Author: santosh
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <string>

#define PI 3.14

bool commandFlag = false;
char* scenario;

double intensities[27];
double mul =1;

ros::Publisher velocity_publisher;

//Function declerations of move and rotate
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double relative_angle, bool clockwise);

//Call back decleration for the laser messages
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg);
//Command call back decleration
void commandCallBack(const std_msgs::String::ConstPtr& msg);

//Function declerations for equilidian distance and degrees to radians conversion
double getDistance(double x1, double y1, double x2, double y2);
double degrees2radians(double angle_in_degrees);

//Wanader without bumping into obstacles 
void wander(void);
void pass(void);

int main(int argc, char **argv)
{
  ROS_INFO("argc: %d, argv: %s", argc, argv[0]);
  if (argc != 2)
  {
  	ROS_INFO("In sufficient command line arguments");
  	return -1;
  }
  scenario = argv[1];
  ROS_INFO("I am [%s]", "In main"); 
  ros::init(argc, argv, "behavior");
  ros::NodeHandle behaviorNode;
 
  velocity_publisher = behaviorNode.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1000);
  ros::Subscriber laser = behaviorNode.subscribe("/robot_1/base_scan", 1, laserCallBack);
  ros::Subscriber command = behaviorNode.subscribe("/navigation_command", 100, commandCallBack);
  ros::Rate loop_rate(100000);


  ROS_INFO("I heard in main: [%d]", commandFlag);
  	
  //while(ros::ok())
  //{

  //}
	

 
  ros::spin();

  return 0;
}

/**
 *  makes the robot move with a certain linear velocity, for 
 *  a certain distance either forward or backward  
 */
void move(double speed, double distance, bool isForward){
   geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis and check condition for the direction
   if (isForward)
	   vel_msg.linear.x =abs(speed);
   else
	   vel_msg.linear.x =-abs(speed);
   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(100);
   //Condition to terminate if moved to the distance specified
   do{
	   velocity_publisher.publish(vel_msg);
	   double t1 = ros::Time::now().toSec();
	   current_distance = speed * (t1-t0);
	   ros::spinOnce();
	   loop_rate.sleep();
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   velocity_publisher.publish(vel_msg);

}

/**
 *  makes the robot turn with a certain angular velocity, for 
 *  a certain distance in either clockwise or counter-clockwise direction  
 */
void rotate (double angular_speed, double relative_angle, bool clockwise){
//angular_speed = degrees2radians(angular_speed);
//relative_angle = degrees2radians(relative_angle);
	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   //Condition to rotate clockwise or counter-clockwise
           if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
           //Condition used to terminate if rotated to the specifed angle
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians  
 */

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/*
 * get the euclidian distance between two points 
 */
double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

/*
 * Call back implementation to read and process laser data  
 */
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg)
{
ROS_INFO("I am in: [%s]", "laser call back");
for (int i=0; i<27; i++) // I need not loop to copy, I am not familiar with std::vectors
{
intensities [i]= laser_msg->intensities[i];
mul = mul*intensities[i]; //check point if robot is blocked 270 degrees
}
if (commandFlag == true)
  	{
  		if (strcmp("wander", scenario) == 0)
  		{
  			wander();
  		}
  		else if (strcmp("pass", scenario) == 0)
  		{
  			pass();
  		}
  		else
  		{
  			ROS_INFO("Invalid argument!, %s", scenario);
  			ROS_INFO("\"wander and pass\" are valid");
  			//return -1;
  		}
  		
	}
  	else
  	{
  		ROS_INFO("errrrrrr");
  	}
}

/*
 * Listen to the start command. 
 */
void commandCallBack(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  commandFlag = true;
  ROS_INFO("I heard: [%d]", commandFlag);
}

/*
 *"Wander without hitting anything" implementation
 */
void wander(void)
{
ROS_INFO("I am [%s]", "wandering");

int samples = 27;
int fov = 4.7123;
double inc = 0.1745; // 270/27 degrees to radians
int center = samples/2;
if (mul == 1)// blocked around 270 degrees
{
rotate(1.0, 3.1415, 1); //about turn
}
if ((intensities [center-1] == 1)||(intensities [center] == 1)||(intensities [center+1] == 1))// obstacle in front
{
	//Check one by one on both sides of the robot to determine free space and rotate by the amount scanned in a first free direction
	for (int i = 2; i< center; i++)
	{
		if(intensities [center - i] == 0)// no obstacle
		{
		rotate(1.0, (i+1)*inc, 1);
		break;
		}
		else if (intensities [center +i] == 0)// no obstacle
		{
		rotate(1.0, (i+1)*inc, 0);
		break;
		}
	}
}
else
{
move(1.0, 1.0, 1);
}

}

/*
 *"Passing behavior without hitting anything" implementation
 */
void pass(void)
{
ROS_INFO("I am [%s]", "passing");

int samples = 27;
int fov = 4.7123;
double inc = 0.1745; // 270/27 degrees to radians
int center = samples/2;
/*if (mul == 1)// blocked around 270 degrees
{
rotate(1.0, 3.1415, 1); //about turn
}*/
if ((intensities [center-1] == 1)||(intensities [center] == 1)||(intensities [center+1] == 1))// obstacle in front
{
	move(0.0, 0.0, 1);
}
else
{
move(1.0, 1.0, 1);
}

}