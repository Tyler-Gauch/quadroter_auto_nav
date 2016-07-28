#include <iostream>
#include <cmath>

#include <ros/ros.h>
//ROS Topic Headers
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Range.h>

//TF Headers
#include <tf/transform_listener.h>

//the higher the number the more stable the 
//map will be but the longer it will wait
#define STABILIZE_SAMPLE_THRESHOLD 25 

ros::NodeHandle * nh;

ros::Subscriber cmd_vel_sub;	//gets the wanted cmd_vel tranforms
ros::Subscriber pose_sub;	//gets the wanted cmd_vel tranforms

float changeY = 0, changeX = 0;
float wantedHoldX = 0, wantedHoldY = 0;
bool firstTime = true;
bool mapStabilized = false;
float currentXTotal = 0, currentYTotal = 0;
int numSamples = 0;

void cmdVelCallback(const geometry_msgs::Twist &lastCommand)
{
	
}

void poseCallback(const geometry_msgs::PoseStamped &currentPosition){
	
	float currentX = currentPosition.pose.position.x * 100; //into cm
	float currentY = currentPosition.pose.position.y * 100; //into cm

	if(firstTime)
	{
		firstTime = false;
		wantedHoldX = currentX;
		wantedHoldY = currentY;
		return;
	}

	changeX = currentX - wantedHoldX;
	changeY = currentY - wantedHoldY;

	if(abs(changeX) < 0.5f)
	{
		changeX = 0;
	}

	if(abs(changeY) < 0.5f)
	{
		changeY = 0;
	}

	//lets keep getting scans until we have a decent map
	if(!mapStabilized)
	{
		numSamples++;
		currentXTotal += changeX;
		currentYTotal += changeY;

		if(numSamples > STABILIZE_SAMPLE_THRESHOLD)
		{
			float avgX = currentXTotal / numSamples;
			float avgY = currentYTotal / numSamples;			

			if(avgX > 1 || avgY > 1)
			{
				numSamples = 0;
				currentXTotal = 0;
				currentYTotal = 0;
				std::cout << "Waiting for map to stabilize..." << std::endl;
				return;
			}else{
				mapStabilized = true;
				std::cout << "Map stabilized" << std::endl;
			}
		}else{
			return;
		}
	}

	std::cout << "DX\t" << changeX << "\tDY:\t" << changeY << std::endl;

}

int main(int argc, char** argv){

	ros::init(argc, argv, "quadroter_auto_nav_radio_sim");

	nh = new ros::NodeHandle();

	cmd_vel_sub = nh->subscribe("/cmd_vel", 1, cmdVelCallback); //topic, queue size, callback
	pose_sub = nh->subscribe("/slam_out_pose", 1, poseCallback); //topic, queue size, callback

	while(nh->ok())
	{
		ros::spinOnce(); // needed to get subscribed messages
	}

}
