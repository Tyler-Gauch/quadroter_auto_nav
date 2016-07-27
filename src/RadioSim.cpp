#include <iostream>
#include <ros/ros.h>
//ROS Topic Headers
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Range.h>

//TF Headers
#include <tf/transform_listener.h>


geometry_msgs::Twist lastCommand;

void cmdVelCallback(const geometry_msgs::Twist &msg)
{
	lastCommand = msg;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "quadroter_auto_nav_radio_sim");
	ros::NodeHandle nh;

	ros::Subscriber cmd_vel_sub;	//gets the wanted cmd_vel tranforms
	tf::TransformListener listener;	//listens for location updates
	tf::StampedTransform transform;	//holds our last known location

	cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmdVelCallback); //topic, queue size, callback

	float changeY = 0, changeX = 0;
	float lastX = 0, lastY = 0;

	while(nh.ok())
	{
		ros::spinOnce(); // needed to get subscribed messages

		try
		{
			//gets the current transform from the map to the base_link
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			//gets us our current x,y coordinate in the occupancy grid
			float currentX = transform.getOrigin().x();
			float currentY = transform.getOrigin().y();
			
			changeY = currentY - lastY;
			changeX = currentX - lastX;

			std::cout << "X/DX" << currentX <<"/" << changeX << " Y/DY " << currentY << "/" << changeY << std::endl;

			lastX = currentX;
			lastY = currentY;
			
		}catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
	}

}
