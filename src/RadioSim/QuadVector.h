#ifndef _QUAD_VECTOR_
#define _QUAD_VECTOR_

#include <ros/ros.h>
//ROS Topic Headers
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Range.h>

class QuadVector{

public:
	QuadVector();
	QuadVector(geometry_msgs::Point p);
	QuadVector(geometry_msgs::Quaternion q);
	QuadVector add(QuadVector b);
	float dot(QuadVector b);
	QuadVector mult(float scalar);
	QuadVector cross(QuadVector b);
	QuadVector rotate(geometry_msgs::Quaternion q);

	float x;
	float y;
	float z;
};

#endif