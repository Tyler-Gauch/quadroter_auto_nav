#include <iostream>
#include <cmath>

#include <ros/ros.h>
//ROS Topic Headers
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Range.h>

//TF Headers
//#include <tf/transform_listener.h>

// Serial handler
#include <serial/serial.h>
#include <string>
#include <cctype>
#include <sstream>

#include <boost/thread/thread.hpp>

#include "PID.h"
#include "QuadVector.h"
#include "Radio.h"
//#include "SerialHandler.h"


//the higher the number the more stable the 
//map will be but the longer it will wait
#define STABILIZE_SAMPLE_THRESHOLD 25 

#define CUTOFF_THRESHOLD 1.5f

#define AVG_THRESHOLD 1

//PID values for the Roll PID Controller
#define PID_X_P 	5.0f
#define PID_X_I 	0.5f
#define PID_X_D 	0.12f
#define PID_X_IMAX 	100.0f

//PID values for the Pitch PID Controller
#define PID_Y_P 	5.0f
#define PID_Y_I 	0.5f
#define PID_Y_D 	0.12f
#define PID_Y_IMAX 	100.0f

#define CONFIG_COMMAND	"initial_start"

//init ros::NodeHandle priv_nh("~");ros node
ros::NodeHandle * nh;

//init subscribers
ros::Subscriber cmd_vel_sub;	//gets the wanted cmd_vel tranforms
ros::Subscriber pose_sub;	//gets the wanted cmd_vel tranforms

//optical flow variables
//////////////////////////////////////////////////

//Position we want to hold at
float wantedHoldX = 0, wantedHoldY = 0;

//last known x and y position in cm
float currentX, currentY;


//if its first time for init stuff
bool firstTime = true;

//wait for map to stabilize
bool mapStabilized = false;

//number of samples taken waiting for map to stabilize
int numSamples = 0;

//current total movement in each axis
float currentXTotal = 0, currentYTotal = 0;

//roll output correction based off of x movement
int of_roll = 0;

//pitch output correction based off of y movement
int of_pitch = 0;

//pid to handle correctoin in x
PID pidX(
	PID_X_P,
	PID_X_I,
	PID_X_D,
	PID_X_IMAX);

//pid to handle correction in y
PID pidY(
	PID_Y_P,
	PID_Y_I,
	PID_Y_D,
	PID_Y_IMAX);

serial::Serial serialInst;
//SerialHandler *serialInst;

Radio throttle;
Radio roll;
Radio pitch;
Radio yaw;
