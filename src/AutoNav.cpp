#include "AutoNav.h"
#include "CommonUtils.h"
#include <sys/time.h>

typedef unsigned long long timestamp_t;

static timestamp_t get_timestamp ()
{
  struct timeval now;
  gettimeofday (&now, NULL);
  return  now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
}

nav_msgs::OccupancyGrid::ConstPtr map;
sensor_msgs::Range::ConstPtr height;
geometry_msgs::PoseStamped::ConstPtr pose;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	map = msg;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr &msg)
{
	height = msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	pose = msg;
}

AutoNav::AutoNav(ros::NodeHandle &n)
{
	nh = n;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	debug = new Debugger(nh, "AutoNav");
	lookatDebug = new Debugger(nh, "lookat");
	map_sub = nh.subscribe("/map", 1, mapCallback); //topic, queue size, callback
	sonar_sub = nh.subscribe("/sonar_height", 1, sonarCallback);
	pose_sub = nh.subscribe("/slam_out_pose", 1, poseCallback);
}

//void AutoNav::moveTo(tf::StampedTransform pose, float x, float y, float & lx, float & ly)
void AutoNav::moveTo(float x, float y, float & lx, float & ly)
{
	
}

//void AutoNav::lookAt(tf::StampedTransform pose, float x, float y, float & ax, float & ay)
void AutoNav::lookAt(int x, int y, float &az, bool initialTurn, double &angle)
{
//	if (initialTurn)
//	{
	int curX = CommonUtils::getGridXPoint(transform.getOrigin().x(), map);
	int curY = CommonUtils::getGridXPoint(transform.getOrigin().y(), map);

	// Normalizes the angle to be between 0 circle to + 2 * M_PI circle. Returns in radians.
	angle = angles::normalize_angle_positive(PI + tf::getYaw(transform.getRotation()));

	// Fix the angle
	double angleDeg = angles::to_degrees(angle);
	if (angleDeg > 90 && angleDeg < 180) {
		angleDeg -= 180;
	} else if (angleDeg >= 180 && angleDeg < 270) {
		angleDeg -= 180;
	} else if (angleDeg >= 270) {
		angleDeg -= 360;
	}
	// Get a point X units on the current facing line of sight
	int newX = 25 * cos(angles::from_degrees(angleDeg));
	int newY = 25 * sin(angles::from_degrees(angleDeg));

	if (angles::to_degrees(angle) > 90 && angles::to_degrees(angle) <= 270) {
		newX += curX;
		newY += curY;
	} else {
		newX = curX - newX;
		newY = curY - newY;
	}

//	std::cout << angles::to_degrees(angles::normalize_angle_positive(PI + tf::getYaw(transform.getRotation()))) << std::endl;

	lookatDebug->addPoint(CommonUtils::getTransformXPoint(newX, map),
			CommonUtils::getTransformYPoint(newY, map),
			0);
	lookatDebug->addPoint(CommonUtils::getTransformXPoint(curX, map),
				CommonUtils::getTransformYPoint(curY, map),
				0);
	lookatDebug->addPoint(CommonUtils::getTransformXPoint(x, map),
				CommonUtils::getTransformYPoint(y, map),
				0);

	// Now, we have three points on a triangle to compute the angle. So get the distances
	double newToCurDist = getDistance(newX, newY, curX, curY);
	double curToNextDist = getDistance(curX, curY, x, y);
	double newToNextDist = getDistance(newX, newY, x, y);

	// We need to get the angle opposite from the new coordinate to the next coordinate (newToNextDist),
	// because that is the angle we are going to be turning to look at the next position.

	// Use the cosine rule since sine will only give us acute angles
	double newToNextAngle = acos(
			(pow(newToCurDist, 2) + pow(curToNextDist, 2) - pow(newToNextDist, 2)) / (2 * newToCurDist * curToNextDist)  // Get cos(newToNextDist)
			);

	//if (newToNextAngle)

//	angle = angles::normalize_angle_positive(angle + newToNextAngle);
	angle -= newToNextAngle;

//	if (y > curY) {
//		angle = angles::normalize_angle_positive(angle + atan2(x - curX, y - curY));
//	} else {
//		angle = angles::normalize_angle_positive(angle + atan2(y - curY, x - curX) + PI);
//	}
//
//	std::cout << "Quadrotor Angle: " << angles::to_degrees(angles::normalize_angle_positive(PI + tf::getYaw(transform.getRotation()))) << std::endl;
//	std::cout << "newToNext Angle: " << angles::to_degrees(newToNextAngle) << std::endl;

	std::cout << "Quadrotor Angle: " << angles::to_degrees(angles::normalize_angle_positive(PI + tf::getYaw(transform.getRotation()))) <<
			"   newToNext: " << angles::to_degrees(newToNextAngle) << "   combined: " << angles::to_degrees(angles::normalize_angle_positive(angle)) << std::endl;

	double angleDif = angles::normalize_angle(tf::getYaw(transform.getRotation()) - angle - PI);
//	std::cout << "Angle compensation: " << angleDif << std::endl;

//	// ~5 degrees either way so stop moving
	if(angleDif >= -0.0872665 && angleDif <= 0.0872665)
	{
		az = 0;
	} else if (angleDif <= 0.3 && angleDif > 0)
	{
		az = 0.05;
	} else if (angleDif >= -0.3 && angleDif < 0)
	{
		az = -0.05;
	} else if (angleDif > 0.3)
	{
		az = 0.5;
	} else
	{
		az = -0.5;
	}

//	az = 0.25;

}

double AutoNav::getDistance(int x1, int y1, int x2, int y2)
{
	return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void AutoNav::sendMessage(float linX, float linY, float linZ, float angX, float angY, float angZ)
{
	geometry_msgs::Twist cmd;
	cmd.linear.x = linX;
	cmd.linear.y = linY;
	cmd.linear.z = linZ;
	cmd.angular.x = angX;
	cmd.angular.y = angY;
	cmd.angular.z = angZ;

	cmd_vel_pub.publish(cmd);
}


void AutoNav::doNav(){

	float lx = 0.0, ly = 0.0f, lz = 0.0f;
	float ax = 0.0f, ay = 0.0f, az = 0.0f;

	int gridx, gridy;
	int currentIndex;
	std::vector<State> path;
	bool atHeight = false;
	double angle = -999;


	while(nh.ok())
	{
		ros::spinOnce(); // needed to get subscribed messages

		debug->removePoints();
		lookatDebug->removePoints();

		lx = ly = lz = 0;
		ax = ay = az = 0;

		try{
			//gets the current transform from the map to the base_link
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			//gets us our current x,y coordinate in the occupancy grid
			float oy = transform.getOrigin().y();
			float ox = transform.getOrigin().x();

			if(!atHeight && transform.getOrigin().z() >= 1)
			{
				atHeight = true;
			} else if(transform.getOrigin().z() < 1)
			{
				//start with getting off the ground
				ax = 0.9;
				az = 0.9;
				ay = 0.9;
				lz = 0.25;
			} else if(path.size() == 0 && atHeight == true)
			{
				gridx = CommonUtils::getGridXPoint(ox, map);
				gridy = CommonUtils::getGridYPoint(oy, map);
				
				currentIndex = CommonUtils::getIndex(gridx,gridy, map);
				Problem p(nh, map);
				timestamp_t t0 = get_timestamp();

				State startState(gridx, gridy, map->data[currentIndex]);
				path = p.search(startState);

				timestamp_t t1 = get_timestamp();
				std::cout << "Got Path with size " << path.size() << " taking " << (t1 - t0) / 1000.0L << " ms" << std::endl;
			} else if(path.size() > 0)
			{
				State s = path.back();
				State goal = path.front();
				if (angle == -999)
				{
					lookAt(goal.x, goal.y, az, true, angle);
				} else
				{
					lookAt(goal.x, goal.y, az, false, angle);
				}

				float nextX = CommonUtils::getTransformXPoint(s.x, map);
				float nextY = CommonUtils::getTransformXPoint(s.y, map);
				for(std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
				{
					debug->addPoint(CommonUtils::getTransformXPoint(i->x, map), CommonUtils::getTransformYPoint(i->y, map), 0);
				}
			}
			debug->publishPoints();
			lookatDebug->publishPoints();
			sendMessage(lx, ly, lz, ax, ay, az);
		}catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
	}
}

// Gets a list of occupancy grid indexes where there is an obstacle
// within a square around the UAV (boundary length determined by threshold)
void AutoNav::getSurroundingPoints(int centerX, int centerY, int threshold) {
	std::vector<int> occupiedIndexies;

	for (int i = centerX - threshold; i <= centerX + threshold; i++) {
		for (int j = centerY - threshold; j <= centerY + threshold; j++) {
			// Convert x and y into single index for occupancy grid
			int curIndex = (i * map->info.width) + j;
			if (map->data[curIndex] == this->MAP_POSITIVE_OBJECT_OCCUPIED) {
				occupiedIndexies.push_back(curIndex);
			}
		}
	}
}

void AutoNav::land()
{
	sendMessage(0,0,-0.25,0,0,0);
}