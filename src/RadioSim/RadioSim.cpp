#include "RadioSim.h"

//constrain an integer to a specific range
int constrain_int32(int input, int min, int max){
	if(input < min)
	{
		return min;
	}else if(input > max)
	{
		return max;
	}else{
		return input;
	}
}

//get the needed correction from the PID controller 
//in a certain direction (x or y)
int getCorrection(float error, PID pid, int & of_dir){
    float new_dir = 0;
    float p,i,d;

    // check if new optflow data available
	p = pid.get_p(-error);
	i = pid.get_i(-error, 1.0f);
	d = pid.get_d(-error, 1.0f);
    
    new_dir = p+i+d;

    // limit amount of change and maximum angle
    of_dir = constrain_int32(new_dir, (of_dir-20), (of_dir+20));

    // limit max angle
    of_dir = constrain_int32(of_dir, -1000, 1000);

    return of_dir;
}

void resetX(){
	currentXTotal = 0;
    pidX.reset_I();
    wantedHoldX = currentX;
}

void resetY(){
	currentYTotal = 0;
    pidY.reset_I();
    wantedHoldY = currentY;
}

//retrieves last message to move the quadrotor and converts
//into PWM output for the motors.
void cmdVelCallback(const geometry_msgs::Twist &lastCommand){
	float rollTrim = 1000;
	float pitchTrim = 1000;
	float yawTrim = 1000;
	float throttleTrim = 1000;

	//lin x = roll
	float roll = (lastCommand.linear.x * rollTrim) + rollTrim;
	if(roll != rollTrim)
	{
		resetX();
	}
	//lin y = pitch
	float pitch = (lastCommand.linear.y * pitchTrim) + pitchTrim;
	if(pitch != pitchTrim)
	{
		resetY();
	}

	//lin z = throttle
	float throttle = (lastCommand.linear.z * throttleTrim) + throttleTrim;
	//ang z = yaw
	float yaw = (lastCommand.angular.z * yawTrim) + yawTrim;
}

float vectorDot(geometry_msgs::Point a, geometry_msgs::Point b){
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

geometry_msgs::Point vectorMult(geometry_msgs::Point a, float scalar){
	geometry_msgs::Point c;
	c.x = a.x * scalar;
	c.y = a.y * scalar;
	c.z = a.z * scalar;
	return c;
}

geometry_msgs::Point vectorAdd(geometry_msgs::Point a, geometry_msgs::Point b){
	geometry_msgs::Point c;
	c.x = a.x + b.x;
	c.y = a.y + b.y;
	c.z = a.z + b.z;
	return c;
}

geometry_msgs::Point vectorCross(geometry_msgs::Point a, geometry_msgs::Point b){
	geometry_msgs::Point c;
	c.x = (a.y * b.z) - (a.z * b.y);
	c.y = (a.z * b.x) - (a.x * b.z);
	c.z = (a.x * b.y) - (a.y * b.x);
	return c;
}

geometry_msgs::Point rotateQuat(geometry_msgs::Point p, geometry_msgs::Quaternion q){
	geometry_msgs::Point quatVector;
	quatVector.x = q.x;
	quatVector.y = q.y;
	quatVector.z = q.z;

	float scalar = q.w;

	geometry_msgs::Point a = vectorMult(quatVector, 2 * vectorDot(quatVector, p));
	geometry_msgs::Point b = vectorMult(p, scalar*scalar - vectorDot(quatVector, quatVector));
	geometry_msgs::Point c = vectorMult(vectorCross(quatVector, p), 2*scalar);

	return vectorAdd(vectorAdd(a, b), c);
}

//retrieves last updated position and calculates the neccessary movement
//needed to stay in one position.
void poseCallback(const geometry_msgs::PoseStamped &currentPosition){

	geometry_msgs::Point pos = rotateQuat(currentPosition.pose.position, currentPosition.pose.orientation);

	currentX = pos.x * 100; //into cm
	currentY = pos.y * 100; //into cm

	if(firstTime)
	{
		firstTime = false;
		wantedHoldX = currentX;
		wantedHoldY = currentY;
		return;
	}

	float changeX = currentX - wantedHoldX;
	float changeY = currentY - wantedHoldY;

	if(abs(changeX) < CUTOFF_THRESHOLD)
	{
		changeX = 0;
	}

	if(abs(changeY) < CUTOFF_THRESHOLD)
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

			if(avgX > AVG_THRESHOLD || avgY > AVG_THRESHOLD)
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

	getCorrection(changeX, pidX, of_roll);
	getCorrection(changeY, pidY, of_pitch);

	// Only print to screen if we moved positions
	if (prevX != currentX || prevY != currentY) {
		std::cout << "DX\t" << of_roll << "\tDY:\t" << of_pitch << std::endl;
	}

	prevX = currentX;
	prevY = currentY;
}

// Function runs on new thread as it is blocking
void checkForInput() {
	while (ros::ok()) {
		std::string input;
		std::getline(std::cin, input);
		serialInst.write(input);
	}
}

//you should know what this is
int main(int argc, char** argv){

	ros::init(argc, argv, "quadroter_auto_nav_radio_sim");

	nh = new ros::NodeHandle();

	//setup serial
	try {
		serialInst.setPort("/dev/ttyAMA0");
		serialInst.setBaudrate(57600);
		serial::Timeout to = serial::Timeout::simpleTimeout(10000);  // 10 second timeout
		serialInst.setTimeout(to);

		serialInst.open();
	} catch (serial::IOException& e) {
		ROS_ERROR_STREAM("Unable to open serial port");
		return -1;
	}

	// Ensure serial port is initialized and open
	if (serialInst.isOpen()) {
		ROS_INFO_STREAM("Serial port initialized successfully...");
	} else {
		ROS_ERROR_STREAM("Serial port initialization failed...");
		return -1;
	}

	cmd_vel_sub = nh->subscribe("/cmd_vel", 1, cmdVelCallback); //topic, queue size, callback
	pose_sub = nh->subscribe("/slam_out_pose", 1, poseCallback); //topic, queue size, callback

	// Create a new thread to write to serial
	boost::thread writeThread(checkForInput);

	while(nh->ok())
	{
		ros::spinOnce(); // needed to get subscribed messages
		std::string result;

		if (serialInst.available()) {
			try {
				result = serialInst.read(1000); //.read(input, 1000);
			} catch (serial::SerialException& e) {
				// Serial error occurred
			}
			std::cout << result << std::endl;
		}


	}

	writeThread.join();

}
