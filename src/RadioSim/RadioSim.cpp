#include "RadioSim.h"

#define DISTANCE_STOP_THRESHOLD 50

float lastChangeX, lastChangeY;
QuadVector lastPos;
float dt;
uint64_t lastTime;
bool mapGood = true;

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
	i = pid.get_i(-error, dt);
	d = pid.get_d(-error, dt);
    
    new_dir = p+i+d;

    // limit amount of change and maximum angle
    of_dir = constrain_int32(new_dir, (of_dir-20), (of_dir+20));

    // limit max angle
    of_dir = constrain_int32(of_dir, -75, 75);

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
	//lin x = roll
	roll.setOutput((lastCommand.linear.y * roll.getTrim()) + roll.getTrim());
	if(roll.getOutput() != roll.getTrim())
	{
		resetX();
	}
	//lin y = pitch
	pitch.setOutput((lastCommand.linear.x * pitch.getTrim()) + pitch.getTrim());
	if(pitch.getOutput() != pitch.getTrim())
	{
		resetY();
	}

	//lin z = throttle
	throttle.setOutput((lastCommand.linear.z * throttle.getTrim()) + throttle.getTrim());
	//ang z = yaw
	yaw.setOutput((lastCommand.angular.z * yaw.getTrim()) + yaw.getTrim());
}

//retrieves last updated position and calculates the neccessary movement
//needed to stay in one position.
void poseCallback(const geometry_msgs::PoseStamped &currentPosition){

	if(!mapGood)
	{
		std::cerr << "Map not good" << std::endl;
		of_roll = 0;
		of_pitch = 0;
		pitch.setOutput(pitch.getTrim());
		roll.setOutput(roll.getTrim());
		return;
	}

	QuadVector pos(currentPosition.pose.position);
	pos = pos.rotate(currentPosition.pose.orientation);

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
			std::cout << "Waiting for map to stabilize..." << std::endl;
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

	if(abs(changeY - lastChangeY) > DISTANCE_STOP_THRESHOLD || abs(changeX - lastChangeX) > DISTANCE_STOP_THRESHOLD )
	{
		of_pitch = 0;
		pitch.setOutput(pitch.getTrim());
		of_roll = 0;
		roll.setOutput(roll.getTrim());
		mapGood = false;
	}else{
		getCorrection(changeX, pidY, of_pitch);
//		of_pitch = -of_pitch;
		getCorrection(changeY, pidX, of_roll);
	}

	lastChangeX = changeX;
	lastChangeY = changeY;
	// Only print to screen if we moved positions
}

int getCheckSum(std::string message){
	int checkSum = 0;
	for(int i = 0; i < message.length(); i++){
		checkSum += message[i];
	}
//	std::cout << "Calculated: " << checkSum << std::endl;
	return checkSum;
}

void serialWriteWithCheckSum(std::string message){
	std::stringstream s;

	s << message << "(" << getCheckSum(message) << ")" << std::endl;
	message = s.str();

//	std::cout << "Sent: " << message << " (of_roll: " << of_roll << ", of_pitch: " << of_pitch << ")" << std::endl;
	serialInst.write(message);
}

//build out the needed drive command and send it to the 
//quadcopter
void sendPWM(){
	std::stringstream s;

	//s << "throttle:" << throttle.getOutput();
	s << "pitch:" << pitch.getOutput() + of_pitch;
	s << ",roll:"  << roll.getOutput() + of_roll;
//	s << ",yaw:"   << yaw.getOutput();
	serialWriteWithCheckSum(s.str());

}

bool checkCheckSum(std::string message){

	std::string::size_type pos = message.find("(");

	//no ( found we didn't get a checksum
	if(pos == std::string::npos)
	{
		return false;
	}

	std::string::size_type pos2 = message.find(")");
	//no ) ofund we didn't get entire checksum
	if(pos2 == std::string::npos)
	{
		return false;
	}

	std::string checkSum = message.substr(pos+1, pos2-1 - pos+1);
	message = message.substr(0, pos);
	std::cout << "CheckSum: " << checkSum << " Message: " << message << std::endl;

	int cs;
	std::istringstream(checkSum) >> cs;

	if(cs != getCheckSum(message))
	{
		return false;
	}
	
	return true;


}

//waits for the apm to send its configuration
//so we can get the proper values
void getConfig(){
	int waiting_count = 0;
	std::cerr << "Waiting for APM to Connect..." << std::endl;
	bool connected = false;
	while(true)
	{
		//just a little something so we no 
		//it didn't die

		if(serialInst.available()){
			std::string config;
			config = serialInst.read(10000); //10000 might be too big not sure yet

			std::cerr << "Checking: " << config << std::endl;

			if(!checkCheckSum(config))
			{
				continue;
			}else if(config.find("APM connected") != std::string::npos)
			{
				std::cerr << "APM Connected." << std::endl;
				serialWriteWithCheckSum(CONFIG_COMMAND);
				std::cout << "Waiting for config..." << std::endl;
				connected = true;
				continue;
			}

			if(!connected)
			{
				continue;
			}

			std::cout << "Received Config: " << config << std::endl;
			config = config.substr(0, config.find("("));
			config.append(",");

			while(config.length() > 0)
			{
				std::string::size_type posColon = config.find(":");
				std::string::size_type posComma = config.find(",");
				std::string param = config.substr(0, posColon);
				std::string value = config.substr(posColon+1, (posComma) - (posColon+1));

				std::cout << "Param: " << param << " Value: " << value << std::endl;

				if(param.find("thr") != std::string::npos)
				{
					int v;
					std::istringstream(value) >> v;
					if(param.find("min") != std::string::npos){
						throttle.setMin(v);
					}else if(param.find("max") != std::string::npos)
					{
						throttle.setMax(v);
					}else if(param.find("trim") != std::string::npos)
					{
						throttle.setTrim(v);
					}
				}else if(param.find("roll") != std::string::npos)
				{
					int v;
					std::istringstream(value) >> v;
					if(param.find("min") != std::string::npos){
						roll.setMin(v);
					}else if(param.find("max") != std::string::npos)
					{
						roll.setMax(v);
					}else if(param.find("trim") != std::string::npos)
					{
						roll.setTrim(v);
					}
				}else if(param.find("pitch") != std::string::npos)
				{
					int v;
					std::istringstream(value) >> v;
					if(param.find("min") != std::string::npos){
						pitch.setMin(v);
					}else if(param.find("max") != std::string::npos)
					{
						pitch.setMax(v);
					}else if(param.find("trim") != std::string::npos)
					{
						pitch.setTrim(v);
					}
				}else if(param.find("yaw") != std::string::npos)
				{
					int v;
					std::istringstream(value) >> v;
					if(param.find("min") != std::string::npos){
						yaw.setMin(v);
					}else if(param.find("max") != std::string::npos)
					{
						yaw.setMax(v);
					}else if(param.find("trim") != std::string::npos)
					{
						yaw.setTrim(v);
					}
				}

				config = config.substr(posComma+1);
			}
			std::cerr << "Exiting initial_setup" << std::endl;
			break;
		}
	}

	throttle.setOutput(throttle.getTrim());
	roll.setOutput(roll.getTrim());
	pitch.setOutput(pitch.getTrim());
	yaw.setOutput(yaw.getTrim());

}


// Function runs on new thread as it is blocking
void checkForInput() {
	while (ros::ok()) {
		std::string input;
		std::getline(std::cin, input);
		std::cerr << "Got: " << input << ". Writing to serial...." << std::endl;
		if(input == "initial_start")
		{
			mapGood = false;
			getConfig();
			mapGood = true;
		}else if(input.find("P:") != std::string::npos){
			std::string val = input.substr(input.find("P:")+2);
			pidX.kP(atof(val.c_str()));
			pidY.kP(atof(val.c_str()));
			std::cerr << "P: " << pidX.kP() << std::endl;
		}else if(input.find("I:") != std::string::npos){
			std::string val = input.substr(input.find("I:")+2);
			pidX.kI(atof(val.c_str()));
			pidY.kI(atof(val.c_str()));
			std::cerr << "I: " << pidX.kI() << std::endl;
		}else if(input.find("D:") != std::string::npos){
			std::string val = input.substr(input.find("D:")+2);
			pidX.kD(atof(val.c_str()));
			pidY.kD(atof(val.c_str()));

			std::cerr << "D: " << pidX.kD() << std::endl;
		}else if(input.find("M:") != std::string::npos){
			std::string val = input.substr(input.find("M:")+2);
			pidX.imax(atof(val.c_str()));
			pidY.imax(atof(val.c_str()));

			std::cerr << "IMAX: " << pidX.imax() << std::endl;
		}
		else{
			serialWriteWithCheckSum(input);
		}
	}
}

//you should know what this is
int main(int argc, char** argv){

	ros::init(argc, argv, "quadroter_auto_nav_radio_sim");

	nh = new ros::NodeHandle();

	ros::NodeHandle priv_nh("~");
	std::string port;
  	int baud_rate;
  	int timeout;

	priv_nh.param("port", port, std::string("/dev/ttyAMA0"));
  	priv_nh.param("baud_rate", baud_rate, 57600);
  	priv_nh.param("timeout", timeout, 100);

	//setup serial
	try {
		serialInst.setPort(port);
		serialInst.setBaudrate(baud_rate);
		serial::Timeout to = serial::Timeout::simpleTimeout(timeout);  // 100 ms timeout
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

	//first thing is get the config
	getConfig();
	std::cerr << "Got Config" << std::endl;
	lastTime = ros::Time::now().toNSec();

	while(nh->ok())
	{
		ros::spinOnce(); // needed to get subscribed messages

		uint64_t currentTime = ros::Time::now().toNSec();
		dt = (float)lastTime - currentTime;
		dt /= 1000000000;
		lastTime = currentTime;

		std::string result;

		if (serialInst.available()) {
			try {
				result = serialInst.read(1000); //.read(input, 1000);
			} catch (serial::SerialException& e) {

				// Serial error occurred
				std::cerr << "Error with main loop read" << std::endl;
			}
			std::cerr << result << std::endl;
			if(checkCheckSum(result))
			{
				std::string input = result.substr(0, result.find("("));
				if(input == "landed")
				{
					std::cerr << "Landed resetting..." << std::endl;
					resetX();
					resetY();
				}else{
					std::cerr << "Message without purpose received '" << input << "'" << std::endl;
				}
			}else{
				std::cerr << "Invalid checksum for '" << result << "'" << std::endl;
			}
		}

		if(mapGood)
			std::cout << "currentX: " << currentX << " currentY: " << currentY << " wantedHoldX: " << wantedHoldX << " wantedHoldY: " << wantedHoldY << " of_roll: " << of_roll << " of_pitch: " << of_pitch << std::endl;

		sendPWM();
	}

	writeThread.join();
	
	// Close the serial port
	serialInst.close();
}
