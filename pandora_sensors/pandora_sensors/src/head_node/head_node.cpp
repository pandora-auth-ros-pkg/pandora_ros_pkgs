#include <head_node/head_node.h>

Head::Head() : m_Co2Publisher (m_rosNode.advertise<controllers_and_sensors_communications::co2Msg> ("/sensors/co2", 1)),
			m_HeadIrPublisher (m_rosNode.advertise<controllers_and_sensors_communications::headIrMsg> ("/sensors/headIr",1)),
			m_HeadSonarPublisher (m_rosNode.advertise<controllers_and_sensors_communications::headSonarMsg> ("/sensors/headSonar",1)),
			m_MlxPublisher (m_rosNode.advertise<controllers_and_sensors_communications::mlxTempMsg> ("/sensors/mlx",1)),
			m_actionServerHead(m_rosNode, "/sensors/moveHead", boost::bind(&Head::controlHeadCallback, this, _1), false)
{
	m_actionServerHead.start();

	m_GripperControl = m_rosNode.advertiseService("/sensors/gripperControl", &Head::controlGripper, this);

	sonarMeasurements = 0;
}

bool Head::controlHeadCallback(const controllers_and_sensors_communications::controlServoGoalConstPtr &goal){
	ros::Rate r(.5);
	bool success = true;
	float commandInterval;
	unsigned char word[20], degrees;
	int count;
	int nr;
	
	ROS_DEBUG("%d",goal->position);

	if(m_actionServerHead.isPreemptRequested() || !ros::ok()){
		ROS_DEBUG("[head] %s: Preempted", action_name_.c_str());
		// set the action state to preempted
		m_actionServerHead.setPreempted();
		success = false;
	}

	currentTimeStamp = ros::Time::now();
	commandInterval = currentTimeStamp.toSec() - previousTimeStamp.toSec();
	ROS_INFO("[head] Command Interval: %f \n", commandInterval);

	if (commandInterval > 1){
		// write to /dev/head
		// After MOV_SERVO command ALWAYS define angle in degrees

		degrees = goal->position;
		// word = { GRIP_OFF, MOV_SERVO, degrees};
		word[0] = GRIP_OFF;
		word[1] = MOV_SERVO;
		word[2] = degrees;
		count = sizeof(word);

		//~ for (int i = 0; i < 3; i++){
			nr = write(fd, &word, count);
			ROS_DEBUG("%d",nr);
			// sleep 1???
			//~ sleep(1);
		//~ }
		// why sleep ?????
		// and previousTimeStamp = currentTimeStamp??
		// usleep(1000*600);
		previousTimeStamp = currentTimeStamp;
		success = true;
	}
	else
		success = false;

	if(success){
		result_.error = controllers_and_sensors_communications::controlServoResult::OK;
		m_actionServerHead.setSucceeded(result_);
	}
	else{
		result_.error = controllers_and_sensors_communications::controlServoResult::ERROR_COMMUNICATING;
		m_actionServerHead.setAborted(result_);
	}

	return true;
}

bool Head::controlGripper(controllers_and_sensors_communications::gripperControlSrv::Request &req, controllers_and_sensors_communications::gripperControlSrv::Response &res){
	
	int gripperState;
	unsigned char state;
	unsigned char word[20];
	int count;

	switch(req.state){
		case controllers_and_sensors_communications::gripperControlSrv::Request::STATE_OFF:
			gripperState = GRIP_OFF;
			state = 0;
			break;
		case controllers_and_sensors_communications::gripperControlSrv::Request::STATE_OPEN:
			gripperState = GRIP_OPEN;
			state = 1;
			break;
		case controllers_and_sensors_communications::gripperControlSrv::Request::STATE_CLOSE:
			gripperState = GRIP_CLOSE;
			state = 2;
			break;
		default:
			break;
	}

	//word = {gripperState, MOV_SERVO, 90};
	word[0] = gripperState;
	word[1] = MOV_SERVO;
	word[2] = 90;

	res.error = controllers_and_sensors_communications::gripperControlSrv::Response::ERROR_OK;
	return true;

}

int Head::processData(){

	int nr;
	int dint;
	timeval tim;
	float dist, co2, mlx0, mlx1, sonar;
	int timeoutCounter;	

	timeoutCounter = 0;

	count = PACKET_SIZE;
	buf =(unsigned char *)malloc(count*sizeof(char));
	if(buf==NULL)
		ROS_ERROR("MALLOC XAZE!");

	
	fd=open("/dev/head", O_RDWR);
    	if (fd == -1){
    		ROS_ERROR("[Head]: cannot open port");
    		return -1;
    	}
    	else{
    		ROS_INFO("[Head]: usb port successfully opened\n");
    	}

	controllers_and_sensors_communications::headIrMsg HeadIrMsg;
	
	controllers_and_sensors_communications::co2Msg Co2Msg;

	controllers_and_sensors_communications::headSonarMsg HeadSonarMsg;

	controllers_and_sensors_communications::mlxTempMsg MlxMsg;

	while (ros::ok()){

		gettimeofday(&tim, NULL);
		nr = read(fd, buf, count);
		if (nr < 0)
			perror("Failed to read /dev/head\n");
		/* Read up2date variable from bufer */
		up2date = *(unsigned int *) &buf[UPDATE_OFFSET];
		ROS_INFO("up2date: %x ",up2date);

		/* Read IR measurements */
		dint = *(unsigned int *) &buf[IR_OFFSET];
		dist = (float) (dint*5.03/1023);		

		ROS_INFO("IR Data: %f ", dist);

		/* read CO2 measurements */
		co2 = *(float *) &buf[CO2_OFFSET];
		co2 = co2 * 10000;
		ROS_INFO("CO2 Data: %f ", co2);
		
		/* Read sonar measurements */
		sonar = *(float *) &buf[SONAR_OFFSET];
		sonar = (sonar/58.0) ; 
		ROS_INFO("Sonar Data: %f ", sonar);

		HeadSonarMsg.header.stamp = ros::Time::now();
		HeadSonarMsg.distance = sonar;

		/* Read BS2 measurements */
		mlx0 = buf[BS2_OFFSET] + (buf[BS2_OFFSET + 1] / 100);
		mlx1 = buf[BS2_OFFSET + 2] + (buf[BS2_OFFSET + 3] / 100);
		ROS_INFO("MLX Data: %f %f", mlx0, mlx1);
		
		HeadIrMsg.header.stamp = ros::Time::now();
		HeadIrMsg.distance = calculateIr(dist);
    
		Co2Msg.header.stamp = ros::Time::now();
		Co2Msg.ppm = co2;
   
		MlxMsg.header.stamp = ros::Time::now();
		MlxMsg.mlxTemp[0] = mlx0;
		MlxMsg.mlxTemp[1] = mlx1;
		 
		m_HeadIrPublisher.publish(HeadIrMsg);
		m_Co2Publisher.publish(Co2Msg);
		m_MlxPublisher.publish(MlxMsg);
		m_HeadSonarPublisher.publish(HeadSonarMsg);
			
		ros::spinOnce();
    }
    
    return 0;
}


int Head::calculateIr(float voltage){
	
	int result;
	
	if(voltage > 2.10)
		result = 0; //too close
	else if( voltage > 2 && voltage < 2.10)
		result = 5;
	else if( voltage < 2.10 && voltage > 1.98 )
		result = 6;
	else if( voltage < 1.98 && voltage > 1.76)
		result = 7;
	else if( voltage < 1.76 && voltage > 1.6 )
		result = 8;
	else if( voltage < 1.6 && voltage > 1.4)
		result = 9;	
	else if( voltage < 1.4 && voltage > 1.2)
		result = 10;	
	else if( voltage < 1.2 && voltage > 1.1)
		result = 11;	
	else if( voltage < 1.1 && voltage > 1.0)
		result = 12;
	else if( voltage < 1.0 && voltage > 0.9)
		result = 13;	
	else if( voltage < 0.9 && voltage > 0.8)
		result = 14;	
	else if( voltage < 0.8 && voltage > 0.75)
		result = 15;	
	else if( voltage < 0.75 && voltage > 0.7)
		result = 16;	
	else if( voltage < 0.7 && voltage > 0.65)
		result = 17;	
	else if( voltage < 0.65 && voltage > 0.6)
		result = 18;
	else if( voltage < 0.6 && voltage > 0.55)
		result = 19;	
	else if( voltage < 0.55 && voltage > 0.5)
		result = 20;	
	else if( voltage < 0.5 && voltage > 0.48)
		result = 21;	
	else if( voltage < 0.48 && voltage > 0.46)
		result = 22;	
	else if( voltage < 0.46 && voltage > 0.44)
		result = 23;	
	else if( voltage < 0.44 && voltage > 0.42)
		result = 24;	
	else if( voltage < 0.42 && voltage > 0.39)
		result = 25;	
	else if( voltage < 0.39 && voltage > 0.36)
		result = 26;	
	else if( voltage < 0.36 && voltage > 0.34)
		result = 27;	
	else if( voltage < 0.34 && voltage > 0.32)
		result = 28;	
	else if( voltage < 0.32 && voltage > 0.3)
		result = 29;	
	else
		result = 30;	// too far
		
	return result*2;	
	 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "head_node");
	ROS_INFO("[head]: Started service controllerAndSensors::head_node");
	Head head;
	ROS_INFO("[head]: Stopped service controllersAndSensors::head_node");
	head.processData();
	return 0;
}
