#include "Platform.hpp"
#include <string>

using namespace std;

Platform::Platform(int port_num_front, int port_num_rear, int baud_rate = 9600)
{
	char motor_port_name_front[10];
	char motor_port_name_rear[10];
	sprintf_s(motor_port_name_front, "COM%d", port_num_front);
	sprintf_s(motor_port_name_rear,  "COM%d", port_num_rear);
	_motorControlFront = new SerialPortControl(string(motor_port_name_front), baud_rate);
	_motorControlRear = new SerialPortControl(string(motor_port_name_rear), baud_rate);
	_isIdle = true;
	_motionState = 0;
	_motorControlFront->writePort(string("2ANSW0\n")); //YC Idea 同步是什麼意思
	_motorControlFront->writePort(string("1ANSW0\n"));

	resetMotorEncoder();
	setSpeed(MOTION_SPEED);
	setAcceleration(MOTION_ACCELERATION);
	setDeceleration(MOTION_DECELERATION);


	stop();
}



void Platform::enableMotor()
{
	_isIdle = true;
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];
	sprintf_s(FrontMotorCmdBuffer, "0en\n1en\n");
	sprintf_s(RearMotorCmdBuffer, "2en\n3en\n");
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
}

void Platform::disableMotor()
{
	_isIdle = true;
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];
	sprintf_s(FrontMotorCmdBuffer, "0di\n1di\n");
	sprintf_s(RearMotorCmdBuffer, "2di\n3di\n");
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
}

void Platform::moveForward(double dist_cm)
{
	stop();
	if(dist_cm >= 0)
		moveForward();
	else
		moveBackward();

	Sleep(FORWARD_FACTOR*fabs(dist_cm)*(84000*20)/(PI*WHEEL_DIAMETER*speed));
	stop();
}

void Platform::moveForward(void)
{
	_isIdle = false;
	_motionState = 1;
	enableMotor();
	Sleep(20);
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];

	sprintf_s(FrontMotorCmdBuffer, "0v%f\n1v%f\n", speed*1.0, (-1)*speed*1.0);
	sprintf_s(RearMotorCmdBuffer, "2v%f\n3v%f\n", speed*1.0, (-1)*speed*1.0);
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
	vL = speed;
	vR = speed;
}

void Platform::moveBackward(void)
{
	_isIdle = false;
	_motionState = 2;
	enableMotor();
	Sleep(20);
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];
	sprintf_s(FrontMotorCmdBuffer, "0v%f\n1v%f\n", (-1)*speed*FRONTRIGHTMOTOR_RATIO, speed*FRONTLEFTMOTOR_RATIO);
	sprintf_s(RearMotorCmdBuffer, "2v%f\n3v%f\n", (-1)*speed*REARRIGHTMOTOR_RATIO, speed*REARLEFTMOTOR_RATIO);
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
	vL = (-1)*speed;
	vR = (-1)*speed;
}

void Platform::moveRight(double dist_cm)
{
	stop();
	enableMotor();
	Sleep(20);
	if(dist_cm >= 0)
		moveRight();
	
		//Sleep(FORWARD_FACTOR*fabs(dist_cm)*(60000*GEAR_RATIO)/(PI*WHEEL_DIAMETER*speed));
	Sleep(FORWARD_FACTOR*fabs(dist_cm)*(84000*20)/(PI*WHEEL_DIAMETER*speed));
	stop();
}

void Platform::moveRight(void)
{
	_isIdle = false;
	//_motionState = 2;
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];
	sprintf_s(FrontMotorCmdBuffer, "0v%f\n1v%f\n", (-1)*speed*1.0, (-1)*speed*1.0);
	sprintf_s(RearMotorCmdBuffer, "2v%f\n3v%f\n", speed*1.0, speed*1.0);
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
	//vL = (-1)*speed;
	//vR = (-1)*speed;
}

void Platform::moveLeft(double dist_cm)
{
	stop();
	enableMotor();
	//Sleep(20);
	if(dist_cm >= 0)
		moveLeft();
	//Sleep(FORWARD_FACTOR*fabs(dist_cm)*(60000*GEAR_RATIO)/(PI*WHEEL_DIAMETER*speed));
	Sleep(FORWARD_FACTOR*fabs(dist_cm)*(84000*20)/(PI*WHEEL_DIAMETER*speed));
	stop();
}

void Platform::moveLeft(void)
{
	_isIdle = false;
	//_motionState = 2;
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];

	sprintf_s(FrontMotorCmdBuffer, "0v%f\n1v%f\n", speed*1.0, speed*1.0);
	sprintf_s(RearMotorCmdBuffer, "2v%f\n3v%f\n", (-1)*speed*1.0, (-1)*speed*1.0);
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
	//vL = (-1)*speed;
	//vR = (-1)*speed;
}
void Platform::moveLeftN(double dist_cm, double ratio)
{
	stop();
	enableMotor();
	//Sleep(20);
	if(dist_cm >= 0)
		moveLeftN(ratio);
	//Sleep(FORWARD_FACTOR*fabs(dist_cm)*(60000*GEAR_RATIO)/(PI*WHEEL_DIAMETER*speed));
	Sleep(FORWARD_FACTOR*fabs(dist_cm)*(84000*20)/(PI*WHEEL_DIAMETER*speed));
	stop();
}

void Platform::moveLeftN(double ratio)
{
	_isIdle = false;
	//_motionState = 2;
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];

	sprintf_s(FrontMotorCmdBuffer, "0v%f\n1v%f\n", speed*1.0, speed*1.0);
	sprintf_s(RearMotorCmdBuffer, "2v%f\n3v%f\n", (-1)*speed*ratio, (-1)*speed*ratio);
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
	//vL = (-1)*speed;
	//vR = (-1)*speed;
}
void Platform::turnDirection(double angle_degree)
{
	stop();
	enableMotor();
	if(angle_degree > 180)
		while(angle_degree > 0)
			angle_degree -= 360;
	if(angle_degree < -180)
		while(angle_degree < 0)
			angle_degree += 360;

	if(angle_degree >= 0)
		turnLeft();
	else
		turnRight();
	
	Sleep(TURN_FACTOR*fabs(angle_degree)*(WHEEL_DISTANCE*PI/360)*(84000*38)/(PI*WHEEL_DIAMETER*speed));
	stop();
}


void Platform::turnLeft(void)
{
	_isIdle = false;
	_motionState = 3;
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];
	sprintf_s(FrontMotorCmdBuffer, "0v%f\n1v%f\n", speed*FRONTRIGHTMOTOR_RATIO, speed*FRONTLEFTMOTOR_RATIO);
	sprintf_s(RearMotorCmdBuffer, "2v%f\n3v%f\n",speed*REARRIGHTMOTOR_RATIO, speed*REARLEFTMOTOR_RATIO);
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
	Sleep(5);
	vL = speed;
	vR = speed;
}

void Platform::turnRight(void)
{
	_isIdle = false;
	_motionState = 4;
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];
	sprintf_s(FrontMotorCmdBuffer, "0v%f\n1v%f\n", (-1)*speed*FRONTRIGHTMOTOR_RATIO, (-1)*speed*FRONTLEFTMOTOR_RATIO);
	sprintf_s(RearMotorCmdBuffer, "2v%f\n3v%f\n", (-1)*speed*REARRIGHTMOTOR_RATIO, (-1)*speed*REARLEFTMOTOR_RATIO);
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
	Sleep(5);
	vL = (-1)*speed;
	vR = (-1)*speed;
}

void Platform::disableTest()
{
	_isIdle = true;
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];
	sprintf_s(FrontMotorCmdBuffer, "0di\n");
	sprintf_s(RearMotorCmdBuffer, "2di\n");
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
}

void Platform::test(double dist_cm)
{
	char FrontMotorCmdBuffer[32];
	char RearMotorCmdBuffer[32];
	enableMotor();
	//disableTest();
	sprintf_s(FrontMotorCmdBuffer, "1v%d\n", (-1)*speed);
	sprintf_s(RearMotorCmdBuffer, "3v%d\n", (-1)*speed);
	_motorControlFront->writePort(string(FrontMotorCmdBuffer));
	_motorControlRear->writePort(string(RearMotorCmdBuffer));
	Sleep(FORWARD_FACTOR*(60000*dist_cm*GEAR_RATIO)/(PI*WHEEL_DIAMETER*speed));
	stop();
}

void Platform::biasControl(int left_motor_vel, int right_motor_vel)
{
	_isIdle = false;
	int LeftMotorVel  = (-1)*left_motor_vel;  // left  node0
	int RightMototVel =      right_motor_vel; // right node1 

	char LeftMotorCmdBuffer[16];
	char RightMotorCmdBuffer[16];

	sprintf_s(LeftMotorCmdBuffer, "2v%d\n", LeftMotorVel); //nLStr = num of characters 
	sprintf_s(RightMotorCmdBuffer, "1v%d\n", RightMototVel);
	
	_motorControlFront->writePort(string(LeftMotorCmdBuffer));
	_motorControlFront->writePort(string(RightMotorCmdBuffer));
	Sleep(5);
	vL = left_motor_vel;
	vR = right_motor_vel;
}

void Platform::stop()
{
	_isIdle = true;
	//_isMotionWatching = false;
	_motionState = 0;
	_motorControlFront->writePort(string("0v0\n1v0\n"));
	_motorControlRear->writePort(string("2v0\n3v0\n"));
	Sleep(5);
}

void Platform::watchMotion()
{
	_isMotionWatching = true;
	//_beginthread(Platform::thrWatchMotion, 0, this);
}

void Platform::thrWatchMotion(void* p_platform)
{
	Platform* platform = (Platform*)p_platform;     
	int prev_left_encoder_value, prev_right_encoder_value, 
		next_left_encoder_value, next_right_encoder_value;
	prev_left_encoder_value = platform->readEncoder(0);
	prev_right_encoder_value = platform->readEncoder(1);

	int stop_count = 0;
	
	while(platform->_isMotionWatching){
		// 1. get encoder value
		// 2. compare prev_enc_val and next_enc_val
		// 3. find out whether platform is moving
		//cout << "Wait: thrWatchMotion " << endl;
		Sleep(5); // wait for motor rotating

		next_left_encoder_value = platform->readEncoder(2);
		next_right_encoder_value = platform->readEncoder(1);
		
		if(	abs(next_left_encoder_value - prev_left_encoder_value) < 300 && 
			abs(next_right_encoder_value - prev_right_encoder_value) < 300){
			
			stop_count++;
		}else
		{
			stop_count = 0;
		}

		if(stop_count >= 2){
			platform->_isIdle = true;
			platform->_isMotionWatching = false;
			platform->_motionState=0;
		}

		prev_left_encoder_value = next_left_encoder_value;
		prev_right_encoder_value = next_right_encoder_value;
	}
	_endthread();
}

int Platform::readEncoder(int which_encoder) //Left wheel node 0
{
	char encoderChar[16];
	sprintf_s(encoderChar, "%dpos\n", which_encoder);
	_motorControlFront->writePort(string(encoderChar)); 
	//cout << "Wait: readLeftEncoder " << endl;
	Sleep(5);
	return atoi(_motorControlFront->readPort().c_str());
}


//設定追pos的速度
//第一個參數是指定node name
void Platform::setSpeed(int speed_rpm)
{
	char speedChar[10];
	sprintf_s(speedChar, "sp%d\n", speed_rpm);
	speed = speed_rpm;
	_motorControlFront->writePort(string(speedChar));
}

void Platform::setAcceleration(int acc)
{
	char accelerationChar[10];
	sprintf_s(accelerationChar, "ac%d\n", acc);
	acceleration = acc;
	_motorControlFront->writePort(string(accelerationChar));
}

void Platform::setDeceleration(int dec)
{
	char decelerationChar[10];
	sprintf_s(decelerationChar, "dec%d\n", dec);
	deceleration = dec;
	_motorControlFront->writePort(string(decelerationChar));
}

void Platform::resetMotorEncoder()
{
	_motorControlFront->writePort("ho\n");
	//cout << "Wait: resetMotorEncoder " << endl;
	Sleep(5);
}
