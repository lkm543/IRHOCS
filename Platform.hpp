/*
//First edition by old000: C.H.Hung , NTU,EE97,master student
//Second edition Modified by kenji : Y.C.Lin , NTU,EE97,master student in 2010.04
//Third edition Modified by W.L.Hsu ,NTU,EE98,PhD student
*/
#ifndef _PLATFORM_HPP
#define _PLATFORM_HPP

#include <math.h>
#include <process.h>
#include "SerialPortControl.hpp"
#include "TimerControl.h"

#define PI               3.1415926

//VSIR Platform parameter
#define NODE_FRONTRIGHTMOTOR     0
#define NODE_FRONTLEFTMOTOR      1
#define NODE_REARRIGHTMOTOR      2
#define NODE_REARLEFTMOTOR       3
#define MOTOR_PORT_FRONT         3  // Serial port
#define MOTOR_PORT_REAR          5  // Serial port
#define FRONTRIGHTMOTOR_RATIO 1.0
#define FRONTLEFTMOTOR_RATIO  1.0
#define REARRIGHTMOTOR_RATIO  1.0
#define REARLEFTMOTOR_RATIO   1.0
#define MOTOR_BAUDRATE		  9600
#define POSITION_RATIO       84000
#define PULSE_RATIO          14336  // 2048*66 = 135168 pulses for 1 circle 1024*14
#define MAXIMUM_SPEED         8000  // RPM
#define MOTION_SPEED           300  // RPM
#define MOTION_ACCELERATION    200  // circle/s^2
#define MOTION_DECELERATION    200  // circle/s^2
#define WHEEL_DIAMETER          15  // cm
#define WHEEL_DISTANCE          45  // cm
#define GEAR_RATIO              14  // 2:1 GEAR
#define rear_RATIO              2 // ¥Ö±a½ü¤ñ 
//Probability parameter 
#define FORWARD_FACTOR       1.010  // compensate for slippage problem
#define TURN_FACTOR          0.977  // compensate for slippage problem 
#define REC_FORWARD_FACTOR   1.330  // prediction about when sampling time set 50ms, the code run time need more 15% time
#define REC_TURN_FACTOR      1.350
#define COMMAND_TIME_DELAY     300

//Enviroment parameter
#define GRID_LENGTH             61  // cm



class Platform
{
public:

	Platform(int motor_port_front, int motor_port_rear, int baud_rate);
	//motion
	void moveForward(double dist_cm);
	void moveForward(void);
	void moveForwardD(double dist_cm);
	void moveBackward(void);
	void moveRight(double dist_cm);
	void moveRight(void);
	void moveLeft(double dist_cm);
	void moveLeft(void);
	void moveLeftN(double dist_cm, double ratio);
	void moveLeftN(double ratio);
	void turnDirection(double angle_degree);        //positive for counter clockwise direction
	void turnLeft(void);
	void turnRight(void);
	void biasControl(int left_motor_vel, int right_motor_vel); // differencial velocity control
	void stop();
	void enableMotor();
	void disableMotor();
	

	//Ricky Test Function
	void test(double dist_cm);
	void disableTest();

	//left  encoder 0
	//right encoder 1
	int readEncoder(int which_encoder);

	//encoder detect
	void watchMotion();
	static void thrWatchMotion(void*);

	//set motor parameter
	void resetMotorEncoder();
	void setSpeed(int speed_rpm);
	void setAcceleration(int acc);
	void setDeceleration(int dec);
	
	//check state
	bool isIdle(){return _isIdle;}
	int stateBack(){return _motionState;}

	int vL;
	int vR;
	int speed;
	int acceleration;
	int deceleration;
	

	//protected:

	int  _motionState; //0 for stop, 1 for straight , 2 for left, 3 for right,
	bool _isIdle;
	bool _isMotionWatching;
	SerialPortControl* _motorControlFront;
	SerialPortControl* _motorControlRear;
};


#endif