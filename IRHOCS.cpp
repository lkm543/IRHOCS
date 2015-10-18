#include "LuoLitaArm.h"
#include "Platform.hpp"
#include "gripper_2.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;
#pragma region Version_explain
/*
20150625 Modified on HCW v4
20150901 Modified to ICRA Paper with CP,Tsai
20151015 Modified to IRHOCS
*/
#pragma endregion
#pragma region Turtorial
/*
===Turtorial===
Car:
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
	void setSpeed(int speed_rpm);
	void setAcceleration(int acc);
	void setDeceleration(int dec);
Robot arm move:
	Move_L_Abs(first_block, 0.0f);
	while (MOVL) {
	}
Robot arm:
	setReqSpeed(Value_Gripper_Speed); //gripper speed
	setReqForce(Value_Postion_Gripper_Force); //gripper force
	setDefaultArmSpeed(Value_Arm_Speed);
*/
#pragma endregion
#pragma region opencv
/*
 VideoCapture cap(0);
	Mat img;
    while (true)
    {
        cap >> img;
        Mat image=img;
        imshow("window label", image);
        waitKey(1);
    }
*/
#pragma endregion
// main 
int main( int argc, char **argv, char **envp)
{
#pragma region Variable
	//Variable
	int speed_car = 500;
	int Value_Postion_Gripper_Close=255; //0-255
	int Value_Postion_Gripper_Force=80; //0-255
	int Value_Gripper_Speed=200; //0-255
	float Value_Arm_Speed=0.5; //0-1.2 , float
	double x_offset=0;
	double y_offset=0;
	double z_offset=0;
#pragma endregion
#pragma region default
	Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f first_corner = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f second_corner = Eigen::Matrix4f::Identity();
	Eigen::Matrix2f desk_transform = Eigen::Matrix2f::Identity();
	Eigen::Matrix4f first_block = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f first_block_temp = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f first_target = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f first_target_temp = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f next_block = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f next_block_temp = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f next_target = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f next_target_temp = Eigen::Matrix4f::Identity();
	bool block=false; // In moving mode or not
	bool display_mode=true; //unknown

// function prototype for periodic timer function
	void RTFCNDCL TimerHandler1( void * nContext ); //unknown
	void setDefaultArmSpeed(float percentage = 1.0f);

	Platform* robot_platform = new Platform(MOTOR_PORT_FRONT, MOTOR_PORT_REAR, MOTOR_BAUDRATE);
	robot_platform->setSpeed(speed_car);
	robot_platform->stop();

	//For Gripper
	init_gripper_para(); //unknown
	initialize_gripper_motion(); //unknown
	GoToPosReqFlag = true; //unknown
	setGoToPosReqFlag( GoToPosReqFlag ); //unknown
	setReqSpeed(Value_Gripper_Speed); //gripper speed
	setReqForce(Value_Postion_Gripper_Force); //gripper force

	init_LuoLita_1(); //move to initial position 

	
    // for periodic timer code
    LARGE_INTEGER  liPeriod_1ms;   // timer period ,unknown
    HANDLE         hTimer1;     // timer handle ,unknown

    //  RTX periodic timer code: unknown
    //  TO DO: Set default timer period to your desired time.
    //         The period needs to be an even multiple of the HAL
    //         period found in the control panel.
    //         This example uses a period of 500 micro seconds.

    liPeriod_1ms.QuadPart  = 10000;

	Init_IMPCard();

    // Create a periodic timer
    if (! (hTimer1 = RtCreateTimer(
                                   NULL,            // security
                                   0,               // stack size - 0 uses default
                                   TimerHandler1,    // timer handler
                                   NULL,            // NULL context (argument to handler)
                                   RT_PRIORITY_MAX, // priority
                                   CLOCK_2) ))      // RTX HAL timer
    {
        //
        // TO DO:  exception code here
        // RtWprintf(L"RtCreateTimer error = %d\n",GetLastError());
        ExitProcess(1);
    }

    if (! RtSetTimerRelative( hTimer1, &liPeriod_1ms, &liPeriod_1ms ))
    {
        //RtWprintf(L"RtSetTimerRelative error = %d\n",GetLastError());
		// TO DO: exception code here
        ExitProcess(1);
    }
	init_LuoLita_2(); //unknown

	init_pose.block(0,0,3,3) = R07Cmd;//save initial rotation
	init_pose.block(0,3,3,1) = P07Cmd;//save initial translation
	setDefaultArmSpeed(Value_Arm_Speed);
#pragma endregion
#pragma region default_matrix
	//Block target
	first_block(0, 0) = 0.0;
	first_block(0, 1) = -1.0;
	first_block(0, 2) = 0.0;
	first_block(0, 3) = 0.0;
	first_block(1, 0) = -1.0;
	first_block(1, 1) = -0.0;
	first_block(1, 2) = 0.0;
	first_block(1, 3) = 0.0;
	first_block(2, 0) = -0.0;
	first_block(2, 1) = 0.0;
	first_block(2, 2) = -1.0;
	first_block(2, 3) = -0.0;
	first_block(3, 0) = 0.0;
	first_block(3, 1) = 0.0;
	first_block(3, 2) = 0.0;
	first_block(3, 3) = 1.0;

	first_target(0, 0) = 0.0;
	first_target(0, 1) = -1.0;
	first_target(0, 2) = 0.0;
	first_target(0, 3) = 0.0;
	first_target(1, 0) = -1.0;
	first_target(1, 1) = -0.0;
	first_target(1, 2) = 0.0;
	first_target(1, 3) = 0.0;
	first_target(2, 0) = -0.0;
	first_target(2, 1) = 0.0;
	first_target(2, 2) = -1.0;
	first_target(2, 3) = -0.0;
	first_target(3, 0) = 0.0;
	first_target(3, 1) = 0.0;
	first_target(3, 2) = 0.0;
	first_target(3, 3) = 1.0;

#pragma endregion
	

	kbCmd='m';
	keyboard_cw();
	kbCmd='j';
	keyboard_cw();
	
	VideoCapture cap(0);
	char click='a';
	Mat img;
    while (true)
    {
        cap >> img;
        Mat image=img;
        imshow("window label", image);
        waitKey(1);
		if ( _kbhit() )
				click = _getche();
		if ( click == 'q' )
		{
			break;
		}
    }
	destroyWindow("window label");
	/////////////////////////////////////////
while(1)
	{
			char Cmd=' ';

			if (display_mode==false)
			{
					if ( _kbhit() )
						 Cmd = _getche();
			}

		    system("cls");
            if (display_mode==true)
			{
					if (ModeArm==2) 
					{
						printf("Cartesian Mode    ");
						if (SubMode==2) printf("translation\n"); 
						else if (SubMode==4) printf("rotation\n"); 
						else if (SubMode==0) printf("LOCK\n");
					}
					else if (ModeArm==1) printf("Joint Mode    ");
			
					printf("Input commend and press enter:\n");
					printf("[1] Move to experiment point\n");
					printf("[2] Move\n");
					printf("[z] Go to initial pose\n");
					printf("[q] Quit\n");
					printf(">>");
					std::cin>>Cmd;
			}

		if ( Cmd == 'q' )
		{
			break;
		}
		switch (Cmd)
		{
			case '1': //move to exp point
			{
				ifstream fin2("Point.txt");
				if(!fin2) { 
					cout << "無法讀入檔案\\n"; 
				} 
				for (int i=0;i<=3;i++)
				{
					fin2 >> first_block(i,0) >> first_block(i,1) >> first_block(i,2) >>first_block(i,3);
				}
				first_block(0,3)=first_block(0,3);
				first_block(1,3)=first_block(1,3);
				first_block(2,3)=first_block(2,3);
				cout <<first_block<<endl; 

				setDefaultArmSpeed(Value_Arm_Speed);
				Move_L_Abs(first_block, 0.0f);
				while (MOVL) {
				}
				Sleep(1000);
				break;
			}
			
			case '2': //Move
			{
				robot_platform->moveForward(50);
				robot_platform->stop();
				break;
			}
			case 'z': //回到init位置
				kbCmd = 'j';
				keyboard_cw();
				Move_L_Abs( init_pose ,0.0f);
				while(MOVL) {} //wait for motion to complete
				break;
		}
		Cmd = ' ';		
		Sleep(29);
		if (display_mode==false)
		DisplayLoop(); //display function by laoda
	}

// esc to exit
#pragma region default_exit
	ByeBye();
	while(1)
	{
		if ( _kbhit() )
		{
			 kbCmd = _getch();
			 cout << kbCmd << endl;
			 if ( kbCmd == kb_ESC )
			 {
			     break;
		 	 }
		}
		Sleep(30);
		system("cls");
		DisplayLoop();
	}

	Sleep(100);
	if(!RtDeleteTimer( hTimer1 ) )
	{
        //RtWprintf(L"RtDeleteTimer error = %d\n",GetLastError());
		// TO DO:  your exception code here
		Close_IMPCard();
        ExitProcess(1);
	}
	
	Close_IMPCard();
	Sleep(1000);
	OutputData();
	
	ExitProcess(0);
#pragma endregion
}
// main end
#pragma region default_parameter

void RTFCNDCL TimerHandler1( PVOID context )
{
	ServoLoop();
}

void setDefaultArmSpeed(float percentage)
{
    Ang_Vel_limit = ( 0.01f*100 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Ang_Acc_limit = ((0.01f*100 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Ang_Dec_limit = Ang_Acc_limit * percentage;
    Lin_Vel_limit = ( 0.01f*160 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Lin_Acc_limit = ((0.01f*320 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Lin_Dec_limit = Lin_Acc_limit * percentage;
    Jn_Vel_limit = ( 0.01f*25* deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Jn_Acc_limit = ((0.01f*25* deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Jn_Dec_limit = Jn_Acc_limit * percentage;
}

#pragma endregion