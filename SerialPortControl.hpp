/*
//First edition by old000: C.H.Hung , NTU,EE97,master student
//Second edition by kenji: Y.C.Lin , NTU,EE97,master student
//          Idea from code by W.L.Hsu, NTU,EE98,PHD student
*/
#ifndef _SERIALPORTCONTROL_HPP
#define _SERIALPORTCONTROL_HPP

#include <cstring>
#include <string>
#include <iostream>
#include <windows.h>

using namespace std;

class SerialPortControl
{
public:
	SerialPortControl(string port_name, int baud_rate)
	{
		openPort(port_name, baud_rate);
	}
	int readLeftEncoder() //Left wheel node 0
	{
		writePort(string("0pos\n")); 
		//cout << "Wait: readLeftEncoder " << endl;
		Sleep(5);
		return atoi(readPort().c_str());
	}
	int readRightEncoder() //Right wheel node 1
	{
		writePort(string("1pos\n"));
		//cout << "Wait: readRightEncoder " << endl;
		Sleep(5);
		return atoi(readPort().c_str());
	}
public:
	bool openPort(string port_name, int baud_rate);
	bool writePort(string str);
	string readPort();
protected:
	HANDLE  _hComm;
};

#endif