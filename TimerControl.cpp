#include "TimerControl.h"
#include <iostream>
#include <process.h>

using namespace std;

TimerControl::TimerControl()
{
	_active = false;
	_elapse_ms = 500;
	_pCallbackFunc = TimerControl::TimerProc;
}



void TimerControl::activate()
{
	_active = true;
	_beginthread(TimerControl::thrRun, 0, this);
}

void TimerControl::deactivate()
{
	_active = false;
}

void TimerControl::thrRun(void* p)
{
	TimerControl* pTC = (TimerControl*)p;
	MSG msg;
	int bRet;
	//SetTimer(NULL, pTC->_timerID, pTC->_elapse_ms, (TIMERPROC)TimerControl::TimerProc);
	pTC->_timerID = SetTimer(NULL, 0, pTC->_elapse_ms, pTC->_pCallbackFunc);
	while(pTC->_active){
		if(bRet = PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)){
			if(bRet != 0){
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
	}
	cout << "Kill Timer!" << endl;
	KillTimer(NULL, pTC->_timerID);
	_endthread();
}
void TimerControl::setElapseTime(int elspse_ms)
{
	_elapse_ms = elspse_ms;
}

void TimerControl::setCallbackFunc(TIMERPROC lpTimerProc)
{
	//SetTimer(NULL, _timerID, _elapse_ms, lpTimerProc);
	_pCallbackFunc = lpTimerProc;
}

void CALLBACK TimerControl::TimerProc(HWND hwnd, UINT message, UINT idTimer, DWORD dwTime){
	cout << "MyTimerProc" << endl;
}

