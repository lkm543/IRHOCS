#ifndef _TIMERCONTROL_HPP
#define _TIMERCONTROL_HPP

#include <windows.h>

class TimerControl
{
public:
	TimerControl();

	void activate();
	void deactivate();
	static void thrRun(void*);

	void setElapseTime(int elspse_ms);
	void setCallbackFunc(TIMERPROC lpTimerProc);
	static void CALLBACK TimerProc(HWND hwnd,		// handle to window for timer messages 
									UINT message,	// WM_TIMER message 
									UINT idTimer,	// timer identifier
									DWORD dwTime);	// current system time 
// protected:
	bool _active;

	int _timerID;
	int _elapse_ms;
	TIMERPROC _pCallbackFunc;
};
#endif