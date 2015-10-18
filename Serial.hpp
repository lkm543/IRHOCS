// modified by adding sendtoUART function
 

// Serial.h

#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

#include <windows.h>
#include <string>

#define FC_DTRDSR       0x01
#define FC_RTSCTS       0x02
#define FC_XONXOFF      0x04
#define ASCII_BEL       0x07
#define ASCII_BS        0x08
#define ASCII_LF        0x0A
#define ASCII_CR        0x0D
#define ASCII_XON       0x11
#define ASCII_XOFF      0x13

class CSerial
{

public:
	CSerial();
	~CSerial();

	BOOL Open( int nPort , int nBaud );
	BOOL Close( void );

	int ReadData( void *, int );
	int SendData( const BYTE *buffer, int size );
	int SendData( const char *, int );
	int ReadDataWaiting( void );

	BOOL IsOpened( void ){ return( m_bOpened ); }

protected:
	BOOL WriteCommByte( unsigned char );

	HANDLE m_hIDComDev;
	OVERLAPPED m_OverlappedRead, m_OverlappedWrite;
	BOOL m_bOpened;

};

#endif
