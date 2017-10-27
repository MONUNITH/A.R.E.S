#include "serial.h"
#include <windows.h>
#include <stdio.h>

bool terminateThreads = false;


DWORD WINAPI ThreadProc(LPVOID iValue){
	while (!terminateThreads){
	  Serial *ser = (Serial*)iValue;
	  if (ser != NULL) {
        unsigned char buf[32768];
        DWORD dwBytesTransferred;
        ReadFile (ser->hPort, &buf, 32767, &dwBytesTransferred, 0);
        for (int i=0; i < dwBytesTransferred; i++){
          ser->appendRxFifo(buf[i]);
        }
	  }
	  Sleep(5);
	}
    return 0;
}

Serial::Serial(){
  hPort = INVALID_HANDLE_VALUE;
}

Serial::~Serial(){
}

bool Serial::begin(char *port, int baud){
    rx_fifo_start = rx_fifo_end = 0;
	DCB PortDCB;
	COMMTIMEOUTS CommTimeouts;
	hPort = CreateFile (port,  GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if ( hPort != INVALID_HANDLE_VALUE )
	{
		PortDCB.DCBlength = sizeof (DCB);
		// Get the default port setting information.
		GetCommState (hPort, &PortDCB);
		// Change the DCB structure settings.
		PortDCB.BaudRate = baud;              // Current baud
		PortDCB.fBinary = TRUE;               // Binary mode; no EOF check
		PortDCB.fParity = TRUE;               // Enable parity checking
		PortDCB.fOutxCtsFlow = FALSE;         // No CTS output flow control
		PortDCB.fOutxDsrFlow = FALSE;         // No DSR output flow control
		PortDCB.fDtrControl = DTR_CONTROL_ENABLE;  // DTR flow control type
		PortDCB.fDsrSensitivity = FALSE;      // DSR sensitivity
		PortDCB.fTXContinueOnXoff = TRUE;     // XOFF continues Tx
		PortDCB.fOutX = FALSE;                // No XON/XOFF out flow control
		PortDCB.fInX = FALSE;                 // No XON/XOFF in flow control
		PortDCB.fErrorChar = FALSE;           // Disable error replacement
		PortDCB.fNull = FALSE;                // Disable null stripping
		PortDCB.fRtsControl = RTS_CONTROL_ENABLE; // RTS flow control
		PortDCB.fAbortOnError = FALSE;        // Do not abort reads/writes on error
		PortDCB.ByteSize = 8;                 // Number of bits/byte, 4-8
		//if (!strcmp(parity, "e")) PortDCB.Parity = EVENPARITY;
		//else if (!strcmp(parity, "o")) PortDCB.Parity = ODDPARITY;
		PortDCB.Parity = NOPARITY;            // 0-4=no,odd,even,mark,space
		PortDCB.StopBits = ONESTOPBIT;        // 0,1,2 = 1, 1.5, 2
		//Configure the port according to the specifications of the DCB structure.
		if (SetCommState (hPort, &PortDCB))
		{
			// Retrieve the time-out parameters for all read and write operations on the port.
			GetCommTimeouts (hPort, &CommTimeouts);
			// Change the COMMTIMEOUTS structure settings.
			CommTimeouts.ReadIntervalTimeout = MAXDWORD;
			CommTimeouts.ReadTotalTimeoutMultiplier = 0;  // 0
			CommTimeouts.ReadTotalTimeoutConstant = 0;     // 0
			CommTimeouts.WriteTotalTimeoutMultiplier = 10;
			CommTimeouts.WriteTotalTimeoutConstant = 1000;
			// Set the time-out parameters for all read and write operations on the port.
			if (SetCommTimeouts (hPort, &CommTimeouts))
			{
				// Direct the port to perform extended functions SETDTR and SETRTS
				// SETDTR: Sends the DTR (data-terminal-ready) signal.
				// SETRTS: Sends the RTS (request-to-send) signal.
				EscapeCommFunction (hPort, SETDTR);
				EscapeCommFunction (hPort, SETRTS);
				printf("Opened serial port %s\n", port);
				DWORD dwGenericThread;
                hThread = CreateThread(NULL, 0, ThreadProc, (LPVOID)this, 0, &dwGenericThread);
			} else printf("Error setting serial port time-out parameters %s\n", port);
		} else printf("Error configuring serial port %s\n", port);
	} else printf("Error opening serial port %s\n", port);
	return 1;
}


void Serial::end(){
	terminateThreads = true;
	if (hPort != INVALID_HANDLE_VALUE)
	{
		if (!CloseHandle (hPort)) printf("Error closing serial port\n");
			else printf("Closed serial port\n");
	}
}

int Serial::available(){
 return (rx_fifo_start != rx_fifo_end);
}

int Serial::read(){
  if (rx_fifo_start == rx_fifo_end) return 0;
  unsigned char res = rx_fifo[ rx_fifo_end ];
  rx_fifo_end = ( rx_fifo_end + 1) & RX_FIFO_SIZE;
  return res;
}

int Serial::peek(){
  return rx_fifo[ rx_fifo_end ];
}

size_t Serial::write(const uint8_t c){
}

void Serial::appendRxFifo(unsigned char c) {
  if ( ((rx_fifo_start + 1) & RX_FIFO_SIZE) == rx_fifo_end) return;
  rx_fifo[ rx_fifo_start ] = c;
  rx_fifo_start = ( rx_fifo_start + 1) & RX_FIFO_SIZE;
}

/*
int Serial::write(std::string &s){
	//printf("gsWriteSerial\n");
	DWORD dwNumBytesWritten;
	size_t len = 0;
	if (!WriteFile (hPort, s.c_str(), len, &dwNumBytesWritten, NULL))
	{
		printf("error writing serial port\n");
		return 0;
	}
	else return dwNumBytesWritten;
}
int Serial::read(std::string &s){
	char buf[4096] = "";
	DWORD dwBytesTransferred;
	//printf("gsReadSerial\n");
	ReadFile (hPort, &buf, 4095, &dwBytesTransferred, 0);
	//if (dwBytesTransferred > 0) printf("bytes received: %d\n", dwBytesTransferred);
	s += buf;
  return dwBytesTransferred;
}

*/


