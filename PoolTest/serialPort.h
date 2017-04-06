#ifndef _SERPORT_H
#define _SERPORT_H

#include <sys/poll.h>
#include <stdint.h>

class serialPort {
	private:
		//HANDLE handle;
		//char errMsg[ERR_SIZE];
		int fd;
		char* portName;

	public:
		serialPort();
		uint8_t openPort(char* port);
		void closePort(void);
		uint8_t configure(int32_t baudrate,char* ser_port);
		int readPort(char *buffer, uint32_t max_bytes);

		int writePort(char *buffer, uint32_t bytes);
		struct pollfd fds[1];
		void flushIn(void);
		void drainOut(void);
		//virtual ~serialPort();
};
#endif