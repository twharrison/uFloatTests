#ifndef _xbee_control_H
#define _xbee_control_H

#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define XBEE_DEVICE "/dev/ttyS4" //Beaglebone Black serial port
//#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define BAUDRATE_XBEE B115200   // Change as needed, keep B


class Xbee_control {
   public:


	  Xbee_control(void);
	  void begin(void);
	  int8_t send_msg_to_base(int port, uint8_t msg[], uint8_t m_size);
	  uint8_t crc_calc(uint8_t header[],uint8_t msg[], int h_size, int m_size);

	  int xbee_fd;
   
   private:

};
#endif