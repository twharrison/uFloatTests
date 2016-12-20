#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "Fona_control.h"

Fona_control fona;

int main(void) 
{
	int count = 0;
	while(count < 10)
	{
		count++;
		fona.get_gps();
		if(count == 5)
		{
			std::cout << "SENDING!!!" << std::endl;
			fona.sms_location_send();
		}
		sleep(2);
	
	}
	return 0;
	
}