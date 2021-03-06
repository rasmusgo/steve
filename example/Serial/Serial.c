//##########################################################
//##                      R O B O T I S                   ##
//## CM-700 (Atmega2561) Example code for Serial Comm.    ##
//##                                           2009.11.10 ##
//##########################################################


#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "serial.h"

int main(void)
{
	int Value = 0;

	serial_initialize(57600);				// USART Initialize
	sei();

	printf( "\n\nSerial Comm. example for CM-700\n\n" );

	while (1)
	{
		unsigned char ReceivedData = getchar();
		if(ReceivedData == 'u')
			Value++;
		else if(ReceivedData == 'd')
			Value--;

		printf("%d\r\n", Value);
	}
	return 1;
}
