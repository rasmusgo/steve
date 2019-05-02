#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>

#include "serial.h"
#include "dynamixel.h"

#define PI	3.141592f

/// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33

// Default setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define NUM_ACTUATOR		4 // Number of actuator
#define STEP_THETA			(PI / 100.0f) // Large value is more fast
#define CONTROL_PERIOD		(10) // msec (Large value is more slow)

#define POSITION_CENTER		512
#define POSITION_LEFT_45	358
#define POSITION_RIGHT_45	666

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

int main(void)
{
	int id[NUM_ACTUATOR];
	float offsets[NUM_ACTUATOR];
	float theta = 0;
    float degrees_to_position_units = 1023.0f / 300.0f;
	int GoalPos;
	int i;
	int CommStatus;
    unsigned char ReceivedData;

	serial_initialize(57600);
	dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
	sei();	// Interrupt Enable

	printf( "\n\nSteve the servo controller\n\n" );

	for( i=0; i<NUM_ACTUATOR; i++ )
	{
		id[i] = 10 + 10 * i;
		offsets[i] = 0.0f;
	}

	// Set goal speed
	dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );
	// Set goal position
	dxl_write_word( BROADCAST_ID, P_GOAL_POSITION_L, POSITION_CENTER );
	_delay_ms(1000);

	while(1)
	{
        ReceivedData = getchar();
        if(ReceivedData == 'w')
        {
			printf( "Forward\n" );
    		for( i=0; i<NUM_ACTUATOR; i++ )
            {
                offsets[i] = 0.0f;
            }
        }
        else if(ReceivedData == 'a')
        {
			printf( "Left\n" );
    		for( i=0; i<NUM_ACTUATOR; i++ )
            {
                offsets[i] = (i == 0 || i == 3) ? -45.0f : 45.0f;
            }
        }

		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
		dxl_set_txpacket_parameter(1, 2);
		for( i=0; i<NUM_ACTUATOR; i++ )
		{
			dxl_set_txpacket_parameter(2+3*i, id[i]);
			GoalPos = POSITION_CENTER + (int)(offsets[i] * degrees_to_position_units);
			printf( "%d  ", GoalPos );
			dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos));
			dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos));
		}
		dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);

		printf( "\n" );

		dxl_txrx_packet();
		CommStatus = dxl_get_result();
		if( CommStatus == COMM_RXSUCCESS )
			PrintErrorCode();
		else
			PrintCommStatus(CommStatus);

		theta += STEP_THETA;

		if( theta > 2*PI )
			theta -= 2*PI;
		_delay_ms(CONTROL_PERIOD);
	}

	return 0;
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}
