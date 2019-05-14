#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>

#include "serial.h"
#include "dynamixel.h"

#define PI    3.141592f

/// Control table address
#define P_CW_ANGLE_LIMIT_L   6
#define P_CCW_ANGLE_LIMIT_L  8
#define P_GOAL_POSITION_L    30
#define P_GOAL_POSITION_H    31
#define P_GOAL_SPEED_L       32
#define P_GOAL_SPEED_H       33

// Default setting
#define DEFAULT_BAUDNUM      1 // 1Mbps
#define NUM_ACTUATOR         4 // Number of actuators
#define STEP_THETA           (PI / 100.0f) // Large value is more fast
#define CONTROL_PERIOD       (10) // msec (Large value is more slow)

#define POSITION_CENTER       512
#define POSITION_LEFT_45      358
#define POSITION_RIGHT_45     666
#define NOMINAL_SPEED         20

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

int main(void)
{
    int ax12a_id[NUM_ACTUATOR];
    int mx12w_id[NUM_ACTUATOR];
    float offsets[NUM_ACTUATOR];
    float speeds[NUM_ACTUATOR];
    float theta = 0;
    float degrees_to_position_units = 1023.0f / 300.0f;
    int GoalPos;
    int GoalSpeed;
    int i;
    int CommStatus;
    unsigned char ReceivedData;

    serial_initialize(57600);
    dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
    sei();    // Interrupt Enable

    printf( "\n\nSteve the servo controller\n\n" );

    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        ax12a_id[i] = 10 + 10 * i;
        mx12w_id[i] = 11 + 10 * i;
        offsets[i] = 0.0f;
        speeds[i] = 0.0f;
    }

    // Set wheel mode for mx12w (run once to store to eeprom)
    /*
    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        dxl_write_word( mx12w_id[i], P_CW_ANGLE_LIMIT_L, 0 );
        dxl_write_word( mx12w_id[i], P_CCW_ANGLE_LIMIT_L, 0 );
    }
    */

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
                speeds[i] = (i < 2) ? -1.0f : +1.0f;
            }
        }
        else if(ReceivedData == 's')
        {
            printf( "Backward\n" );
            for( i=0; i<NUM_ACTUATOR; i++ )
            {
                offsets[i] = 0.0f;
                speeds[i] = (i < 2) ? +1.0f : -1.0f;
            }
        }
        else if(ReceivedData == 'a')
        {
            printf( "Left\n" );
            for( i=0; i<NUM_ACTUATOR; i++ )
            {
                offsets[i] = (i == 0 || i == 3) ? -45.0f : +45.0f;
                speeds[i] = +1.0f;
            }
        }
        else if(ReceivedData == 'd')
        {
            printf( "Right\n" );
            for( i=0; i<NUM_ACTUATOR; i++ )
            {
                offsets[i] = (i == 0 || i == 3) ? -45.0f : +45.0f;
                speeds[i] = -1.0f;
            }
        }
        else if(ReceivedData == 'q')
        {
            printf( "Forward right\n" );
            offsets[0] = +14.32f; speeds[0] = -0.645f;
            offsets[1] = -14.32f; speeds[1] = -0.645f;
            offsets[2] = +9.59f;  speeds[2] = +1.0f;
            offsets[3] = -9.59f;  speeds[3] = +1.0f;
        }
        else if(ReceivedData == 'z')
        {
            printf( "Back right\n" );
            offsets[0] = +14.32f; speeds[0] = +0.645f;
            offsets[1] = -14.32f; speeds[1] = +0.645f;
            offsets[2] = +9.59f;  speeds[2] = -1.0f;
            offsets[3] = -9.59f;  speeds[3] = -1.0f;
        }
        else if(ReceivedData == 'e')
        {
            printf( "Forward left\n" );
            offsets[0] = -9.59f;  speeds[0] = -1.0f;
            offsets[1] = +9.59f;  speeds[1] = -1.0f;
            offsets[2] = -14.32f; speeds[2] = +0.645f;
            offsets[3] = +14.32f; speeds[3] = +0.645f;
        }
        else if(ReceivedData == 'c')
        {
            printf( "Back left\n" );
            offsets[0] = -9.59f;  speeds[0] = +1.0f;
            offsets[1] = +9.59f;  speeds[1] = +1.0f;
            offsets[2] = -14.32f; speeds[2] = -0.645f;
            offsets[3] = +14.32f; speeds[3] = -0.645f;
        }
        else if(ReceivedData == ' ')
        {
            printf( "Stop\n" );
            for( i=0; i<NUM_ACTUATOR; i++ )
            {
                offsets[i] = 0.0f;
                speeds[i] = 0.0f;
            }
        }

        for( i=0; i<NUM_ACTUATOR; i++ )
        {
            GoalPos = POSITION_CENTER + (int)(offsets[i] * degrees_to_position_units);
            printf( "%d  ", GoalPos );
            dxl_write_word( ax12a_id[i], P_GOAL_POSITION_L, GoalPos );
        }
        printf( "\n" );

        for( i=0; i<NUM_ACTUATOR; i++ )
        {
            GoalSpeed = speeds[i] > 0 ?
                1024 + (int)(speeds[i] * NOMINAL_SPEED) :
                (int)(-speeds[i] * NOMINAL_SPEED);
            printf( "%d  ", GoalSpeed );
            dxl_write_word( mx12w_id[i], P_GOAL_SPEED_L, GoalSpeed );
        }
        printf( "\n" );

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
