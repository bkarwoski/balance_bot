#ifndef BB_H
#define BB_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> // for isatty()
#include <string.h>
#include <math.h> // for M_PI
#include <signal.h>
#include <pthread.h>
#include <rc/mpu.h>
#include <rc/math/quaternion.h>
#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_motor.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"
#include "../xbee_serial/xbee_receive.h"
#include "../common/mb_path_planning.h"



// global variables
rc_mpu_data_t mpu_data;
pthread_mutex_t state_mutex;
pthread_mutex_t setpoint_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;
int motorsOn;

xbee_packet_t xbeeMsg;
int XBEE_portID;

// functions
void balancebot_controller();

//threads
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);

#endif