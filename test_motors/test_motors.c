/*******************************************************************************
* test_motors.c
*
* 	A basic script to drive the motors to check that the functions in
* 	../common/mb_motor.c are functional. It is also helps for checking that 
* 	the motor defs in ../common/mb_defs.h are behaving as the user desires 
* 	them to be sign wise. It is compiled in ../bin. It's important to run 
* 	this every time you deal with a new motor, whether at lab start 
* 	or due to a motor switch.
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

int signum(double n);
int test_motor_duty(int motor, int polarity, float duty, float dtime_s);
int test_motor_dir(float duty, float dtime_s);

int main(int argc, char** argv){

    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

    // start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }

    if(rc_encoder_init()<0){
        fprintf(stderr,"ERROR: failed to initialze encoders\n");
        return -1;
    }

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    // done initializing so set state to RUNNING
    rc_set_state(RUNNING);
    
    if(rc_get_state()==RUNNING){
        mb_motor_brake(1);

        rc_encoder_write(1, 0);
        rc_encoder_write(2, 0);
       
        printf("Please ensure you always run this script in cases of motor switches\n");
        printf("You will be guaranteed 20.4:1 ratio motors for day1 Balancebot and 34:1 day1 Mbot\n");
        printf("Make sure you note the free spin velocity values and compare to them in case you switch motors\n");
        printf("This will ensure that you don't use a diffrent gear ratio\n");

        rc_nanosleep(3E9);
        printf("Diagonising Motors Operation Capability ...\n\n");

        //run right forward for 2s
        printf("Testing RIGHT FWD\n");
        mb_motor_set(LEFT_MOTOR, 0.0);
        if(test_motor_duty(RIGHT_MOTOR, ENC_1_POL, 0.95, 2.0) < 0){
            fprintf(stderr,"ERROR: Abnormal encoder readings on RIGHT motor. Did the motor spin ?\n");
            return -1;
        }
        printf("Encoder Reads Right FWD\n\n");
        rc_nanosleep(3E9);

        //run left forward for 2s
        printf("Testing LEFT FWD\n");
        if(test_motor_duty(LEFT_MOTOR, ENC_2_POL, 0.95, 2.0) < 0){
            fprintf(stderr,"ERROR: Abnormal encoder readings on LEFT motor. Did the motor spin ?\n");
            return -1;
        }
        printf("Encoder Reads Left FWD\n\n");
        rc_nanosleep(3E9);

        //run left backwards for 2s

        printf("Testing LEFT BKWD\n");
        if(test_motor_duty(LEFT_MOTOR, ENC_2_POL, -0.95, 2.0) < 0){
            fprintf(stderr,"ERROR: Abnormal encoder readings on LEFT motor. Did the motor spin ?\n");
            return -1;
        }
        printf("Encoder Reads Left BKWD\n\n");
        rc_nanosleep(3E9);

        //run right backwards for 2s
        printf("Testing RIGHT BKWD\n");
        if(test_motor_duty(RIGHT_MOTOR, ENC_1_POL, -0.95, 2.0) < 0){
            fprintf(stderr,"ERROR: Abnormal encoder readings on RIGHT motor. Did the motor spin ?\n");
            return -1;
        }
        printf("Encoder Reads Right BKWD\n\n");
        rc_nanosleep(3E9);

        //////////////////////////////////////////////
        // [OPTIONAL] TODO: Incorporate test_motor_dir
        //
        //
        //
        //////////////////////////////////////////////


        
        printf("Motors passed all tests.\n");

        mb_motor_disable();
        rc_nanosleep(1E9);
    }
    
    // exit cleanly
    mb_motor_cleanup();
    rc_remove_pid_file();   // remove pid file LAST
    return 0;
}


int signum(double n) { return (n < 0.0) ? -1 : (n > 0.0) ? +1 : 0; }

int test_motor_duty(int motor, int polarity, float duty, float dtime_s){
    int encoder_ticks;
    float rot_speed;
    rc_encoder_write(motor, 0);
    mb_motor_set(motor, duty);
    rc_nanosleep((int)dtime_s * 1E9);
    encoder_ticks = polarity * rc_encoder_eqep_read(motor);
    rot_speed = encoder_ticks * (2.0*3.14) / (GEAR_RATIO * ENCODER_RES) / dtime_s;
    printf("[ Ticks : %d,  Rot Speed (rad/s) : %3.4f]\n", encoder_ticks, rot_speed);
    mb_motor_set(motor, 0.0);

    if (encoder_ticks < 10.0 && encoder_ticks > -10.0)
    {
        return -1;
    }

    else {
        return 1;
    }
}

int test_motor_brake(int motor, int polarity, float duty, float dtime_s){
    int encoder_ticks;
    float rot_speed;
    rc_encoder_write(motor, 0);
    mb_motor_set(motor, duty);
    rc_nanosleep((int)dtime_s * 1E9);
    encoder_ticks = polarity * rc_encoder_eqep_read(motor);
    rot_speed = encoder_ticks * (2.0*3.14) / (GEAR_RATIO * ENCODER_RES) / dtime_s;
    printf("[ Ticks : %d,  Rot Speed (rad/s) : %3.4f]\n", encoder_ticks, rot_speed);
    mb_motor_set(motor, 0.0);

    if (encoder_ticks < 10.0 && encoder_ticks > -10.0)
    {
        return -1;
    }

    else {
        return 1;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// [OPTIONAL] TODO : Incorporate this function in the test motors
//
//
// Function to test direction that you could add to the script. It will test that the signs of the obtained speeds
// given the specified polarities in mb_defs.h match.
int test_motor_dir(float duty, float dtime_s){

    int left_encoder, right_encoder;
    float left_rot_speed, right_rot_speed;
    rc_encoder_write(1, 0);
    rc_encoder_write(2, 0);
    mb_motor_set_all(duty);
    rc_nanosleep((int)(dtime_s * 1E9));
    left_encoder = ENC_2_POL * rc_encoder_read(LEFT_MOTOR);
    right_encoder = ENC_1_POL * rc_encoder_read(RIGHT_MOTOR);
    left_rot_speed = left_encoder *(2*M_PI) / (GEAR_RATIO * ENCODER_RES) / dtime_s;
    right_rot_speed = right_encoder *(2*M_PI) / (GEAR_RATIO * ENCODER_RES)/ dtime_s;
    printf("[ Left Rot Speed (rad/s) : %3.4f, Right Rot Speed (rad/s) : %3.4f ]\n", left_rot_speed, right_rot_speed);
    mb_motor_set_all(0.0);

    if (signum(left_rot_speed) != signum(right_rot_speed))
    {
        return -1;
    }

    else {
        return 1;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////