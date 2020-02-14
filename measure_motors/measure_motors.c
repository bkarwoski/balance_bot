/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
* TODO: Option A: Capture encoder readings, current readings, timestamps etc. 
*       to a file to analyze and determine motor parameters
*       
*       Option B: Capture the same information within get_motor_params and follow
*       on its structure for obtaining the parameters and printing them in your
*       terminal.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

FILE* f1;

/*******************************************************************************
* int main() 
*
*******************************************************************************/

int get_motor_params(int motor, int polarity, float resistance, float dtime_s, FILE *outFile);

int main(int argc, char *argv[]) {
    
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

//    if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
//        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
//        return -1;
//    }
	
	if (argc != 2) {
		fprintf(stderr, "Error: first argument must be filename\n");
		return -1;
	}

    // initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();


    rc_set_state(RUNNING);

    /**********************************************************************
    [OPTION A] TODO : Loop and Save data to a file and migrate that info
                      to python or matlab
    
    while(rc_get_state()!=EXITING){
        rc_nanosleep(1E9);
        //get data
        //save to file
    }
    // close file    
    **********************************************************************/
    
    /**********************************************************************
    [OPTION B] TODO : Follow on the guide within get_motor_params and 
                      construct it accordingly. Then run it for each motor
                      given you know its resistance. */

    int pass_mot1, pass_mot2;
    float dtime_s = 5;  // 5sec is usuall enough but you can change
	FILE *outFile = fopen(argv[1], "w");
    mb_motor_init();
	printf("RIGHT MOTOR\n");
    pass_mot1 = get_motor_params(RIGHT_MOTOR, ENC_1_POL, 9.2, dtime_s, outFile);
	printf("LEFT MOTOR\n");
    pass_mot2 = get_motor_params(LEFT_MOTOR, ENC_2_POL, 9.2, dtime_s, outFile);
	fclose(outFile);

    // exit cleanly
    rc_adc_cleanup();
    rc_encoder_eqep_cleanup();
    rc_remove_pid_file();   // remove pid file LAST
	if (pass_mot1 && pass_mot2) {
		return 0;
	}
	return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// [OPTION B] TODO : Fill in get_motor_params to obtain motor parameters

int get_motor_params(int motor, int polarity, float resistance, float dtime_s, FILE *outFile){

    // Parameters to feed in:
    /***************************************************************************************************************
    > motor : Motor number referring to right or left motor
    > polarity : Encoder polarity, you can remove this as an argument and just use the global variable from the defs
    > resistance : The resistance value you measured off the motors as we will use this in the calculations
    > dtime_s : The time to complete the transient measurements
    ***************************************************************************************************************/

    // Pass flag you can manipulate to return success or fail of motor measurement
    int pass_flag = -1;
    
    // Variable defs;

    float rads_end, speed, noload_speed = 0, mot_constant = 0, stall_torque, visc_fric, shaft_inertia, coul_fric;
	float meas_speed[2], thisk, meas_curr[2];
    double noload_current = 0;
    double dt, start_time, time_elapse, prevtime, time_const;
    //int got_time_const = 0;
	double batt_volt = 10.8;

    rc_encoder_write(motor, 0);             // Reset the enocder

    // First run for steady state data and obtain all info attainable from that.
	int numloops = 0;
	for (float duty = 0.5; duty < 1; duty += .49) {

		mb_motor_set(motor, duty);              // Command the duty cycle we provided
		rc_nanosleep(5E8); //wait to get to steady state
		rc_encoder_write(motor, 0);
		uint64_t nanosec_ss = 5E9;
		int numCurrLoops;
		meas_curr[numloops] = 0;
		for (numCurrLoops = 0; numCurrLoops < nanosec_ss/1E9; numCurrLoops++) {
			rc_nanosleep(1E9); // Sleep for 1s
			float newCurr = mb_motor_read_current(motor);    // Read from the analog pin to get the no-load current
			printf("Current after %d s: %3.4f\n", numCurrLoops+1, newCurr);
			meas_curr[numloops] += newCurr;
		}
		meas_curr[numloops] /= numCurrLoops; //take the average of the measurements
		rads_end = rc_encoder_read(motor) / (ENCODER_RES * GEAR_RATIO) * polarity * 2 * M_PI;
		meas_speed[numloops] = rads_end / (nanosec_ss / (1E9)); //rad/s
		thisk = (duty * batt_volt - resistance * meas_curr[numloops]) / meas_speed[numloops];
		printf("At %2f duty cycle:\n\tmot constant: %f speed: %f curr: %f\n", duty, thisk, meas_speed[numloops], meas_curr[numloops]);
		//add up averages
		noload_speed = meas_speed[numloops];
		mot_constant += thisk;
		noload_current = meas_curr[numloops];
		numloops++;
	}
	mot_constant /= numloops;
	
    stall_torque = mot_constant * batt_volt / resistance; // max voltage
    visc_fric = mot_constant * (meas_curr[1] - meas_curr[0]) / (meas_speed[1] - meas_speed[0]);
	coul_fric = mot_constant * meas_curr[1] - visc_fric * meas_speed[1];
	//print everything
	printf("[ No Load Speed (rad/s) : %3.4f, No Load Current (A) : %3.4lf,  Stall Torque (N*m) : %3.4f ]\n", noload_speed, noload_current, stall_torque);
    printf("[ Motor Constant K (V*s) : %3.4f, Viscous Friction (kg*m^2/s) : %3.4e,  Coulomb friction (N*m) : %3.4f ]\n", mot_constant, visc_fric, coul_fric);
    /////////////////////////////////////////////////// steady state calcs are done
    mb_motor_set(motor, .99);
    rc_nanosleep(1E9);
	printf("Calculating Moment of inertia\n");
	fprintf(outFile, "Motor %d\n\ntime (s),pos (rad)\n", motor);

    // We need to time the transient run now in order to obtain the time constant.
    // We will keep monitoring our spin speed in loops and once it exceeds or matches 63%
    // of our no load speed calculated above, we record the time as the time constant.

    start_time = (double)(rc_nanos_since_epoch())*1.0E-9;
    //prevtime = 0.0;                         // Prev loop time, needed to get dt

    // Our while loop termination condition is the max run time dtime_s we provide as an argument to the function
    mb_motor_set(motor, 0); //turn off the motor, now current is 0 because it's coast
	rc_encoder_write(motor, 0);
	time_elapse = 0.0;                      // Current loop time 
	//speed = 10; //initialize somehow
    while(time_elapse < 1) {
        //prevtime = time_elapse;
        time_elapse = (double)(rc_nanos_since_epoch())*1.0E-9 - start_time;
        //dt = time_elapse - prevtime;
		//static double prev_rad = 0;
        double now_rad = rc_encoder_read(motor) / (ENCODER_RES * GEAR_RATIO) * polarity * 2 * M_PI;
        //speed =  (now_rad - prev_rad) / dt;
		//prev_rad = now_rad;
		fprintf(outFile, "%3.4f, %3.4f\n", time_elapse, now_rad);
		printf("%3.4f, %3.4f\n", time_elapse, now_rad);
        /*if(!got_time_const && speed > (0.63 * noload_speed) ){
            printf("Got time constant\n");
            got_time_const = 1;
            time_const = start_time - time_elapse;
        } */
        rc_nanosleep(1E7);
    }
	fprintf(outFile, "\n\n");

    // shaft_inertia = (time_const * mot_constant * mot_constant) / 9.2;

    ///////////////////////////////////////////////////////////// Finally, print shaft inertia
    //printf("Shaft inertia (kg*m^2) : %1.4e\n\n", shaft_inertia);

    pass_flag = 1;

    return pass_flag;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////