/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include "balancebot.h"

typedef struct control_conf {
    float Kpo;
    float Kio;
    float Kdo;
	float INTMAXo;
    float Kpi;
    float Kii;
    float Kdi;
	float INTMAXi;
	float Kph;
    float Kih;
    float Kdh;
	float INTMAXh;
	float theta_bias;
	float deadBand;
	float punch;
} control_conf_t;

int callback_counter;
int params_num = 6;
control_conf_t control_conf_params = {0};
mb_setpoints_t mb_setpoints = {0};
FILE *outFile;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char *argv[]){
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

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
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

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initalize XBee Radio
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
	callback_counter = 0;
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	//printf("initializing controller...\n");
	//mb_controller_init();

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);
    mb_state.cur_target_idx = 1;

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);
	
	if (argc > 1 && strcmp(argv[1], "nomotor") == 0) {
		motorsOn = 0;
	} else {
		motorsOn = 1;
	}

	if (argc > 1 && strcmp(argv[1], "step") == 0) {
		outFile = fopen("step.csv", "w");
	}

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	mb_state.last_heading = 0;
	mb_state.last_pos = 0;
	mb_state.target_x = 0;
	mb_state.target_y = 0;
	
	// exit cleanly
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST
	fclose(outFile);
	return 0;
}

double doInnerLoop(double ref_pitch, double pitch, control_conf_t control_conf_params) {
	double error = ref_pitch - pitch;
	static double last_error = 0;
	double derivative = (error - last_error) / DT;
	last_error = error;
	static double integral = 0;
	integral += error * DT;
	if(integral > control_conf_params.INTMAXi) {
		integral = control_conf_params.INTMAXi;
	} else if (integral < -control_conf_params.INTMAXi) {
		integral = -control_conf_params.INTMAXi;
	}
	mb_state.innerloopintegral = integral;
	double PWMOut = control_conf_params.Kpi*error + control_conf_params.Kdi*derivative + control_conf_params.Kii*integral;
	if (PWMOut > 1) {
		PWMOut = 1;
	} else if (PWMOut < -1) {
		PWMOut = -1;
	}
	return PWMOut;
}

double doOuterLoop(double ref_location, double location, control_conf_t control_conf_params) {
	double error = ref_location - location;
	static double last_error = 0;
	double new_derivative = (error - last_error) / DT;
	static double filt_derivative = 0;
	double alpha = 0.0489; // for the filter , f_cutoff = 5.1 Hz
	filt_derivative = alpha * new_derivative + (1-alpha) * filt_derivative;
	last_error = error;
	//filter the error here
	static double integral = 0;
	integral += error * DT;
	if(integral > control_conf_params.INTMAXo) {
		integral = control_conf_params.INTMAXo;
	} else if (integral < -control_conf_params.INTMAXo) {
		integral = -control_conf_params.INTMAXo;
	}
	mb_state.outerloopintegral = integral;
	double thetaOut = control_conf_params.Kpo*error + control_conf_params.Kdo*filt_derivative + control_conf_params.Kio*integral;
	if (thetaOut > 0.55) {
		thetaOut = 0.55;
	} else if (thetaOut < -0.55) {
		thetaOut = -0.55;
	}
	return thetaOut;
	
}

void heading_controller(double heading_ref, double heading, double *right_cmd, double *left_cmd, control_conf_t control_conf_params) {
	double error = heading_ref - heading;
	if (error > M_PI) {
		error = M_PI - error;
	}
	if (error < -M_PI) {
		error = 2 * M_PI + error;
	}
	mb_state.headingError = error;
	static double last_error = 0;
	double new_derivative = (error - last_error) / DT;
	static double filt_derivative = 0;
	double alpha = 0.0489; // for the filter , f_cutoff = 5.1 Hz
	filt_derivative = alpha * new_derivative + (1-alpha) * filt_derivative;
	last_error = error;
	//filter the error here
	static double integral = 0;
	integral += error * DT;
	if(integral > control_conf_params.INTMAXh) {
		integral = control_conf_params.INTMAXh;
	} else if (integral < -control_conf_params.INTMAXh) {
		integral = -control_conf_params.INTMAXh;
	}
	mb_state.headingintegral = integral;
	double deltaPWM = control_conf_params.Kph*error + control_conf_params.Kdh*filt_derivative + control_conf_params.Kih*integral;
	if (deltaPWM > 1) {
		deltaPWM = 1;
	} else if (deltaPWM < -1) {
		deltaPWM = -1;
	}
	mb_state.headingDeltaPWM = deltaPWM;
	*left_cmd -= deltaPWM / 2;
	*right_cmd += deltaPWM / 2;
	if (*left_cmd < -1) {
		*right_cmd -= (*left_cmd + 1);
		*left_cmd = -1;
	}
	if (*right_cmd < -1) {
		*left_cmd -= (*right_cmd + 1);
		*right_cmd = -1;
	}
	if (*right_cmd > 1) {
		*left_cmd -= (*right_cmd - 1);
		*right_cmd = 1;
	}
	if (*left_cmd > 1) {
		*right_cmd -= (*left_cmd - 1);
		*left_cmd = 1;
	}
}

void deadbandAndPunch(int motor, double *PWM, control_conf_t control_conf_params) {
	if (*PWM > -control_conf_params.deadBand && *PWM < control_conf_params.deadBand) {
		*PWM = 0;
	} else if (PWM > 0) {
		*PWM += control_conf_params.punch;
	} else { //if PWM < 0
		*PWM -= control_conf_params.punch;
	}
}

/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*******************************************************************************/
void balancebot_controller(){
	if(mb_setpoints.reset_ctl){
		mb_state.distance_ref = 0;
		mb_state.heading = 0;
		mb_state.psi_ref = 0;
		mb_state.theta_ref = 0;
		mb_state.headingintegral = 0;
		mb_state.innerloopintegral = 0;
		mb_state.theta = 0;
		mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);
		mb_state.cur_target_idx = 1;

		rc_encoder_eqep_write(1, 0);
		rc_encoder_eqep_write(2, 0);
	}
	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.theta = -mpu_data.dmp_TaitBryan[TB_PITCH_X] + control_conf_params.theta_bias;
	// Read encoders
	callback_counter += 1;
	if (callback_counter % 101 == 0) {
		callback_counter = 1;
		FILE *fptr = fopen("../balancebot/controller_conf.txt", "r");
		if (!fptr) {
        	fprintf(stderr, "Could not open controller file\n");
        	exit(1);
    	}
		fscanf(fptr, "%f %f %f %f %f %f %f %f %f %f %f %f %f", &control_conf_params.Kpo, &control_conf_params.Kio, &control_conf_params.Kdo,
		&control_conf_params.INTMAXo,
		&control_conf_params.Kpi, &control_conf_params.Kii, &control_conf_params.Kdi,
		&control_conf_params.INTMAXi,
		&control_conf_params.Kph, &control_conf_params.Kih, &control_conf_params.Kdh,
		&control_conf_params.INTMAXh,
		&control_conf_params.theta_bias);
		fclose(fptr);
		fptr = fopen("../balancebot/other_conf.txt", "r");
		if (!fptr) {
        	fprintf(stderr, "Could not open other file\n");
        	exit(1);
    	}
		fscanf(fptr, "%f %f", &control_conf_params.deadBand, &control_conf_params.punch);
		fclose(fptr);
	}
    // Update odometry 
	// pos in meters
	double left_pos = get_mot_dist(LEFT_MOTOR);
	double right_pos = get_mot_dist(RIGHT_MOTOR);
	mb_state.left_pos = left_pos;
	mb_state.right_pos = right_pos;
	
	mb_state.phi = (left_pos + right_pos) / 2;
	double heading = (right_pos - left_pos) / WHEEL_BASE;
	while(heading < M_PI){
		heading += 2*M_PI;
	}

	while(heading > M_PI){
		heading -= 2*M_PI;
	}

	mb_state.heading = heading;
	
	mb_odometry_update(&mb_odometry, &mb_state);

    // Calculate controller outputs
	
	//TODO: move this afer setpoints.ctl
	mb_state.theta_ref = doOuterLoop(mb_state.distance_ref, mb_state.phi, control_conf_params);
	// mb_state.theta_ref = theta_ref;
	//fprintf(outFile, "distref,%f,distreal,%f\n", mb_state.distance_ref, mb_state.dist_to_target);

    if(mb_setpoints.ctl == 0){
		//IDLE
		fprintf(outFile,"psiref,%f,psi,%f\n", mb_state.psi_ref, mb_odometry.psi);

   	}
	
    if(mb_setpoints.ctl == 1){
		//STEP RESPONSE
		//getReferences(mb_odometry, &mb_state);
		mb_state.psi_ref = M_PI / 2;
		//record data to CSV
		fprintf(outFile,"psiref,%f,psi,%f\n", mb_state.psi_ref, mb_odometry.psi);
		
   	}

	if(mb_setpoints.ctl == 2){
    	//MANUAL CONTROL
		mb_state.distance_ref += mb_setpoints.fwd_velocity * DT * 0.8;
		mb_state.psi_ref += mb_setpoints.turn_velocity * DT * 0.75;
		while(mb_state.psi_ref < M_PI){
			mb_state.psi_ref += 2*M_PI;
		}

		while(mb_state.psi_ref > M_PI){
			mb_state.psi_ref -= 2*M_PI;
		}
		// printf("DSM controller distance ref= %f, added ref = %f\n", mb_state.distance_ref,  mb_setpoints.fwd_velocity * DT * 0.2);
   	}

	
	double innerLoopPWM = doInnerLoop(mb_state.theta_ref, mb_state.theta, control_conf_params);
	
	mb_state.right_cmd = innerLoopPWM;
	mb_state.left_cmd = innerLoopPWM;
	
	heading_controller(mb_state.psi_ref, heading, &mb_state.right_cmd, &mb_state.left_cmd, control_conf_params);
	
	deadbandAndPunch(RIGHT_MOTOR, &mb_state.right_cmd, control_conf_params);
	deadbandAndPunch(LEFT_MOTOR, &mb_state.left_cmd, control_conf_params);
	
	mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd * motorsOn);
	mb_motor_set(LEFT_MOTOR, mb_state.left_cmd * motorsOn);

	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	
	
   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);

}

/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){
	//mb_setpoints_t *s = (mb_setpoints_t *)ptr;
	while(1){

		if(rc_dsm_is_new_data()){
			if(rc_dsm_ch_normalized(5) == 0){
				mb_setpoints.ctl = 2; //Manual
			} else if(rc_dsm_ch_normalized(5) == 0.5) {
				mb_setpoints.ctl = 1; //Step
			} else if(rc_dsm_ch_normalized(5) == 1){
				mb_setpoints.ctl = 0; //Idle
			} else {
				printf("Uh oh, control mode channel 5 error\n");
				return NULL;
			}
			mb_setpoints.fwd_velocity = rc_dsm_ch_normalized(3);
			mb_setpoints.turn_velocity =rc_dsm_ch_normalized(2);
			if(rc_dsm_ch_normalized(7) >= 0.5){
				mb_setpoints.reset_ctl = 1;
			} else {
				mb_setpoints.reset_ctl = 0;
			}
		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}

/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("  ctl    |");
			printf(" dist_ref|");
			printf("position |");
			printf("  R Pos  |");
			printf("head_err |");
			printf(" heading |");
			printf("    x    |");
			printf("    y    |");
			printf("Target Idx|");
			printf("dist to targ|");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3d  |", mb_setpoints.ctl);
			printf("%7.3f  |", mb_state.distance_ref);
			printf("%7.4f  |", mb_state.phi);
			printf("%7.4f  |", mb_state.right_pos);
			printf("%7.4f  |", mb_state.headingError);
			printf("%7.4f  |", mb_state.heading);
			printf("%7.4f  |", mb_odometry.x);
			printf("%7.4f  |", mb_odometry.y);
			printf("%d     | ", mb_state.cur_target_idx);
			printf("%7.4f  |", mb_state.dist_to_target);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 
