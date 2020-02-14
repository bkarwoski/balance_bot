#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float  theta;         // body angle (rad)
    float  phi;           // average wheel angle (rad)
	double heading;
	double last_heading;
    double left_pos;      // left encoder counts since last reading
    double right_pos;     // right encoder counts since last reading
	double last_pos;
	
	//calcs 
	float innerloopintegral;
	float outerloopintegral;
	float headingintegral;
	float headingDeltaPWM;
    float headingError;
	float theta_ref;
    float target_x;
    float target_y;
    int cur_target_idx;
    float dist_to_target;
	
    //outputs
    double   left_cmd;  //left wheel command [-1..1]
    double   right_cmd; //right wheel command [-1..1]

    //references
    float distance_ref;
    float psi_ref;
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{
    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int ctl;
    int reset_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
};

#endif
