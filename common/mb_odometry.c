/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta) {
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->psi = theta;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state) {
	float pos = (mb_state->left_pos + mb_state->right_pos) / 2;
    float del_dist = pos - mb_state->last_pos;
	float avgAngle = (mb_state->heading + mb_state->last_heading) / 2; //average heading over last step
    float del_x = del_dist * cos(avgAngle);
    float del_y = del_dist * sin(avgAngle);
    mb_odometry->x += del_x;
    mb_odometry->y += del_y;
    mb_odometry->psi = mb_state->heading;
    mb_state->last_pos = pos;
    mb_state->last_heading = avgAngle;
}

float mb_clamp_radians(float angle){
    if (angle > 0.007537){
        return 0.007537;
    }
    if (angle < -0.007537) {
        return -0.007537;
    }
    return angle;
}
