/*******************************************************************************
* mb_motor.h
*******************************************************************************/

#ifndef MB_MOTOR_H
#define MB_MOTOR_H

// #define MDIR1_CHIP              
// #define MDIR1_PIN                    //MDIRR1 gpio(CHIP.PIN) P9.12
// #define MDIR2_CHIP              
// #define MDIR2_PIN                    //MDIRR2 gpio(CHIP.PIN)
#define MOT_BRAKE_EN_PIN   0,20         // gpio0.20  P9.41
// #define MOT1_CS_PIN                  // analog in of motor 1 current sense
// #define MOT2_CS_PIN                  // analog in of motor 2 current sense
#define MB_MOTOR_DEFAULT_PWM_FREQ 25000

//fuctions
int mb_motor_init();
int mb_motor_init_freq(int pwm_freq_hz);
int mb_motor_disable();
int mb_motor_brake(int brake_en);
int mb_motor_set(int motor, double duty);
int mb_motor_set_all(double duty);
int mb_motor_cleanup();
double mb_motor_read_current(int motor);
double get_mot_dist(int motor);

#endif