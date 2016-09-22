#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX_MOTOR 1000            ///< Maximum PWM signal (1000us is added)
#define MAX_CMD 1024              ///< Maximum thrust, roll, pitch and yaw command
#define MIN_CMD -MAX_CMD          ///< Minimum roll, pitch, yaw command
#define PANIC_TIME 2*1000         ///< Time to keep thrust in panic mode (us)
#define PANIC_THRUST 0.4*MAX_CMD  ///< The amount of thrust in panic mode
#define MAX_YAW_RATE 45*131       ///< The maximum yaw rate from js (131 LSB / (degrees/s))
#define MAX_ANGLE 0               ///< The maximum angle from js

#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }

/* Local variables */
static uint32_t panic_start;                  ///< Time at which panic mode is entered
static uint16_t cmd_thrust;                   ///< The thrust command
static int16_t cmd_roll, cmd_pitch, cmd_yaw ;  ///< The roll, pitch, yaw command
static uint16_t sp_thrust;                    ///< The thrust setpoint
static int16_t sp_roll, sp_pitch, sp_yaw;     ///< The roll, pitch, yaw setpoint
static int16_t cphi, ctheta, cpsi;            ///< Calibration values of phi, theta, psi
static int16_t cp, cq, cr;                   ///< Calibration valies of p, q and r
//static uint16_t groll_p, groll_i, groll_d = 0;    ///< The roll control gains (2^CONTROL_FRAC)
//static uint16_t gpitch_p, gpitch_i, gpitch_d = 0; ///< The pitch control gains (2^CONTROL_FRAC)
static uint16_t gyaw_d;                       ///< The yaw control gains (2^CONTROL_FRAC)

/* Set the motor commands */
void update_motors(void);
/* Calculate the motor commands from thrust, roll, pitch and yaw commands */
static void motor_mixing(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw);
/* Change the control mode */
void set_control_mode(enum control_mode_t mode);
/* Set the control gains */
void set_control_gains(uint16_t yaw_d);
/* Set the control commands (from joystick or keyboard) */
void set_control_from_js(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw);
/* Run the filters and control */
void run_filters_and_control(void);

#endif /* _CONTROL_H */