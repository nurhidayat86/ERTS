/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input = 1-2ms
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
 #include "protocol.h"
//#include "control.h"

#define PRESCALE 3
#define MAX_THRUST_COM 8192
#define MIN_THRUST_COM 0
#define MAX_ATTITUDE_COM 8192
#define MIN_ATTITUDE_COM -MAX_ATTITUDE_COM

#define MAX_MOTOR 1000            ///< Maximum PWM signal (1000us is added)
#define MAX_CMD 1024              ///< Maximum thrust, roll, pitch and yaw command
#define MIN_CMD -MAX_CMD          ///< Minimum roll, pitch, yaw command
#define PANIC_TIME 1000*1000         ///< Time to keep thrust in panic mode (us)
#define PANIC_THRUST 0.4*MAX_THRUST_COM  ///< The amount of thrust in panic mode
#define MAX_YAW_RATE 45*131       ///< The maximum yaw rate from js (131 LSB / (degrees/s))
#define MAX_ANGLE 0               ///< The maximum angle from js

#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }

/* Local variables */
static uint32_t panic_start = 0;                  ///< Time at which panic mode is entered
static uint16_t cmd_thrust = 0;                   ///< The thrust command
static int16_t cmd_roll, cmd_pitch, cmd_yaw = 0;  ///< The roll, pitch, yaw command
static uint16_t sp_thrust = 0;                    ///< The thrust setpoint
static int16_t sp_roll, sp_pitch, sp_yaw = 0;     ///< The roll, pitch, yaw setpoint
static int16_t cphi, ctheta, cpsi = 0;            ///< Calibration values of phi, theta, psi
static int16_t cp, cq, cr = 0;                    ///< Calibration valies of p, q and r
//static uint16_t groll_p, groll_i, groll_d = 0;    ///< The roll control gains (2^CONTROL_FRAC)
//static uint16_t gpitch_p, gpitch_i, gpitch_d = 0; ///< The pitch control gains (2^CONTROL_FRAC)
static uint16_t gyaw_d = 0;                       ///< The yaw control gains (2^CONTROL_FRAC)

/* Set the motor commands */
void update_motors(void)
{
	NRF_TIMER1->CC[0] = 1000 + ae[0];
	NRF_TIMER1->CC[1] = 1000 + ae[1];
	NRF_TIMER1->CC[2] = 1000 + ae[2];
	NRF_TIMER1->CC[3] = 1000 + ae[3];
}

/* Calculate the motor commands from thrust, roll, pitch and yaw commands */
static void motor_mixing(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw) {
  // Front motor
  ae[0] = thrust + pitch - yaw;
  //ae[0] *= MAX_MOTOR;
  //ae[0] /= MIN_CMD;
  //Bound(ae[0], MIN_CMD, MAX_CMD);
  ae[0] = ae[0]>>PRESCALE;
  Bound(ae[0], 0, MAX_MOTOR);

  // Right motor
  ae[1] = thrust - roll + yaw;
  // ae[1] *= MAX_MOTOR;
  // ae[1] /= MIN_CMD;
  // Bound(ae[1], MIN_CMD, MAX_CMD);
  ae[1] = ae[1]>>PRESCALE;
  Bound(ae[1], 0, MAX_MOTOR);

  // Back motor
  ae[2] = thrust - pitch - yaw;
  // ae[2] *= MAX_MOTOR;
  // ae[2] /= MIN_CMD;
  // Bound(ae[2], MIN_CMD, MAX_CMD);
  ae[2] = ae[2]>>PRESCALE;
  Bound(ae[2], 0, MAX_MOTOR);

  // Left motor
  ae[3] = thrust + roll + yaw;
  // ae[3] *= MAX_MOTOR;
  // ae[3] /= MIN_CMD;
  // Bound(ae[3], MIN_CMD, MAX_CMD);
  ae[3] = ae[3]>>PRESCALE;
  Bound(ae[3], 0, MAX_MOTOR);
  //printf("%d %d %d %d\n", ae[0], ae[1], ae[2], ae[3]);
}

/* Change the control mode */
void set_control_mode(enum control_mode_t mode) {
  /* Certain modes require to reset variables */
  switch(control_mode) {
    /* Mode panic needs the enter time */
    case MODE_PANIC:
      panic_start = get_time_us();
      break;

    /* Mode manual needs to reset the setpoints */
    case MODE_MANUAL:
      sp_thrust = sp_roll = sp_pitch = sp_yaw = 0;
      break;

    /* Mode calibration needs to calibrate the drone */
    case MODE_CALIBRATION:
      cphi = phi;
      ctheta = theta;
      cpsi = psi;
      cp = sp;
      cq = sq;
      cr = sr;
      break;

    default:
      break;
  };
  control_mode = mode;
}

/* Set the control gains */
void set_control_gains(uint16_t yaw_d) {
  gyaw_d = yaw_d;
}

/* Set the control commands (from joystick or keyboard) */
void set_control_command(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw) {
  /* Based on the control mode we need to use it seperately */
  switch(control_mode) {
    /* Mode manual copies it to the commands */
    case MODE_MANUAL:
      cmd_thrust = thrust;
      cmd_roll = roll;
      cmd_pitch = pitch;
      cmd_yaw = yaw;
      break;

    /* Mode yaw sets the yaw setpoint and the rest as commands */
    case MODE_YAW:
      cmd_thrust = thrust;
      cmd_roll = roll;
      cmd_pitch = pitch;
      sp_yaw = yaw * MAX_YAW_RATE / MAX_CMD;
      break;

    /* Roll, pitch and yaw setpoint is set and the thrust as command */
    case MODE_FULL:
      cmd_thrust = thrust;
      sp_roll = roll * MAX_ANGLE / MAX_CMD;
      sp_pitch = pitch * MAX_ANGLE / MAX_CMD;
      sp_yaw = yaw * MAX_YAW_RATE / MAX_CMD;
      break;

    default:
      break;
  }
}

/* Run the filters and control */
void run_filters_and_control(void)
{
  /* Based on the control mode execute commands */
	switch(control_mode) {
    /* Safe mode (no thrust at all) */
    case MODE_SAFE:
      ae[0] = ae[1] = ae[2] = ae[3] = 0;
      break;

    /* Panic mode (PANIC_THRUST of thrust for PANIC_TIME seconds, then safe mode) */
    case MODE_PANIC:
      motor_mixing(PANIC_THRUST, 0, 0, 0);

      // Check if time exceeded
      if((get_time_us() - panic_start) > PANIC_TIME) {
        set_control_mode(MODE_SAFE);
      }
      break;

    /* Manual mode is direct mapping from sticks to controls */
    case MODE_MANUAL:
      motor_mixing(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);
      break;

    /* Calibration mode (not thrust at all) */
    case MODE_CALIBRATION:
      ae[0] = ae[1] = ae[2] = ae[3] = 0;

      // It takae sometimes (~ 6s) until it returns a stable value
      // Also calibrate here (until leave mode)
      cphi = phi;
      ctheta = theta;
      cpsi = psi;
      cp = sp;
      cq = sq;
      cr = sr;
      break;

    /* Yaw rate controlled mode */
    case MODE_YAW:
      cmd_yaw = ((sp_yaw - sr - cr) * gyaw_d) >> CONTROL_FRAC;
      motor_mixing(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);
      break;

    /* Just in case an invalid mode is selected go to SAFE */
    default:
      control_mode = MODE_SAFE;
      break;
  };

  /* Update the motor commands */
	update_motors();
}
