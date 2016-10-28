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

#define PRESCALE 3                      // shift the control output result so that it will be suitable with rotor PWM scale
#define THRUST_MIN 200<<3               // minimal thrust in motor mixing function so that it will continue to rotate (prevent the stall) in extreme control output 
#define THRUST_MIN_CONTROL 300<<3       // minimal thrust must be provided by PC so that the drone can do the manuver at the enough amount of thrust

#define MAX_THRUST_COM 8192                 // max thrust command that drone can get from the PC, it correspond with max PWM (1000) in motor part
#define MIN_THRUST_COM 0                    // PWM 0
#define MAX_ATTITUDE_COM 1024               // max attitude command that the drone can get from PC
#define MIN_ATTITUDE_COM -MAX_ATTITUDE_COM  // min attitude command (same value different rotation direction)

#define MAX_MOTOR 1000                      // Maximum PWM signal (1000us is added)
#define PANIC_TIME 2000*1000                // Time to keep thrust in panic mode (us)
#define PANIC_THRUST 0.4*MAX_THRUST_COM     // The amount of thrust in panic mode

// P
#define RATE_SHIFT_YAW 4            // yaw rate reading divider                   2047 bit    =   2000 deg/s
#define RATE_GAIN_SHIFT_YAW 0       // yaw gain divider                           does not need divider
// P1
#define ANGLE_SHIFT 4               // roll and pitch attitude reading divider    1023 bit    =   90 deg
#define ANGLE_GAIN_SHIFT 3          // roll and pitch gain divider                give 1/8 step for gain multiplication
// P2
#define RATE_SHIFT 4                // roll and pitch rate reading divider        2047 bit    =   2000 deg/s        
#define RATE_GAIN_SHIFT 1           // roll and pitch gain divider                give 1/2 step for gain multiplication

#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }

/* Local variables */
static uint32_t panic_start = 0;                    // Time at which panic mode is entered
static uint16_t cmd_thrust = 0;                     // The thrust command
static int16_t cmd_roll, cmd_pitch, cmd_yaw = 0;    // The roll, pitch, yaw command
static uint16_t sp_thrust = 0;                      // The thrust setpoint
static int16_t sp_roll, sp_pitch, sp_yaw = 0;       // The roll, pitch, yaw setpoint

static uint8_t gyaw_d = 0;                          // The yaw rate control gains
static uint8_t g_angle_d = 0;                       // The attitude control gains
static uint8_t g_rate_d = 0;                        // The attitude rate control gains
static uint32_t current_panic = 0;

//Generic function
void update_motors(void)
{
	NRF_TIMER1->CC[0] = 1000 + ae[0];
	NRF_TIMER1->CC[1] = 1000 + ae[1];
	NRF_TIMER1->CC[2] = 1000 + ae[2];
	NRF_TIMER1->CC[3] = 1000 + ae[3];
}


/*------------------------------------------------------------
 * void update_motors(void)
 * Author		: Angga Irawan
 * Adapted from :
 * Funtionalty	: Calculate the motor commands from thrust, roll, pitch and yaw commands
 *------------------------------------------------------------*/

static void motor_mixing(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw) {
    
    /* 
    Each motor PWM is limited such that: 
        -it will rotate normally if the thrust is below thrust minimum (200)
        -do the combination of manuver if the sum of the combination larger than the minimal thrust
        -it will rotate on minimum thrust if the thrust is greater than the thrust min but the total of combination < thrust min
    This restriction will enable the motor to keep rotating even if the combination of command result in huge delta on PWM
    */ 
    // Front motor
    if (thrust < THRUST_MIN) ae[0] = thrust;
    else if ((thrust + pitch - yaw)>THRUST_MIN) ae[0] = thrust + pitch - yaw;
    else ae[0] = THRUST_MIN;
    ae[0] = ae[0]>>PRESCALE;
    Bound(ae[0], 0, MAX_MOTOR);

    // Right motor
    if (thrust < THRUST_MIN) ae[1] = thrust;
    else if ((thrust - roll + yaw)>THRUST_MIN) ae[1] = thrust - roll + yaw;
    else ae[1] = THRUST_MIN;
    ae[1] = ae[1]>>PRESCALE;
    Bound(ae[1], 0, MAX_MOTOR);

    // Back motor
    if (thrust < THRUST_MIN) ae[2] = thrust;
    else if ((thrust - pitch - yaw)>THRUST_MIN) ae[2] = thrust - pitch - yaw;
    else ae[2] = THRUST_MIN;
    ae[2] = ae[2]>>PRESCALE;
    Bound(ae[2], 0, MAX_MOTOR);

    // Left motor
    if (thrust < THRUST_MIN) ae[3] = thrust;
    else if ((thrust + roll + yaw)>THRUST_MIN) ae[3] = thrust + roll + yaw;
    else ae[3] = THRUST_MIN;
    ae[3] = ae[3]>>PRESCALE;
    Bound(ae[3], 0, MAX_MOTOR);
}
/*---------------------------------------------------------------------------*/



/*------------------------------------------------------------
 * void set_control_mode(enum control_mode_t mode)
 * Author		: Arif Nurhidayat
 * Adapted from :
 * Funtionalty	: Change the control mode, and set intermediate action if it is needed
 *------------------------------------------------------------*/
/*  */
void set_control_mode(enum control_mode_t mode) {
    /* Certain modes require to reset variables */
    control_mode = mode;
    switch(control_mode) {
        case MODE_SAFE:
            P = P1 = P2 = 0;
            break;

        /* Mode panic needs the enter time */
        case MODE_PANIC:
            //nrf_gpio_pin_toggle(YELLOW);
            nrf_gpio_pin_clear(YELLOW);
            panic_start = get_time_us();
            break;

        /* Mode manual needs to reset the setpoints */
        case MODE_MANUAL:
            sp_thrust = sp_roll = sp_pitch = sp_yaw = 0;
            break;

        /* Mode calibration needs to calibrate the drone */
        case MODE_CALIBRATION:
        // doing calibration in run filter and control       
            break;

        /* Full Control Mode */
        case MODE_FULL:
            break;
        
        /* Raw Mode */
        case MODE_RAW:        
            break;

        case MODE_START:
            break;

        case MODE_FINISH:
            break;

        default:
            break;
    };
}
/*---------------------------------------------------------------------------*/




/*------------------------------------------------------------
 * void update_motors(void)
 * Author		: Reggie
 * Adapted from :
 * Funtionalty	: Set the control gain
 *------------------------------------------------------------*/
void set_control_gains(uint8_t yaw_d, uint8_t g_angle, uint8_t g_rate) {
    gyaw_d = yaw_d;
    g_angle_d = g_angle;
    g_rate_d = g_rate;
}
/*-------------------------------------------------------------*/


/*------------------------------------------------------------
 * void set_control_mode(enum control_mode_t mode)
 * Author		: Arif Nurhidayat
 * Adapted from :
 * Funtionalty	: Set the control commands (from joystick or keyboard)
 *------------------------------------------------------------*/

void set_control_command(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw) {
    /* Based on the control mode we need to use it seperately */
    switch(control_mode) {
        case MODE_PANIC:
            cmd_thrust = thrust;
        	break;

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
            sp_yaw = yaw;
            break;

        /* Roll, pitch and yaw setpoint is set and the thrust as command */
        case MODE_FULL:
            cmd_thrust = thrust;
            sp_roll = roll;
            sp_pitch = pitch;
            sp_yaw = yaw;
            break;

        case MODE_RAW:
            cmd_thrust = thrust;
            sp_roll = roll;
            sp_pitch = pitch;
            sp_yaw = yaw;
            break;

        case MODE_START:
            cmd_thrust = 0;
            cmd_roll = 0;
            cmd_pitch = 0;
            cmd_yaw = 0;
            break;

        case MODE_FINISH:
            cmd_thrust = 0;
            cmd_roll = 0;
            cmd_pitch = 0;
            cmd_yaw = 0;
            break;

        default:
            break;
    }
}
/*-------------------------------------------------------------*/


/*------------------------------------------------------------
 * void update_motors(void)
 * Author		: Angga Irawan
 * Adapted from : Generic program given by example
 * Funtionalty	: Run the filters and control
 *------------------------------------------------------------*/
void run_filters_and_control(void)
{
    /* Based on the control mode execute commands */
    switch(control_mode) {
        /* Safe mode (no thrust at all) */
        case MODE_SAFE:
            ae[0] = ae[1] = ae[2] = ae[3] = 0;
            nrf_gpio_pin_set(YELLOW);
            break;

        /* Panic mode (PANIC_THRUST of thrust for PANIC_TIME seconds, then safe mode) */
        case MODE_PANIC:
            nrf_gpio_pin_clear(YELLOW);
            if(cmd_thrust > PANIC_THRUST) motor_mixing(PANIC_THRUST, 0, 0, 0);
            else motor_mixing(cmd_thrust, 0, 0, 0);
            lost_flag = true;
            current_panic = get_time_us() - panic_start;
            if(current_panic > PANIC_TIME) 
            {
                set_control_mode(MODE_SAFE);
                pc_link = true;         
            }
            break;

        /* Manual mode is direct mapping from sticks to controls */
        case MODE_MANUAL:
            motor_mixing(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);    
            break;

        /* Calibration mode (no thrust at all) */
        case MODE_CALIBRATION:
            ae[0] = ae[1] = ae[2] = ae[3] = 0;
            calibration();
            break;

        /* Yaw rate controlled mode */
        case MODE_YAW:
            
            // do the yaw control if the thrust is high enough
            if(cmd_thrust > THRUST_MIN_CONTROL)
            {               
                
                if(init_raw == true) cmd_yaw = ((((sp_yaw>>RATE_SHIFT_YAW) - ((r_butter)>>RATE_SHIFT_YAW))* gyaw_d)>>RATE_GAIN_SHIFT_YAW);
                else cmd_yaw = ((((sp_yaw>>RATE_SHIFT_YAW) + ((sr - cr)>>RATE_SHIFT_YAW))* gyaw_d)>>RATE_GAIN_SHIFT_YAW);
                motor_mixing(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);
            }
            else
            {
                motor_mixing(cmd_thrust, 0, 0, 0);
            }

            break;

        case MODE_FULL:
            // notes for DMP mode, sensor reading sign convention
            // rate = y and z have the opposite sign with the convention
            // attitude angle = z has the opposite sign with the convention
            // P1 angle, P2 rate
            if(cmd_thrust > THRUST_MIN_CONTROL)     // do the control if the thrust is enough
            {               
                // use different signal type and also signal sign for raw mode 
                if(init_raw == true) 
                {
                    cmd_roll = (((sp_roll - ((phi)>>ANGLE_SHIFT))*g_angle_d)>>ANGLE_GAIN_SHIFT) - ((((estimated_p)>>RATE_SHIFT)*g_rate_d)>>RATE_GAIN_SHIFT);
                    cmd_pitch = (((sp_pitch - ((theta)>>ANGLE_SHIFT))*g_angle_d)>>ANGLE_GAIN_SHIFT) - ((((estimated_q)>>RATE_SHIFT)*g_rate_d)>>RATE_GAIN_SHIFT);
                    cmd_yaw = ((( (sp_yaw>>RATE_SHIFT_YAW) - ((r_butter)>>RATE_SHIFT_YAW))* gyaw_d)>>RATE_GAIN_SHIFT_YAW);
                }
                else
                // do the dmp control
                {
                    cmd_roll = (((sp_roll - ((phi - cphi)>>ANGLE_SHIFT))*g_angle_d)>>ANGLE_GAIN_SHIFT) - ((((sp - cp)>>RATE_SHIFT)*g_rate_d)>>RATE_GAIN_SHIFT);
                    cmd_pitch = (((sp_pitch - ((theta - ctheta)>>ANGLE_SHIFT))*g_angle_d)>>ANGLE_GAIN_SHIFT) + ((((sq - cq)>>RATE_SHIFT)*g_rate_d)>>RATE_GAIN_SHIFT);
                    cmd_yaw = ((( (sp_yaw>>RATE_SHIFT_YAW)  + ((sr - cr)>>RATE_SHIFT_YAW))* gyaw_d)>>RATE_GAIN_SHIFT_YAW);
                }
                motor_mixing(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);            
            }
            // just do the thrust command
            else    
            {
                motor_mixing(cmd_thrust, 0, 0, 0);
            }
            break;

        case MODE_RAW:
            // Doing nothing just toggle the raw flag
            break;

        case MODE_HEIGHT:
            break;

        // prevent the invalid mode
        default:
            control_mode = MODE_SAFE;
            break;
    };

    // update motors command
    update_motors();
}
/*-------------------------------------------------------------*/


/*------------------------------------------------------------
 * void calibration(void)
 * Author		: Reggie
 * Adapted from :
 * Funtionalty	: Performing calibration by averaging last 16 values from the sensors
 *------------------------------------------------------------*/
void calibration(void)
{
    uint8_t i;
    static int16_t cal_phi[16], cal_theta[16], cal_sax[16], cal_say[16], cal_sp[16], cal_sq[16], cal_sr[16];
    int32_t sum_phi=0, sum_theta=0, sum_sax=0, sum_say=0, sum_sp=0, sum_sq=0, sum_sr = 0;

    //shift the value by one;
    for (i=0;i<15;i++)
    {
        cal_phi[i] = cal_phi[i+1];
        cal_theta[i] = cal_theta[i+1];
        cal_sax[i] = cal_sax[i+1];
        cal_say[i] = cal_say[i+1];
        cal_sp[i] = cal_sp[i+1];
        cal_sq[i] = cal_sq[i+1];
        cal_sr[i] = cal_sr[i+1];

    }

    //pushing new value;
    cal_phi[15] = phi; cal_theta[15] = theta; cal_sax[15] = sax; cal_say[15] = say; cal_sp[15] = sp; cal_sq[15] = sq; cal_sr[15]=sr;

    //summing the input data
    for (i=0;i<16;i++)
    {
        sum_phi += cal_phi[i]; sum_theta += cal_theta[i]; sum_sax += cal_sax[i]; sum_say += cal_say[i]; sum_sp += cal_sp[i]; sum_sq += cal_sq[i]; sum_sr += cal_sr[i];
    }

    //averaging data
    cphi = sum_phi>>4; ctheta = sum_theta>>4; csax = sum_sax>>4; csay = sum_say>>4; cp = sum_sp>>4; cq = sum_sq>>4; cr = sum_sr>>4;
}
/*------------------------------------------------------------*/