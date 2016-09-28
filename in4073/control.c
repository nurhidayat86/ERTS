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

#include "quadrupel.h"
#define CONTROL_FRAC 6           // 
#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }

/* Local variables */

void update_motors(void)
{								
	NRF_TIMER1->CC[0] = 1000 + ae[0];			
	NRF_TIMER1->CC[1] = 1000 + ae[1];			
	NRF_TIMER1->CC[2] = 1000 + ae[2];			
	NRF_TIMER1->CC[3] = 1000 + ae[3];		
}

/* Calculate the motor commands from thrust, roll, pitch and yaw commands */
void motor_mixing(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw) {
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

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters
	update_motors();
	// yaw_control();
}

void calibration(void)
{
    int samples = 100;
    int i = 0, j = 0;
    int sum1, sum2, sum3, sum4, sum5, sum6;

    for(i=0, sum1=0, sum2=0, sum3=0, sum4=0, sum5=0, sum6=0; i<samples; i++) {

        if (check_sensor_int_flag()) 
        {
	    get_dmp_data();

	    j++;  // number of samples taken

	    clear_sensor_int_flag();
        }
		

/****************************************************/
        sum1 += sax;


/****************************************************/
        sum2 += say;


/****************************************************/
        sum3 += saz;


/****************************************************/
        sum4 += sp;


/****************************************************/
        sum5 += sp;


/****************************************************/
        sum6 += sp;

    nrf_delay_ms(20);

    }

    printf("%d samples taken \n", j);

    cphi = sum1/samples;
    ctheta = sum2/samples;
    cpsi = sum3/samples;
    cp = sum4/samples;
    cq = sum5/samples;
    cr = sum6/samples;
}


void yaw_control(void)
{
      cmd_yaw = ((sp_yaw - sr - cr) * gyaw_d) >> CONTROL_FRAC;
      motor_mixing(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);


}
