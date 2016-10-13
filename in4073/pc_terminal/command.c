/*------------------------------------------------------------
 * Command combination
 *
 * 
 *------------------------------------------------------------
 */
#include "command.h"
#include "keyboard.h"
#include "serial.h"



void InitCommand(struct msg_combine_all_t* combine_msg_all, struct msg_combine_t* combine_msg, struct msg_keyboard_t* keyboard_msg, struct msg_joystick_t* joystick_msg, struct msg_tuning_t* tuning_msg)
{
	joystick_msg->update = false;
	joystick_msg->mode = MODE_SAFE;
	joystick_msg->thrust = 0;
	joystick_msg->roll = 0;
	joystick_msg->pitch = 0;
	joystick_msg->yaw = 0;

	keyboard_msg->update = false;
	keyboard_msg->mode = MODE_SAFE;
	keyboard_msg->thrust = 0;
	keyboard_msg->roll = 0;
	keyboard_msg->pitch = 0;
	keyboard_msg->yaw = 0;

	combine_msg->update = false;
	combine_msg->mode = MODE_SAFE;
	combine_msg->thrust = 0;
	combine_msg->roll = 0;
	combine_msg->pitch = 0;
	combine_msg->yaw = 0;
	
	tuning_msg->update = false;
	tuning_msg->P = 0;
	tuning_msg->P1 = 0;
	tuning_msg->P2 = 0;
	tuning_msg->log_flag = false;

	combine_msg_all->update = false;
	combine_msg_all->mode = MODE_SAFE;
	combine_msg_all->thrust = 0;
	combine_msg_all->roll = 0;
	combine_msg_all->pitch = 0;
	combine_msg_all->yaw = 0;
	combine_msg_all->P = 0;
	combine_msg_all->P1 = 0;
	combine_msg_all->P2 = 0;
	combine_msg_all->log_flag = 0;
}

void CombineCommand(struct msg_combine_all_t* combine_msg_all, struct msg_combine_t* combine_msg, struct msg_keyboard_t* keyboard_msg, struct msg_joystick_t* joystick_msg, struct msg_tuning_t* tuning_msg)
{
	if(keyboard_msg->update)
	{combine_msg->mode = joystick_msg->mode = keyboard_msg->mode;}
	else
	{combine_msg->mode = keyboard_msg->mode = joystick_msg->mode;}

	// combine the message if the mode are neither safe mode nor panic mode 
	if(combine_msg->mode != MODE_SAFE && combine_msg->mode != MODE_PANIC){
		
		// thrust treshold from joystick
		// to make sure the message will be updated if the thrust value from joystik greater than treshold
		if( joystick_msg->thrust > 30)
		{
			combine_msg->thrust = joystick_msg->thrust + keyboard_msg->thrust;
			combine_msg->roll = joystick_msg->roll + keyboard_msg->roll;
			combine_msg->pitch = joystick_msg->pitch + keyboard_msg->pitch;
			combine_msg->yaw = joystick_msg->yaw + keyboard_msg->yaw;
		}
		// reset the value from keyboard as well if the joystick thrust below the threshold
		else
		{
			combine_msg->thrust = keyboard_msg->thrust = 0;
			combine_msg->roll = keyboard_msg->roll = 0;
			combine_msg->pitch = keyboard_msg->pitch = 0;
			combine_msg->yaw = keyboard_msg->yaw = 0;	
		}
		
		// limit the command value, it is not fixed yet, it depends on the bit we play especially for the joystick part
		Bound(combine_msg->thrust, MIN_THRUST_COM, MAX_THRUST_COM);
		Bound(combine_msg->roll, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
		Bound(combine_msg->pitch, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
		Bound(combine_msg->yaw, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
	}

	// reset the combine command if the mode is safe mode
	if(combine_msg->mode == MODE_SAFE) 
	{
		// combine_msg->thrust = keyboard_msg->thrust = 0;	
		// combine_msg->roll = keyboard_msg->roll = 0;
		// combine_msg->pitch = keyboard_msg->pitch = 0;
		// combine_msg->yaw = keyboard_msg->yaw = 0;

		combine_msg->thrust = joystick_msg->thrust + keyboard_msg->thrust;
		combine_msg->roll = joystick_msg->roll + keyboard_msg->roll;
		combine_msg->pitch = joystick_msg->pitch + keyboard_msg->pitch;
		combine_msg->yaw = joystick_msg->yaw + keyboard_msg->yaw;

		tuning_msg->P = 0;
		tuning_msg->P1 = 0;
		tuning_msg->P2 = 0;
	}
	
	combine_msg_all->mode = combine_msg->mode;
	combine_msg_all->thrust = combine_msg->thrust;
	combine_msg_all->roll = combine_msg->roll;
	combine_msg_all->pitch = combine_msg->pitch;
	combine_msg_all->yaw = combine_msg->yaw;
	combine_msg_all->P = tuning_msg->P;
	combine_msg_all->P1 = tuning_msg->P1;
	combine_msg_all->P2 = tuning_msg->P2;
	combine_msg_all->log_flag = tuning_msg->log_flag;

	// update the flag, it indicates the message has been updated
	combine_msg->update=false;
	// combine_msg->update=TRUE;
	joystick_msg->update=false;
	keyboard_msg->update=false;	
	combine_msg_all->update = true;
}

void SendCommand(struct msg_combine_t* combine_msg)
{
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;

	encode_packet((uint8_t *) combine_msg, sizeof(struct msg_combine_t), MSG_COMBINE, output_data, &output_size);
	
	// send the message
	for (i=0; i<output_size; i++) {	
		rs232_putchar((char) output_data[i]);
		// printf("0x%X ", (uint8_t) output_data[i]);
	}
	// printf("\n");
}

void SendCommandTuning(struct msg_tuning_t* tuning_msg)
{
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;

	encode_packet((uint8_t *) tuning_msg, sizeof(struct msg_tuning_t), MSG_TUNE, output_data, &output_size);
	// send the message
	for (i=0; i<output_size; i++) {	
		rs232_putchar((char) output_data[i]);
	}
}

void SendCommandAll(struct msg_combine_all_t* combine_msg_all)
{
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;

	encode_packet((uint8_t *) combine_msg_all, sizeof(struct msg_combine_all_t), MSG_COMBINE_ALL, output_data, &output_size);
	
	// send the message
	for (i=0; i<output_size; i++) {	
		rs232_putchar((char) output_data[i]);
		// printf("0x%X ", (uint8_t) output_data[i]);
	}
	// printf("\n");
}
