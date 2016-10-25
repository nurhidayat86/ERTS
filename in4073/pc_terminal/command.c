/*------------------------------------------------------------
 * Command combination
 *
 * 
 *------------------------------------------------------------
 */
#include "command.h"
#include "keyboard.h"
#include "serial.h"

void InitCommandAll(struct msg_joystick_t* joystick_msg, struct msg_keyboard_t* keyboard_msg, struct msg_combine_all_t* combine_msg_all)
{
	combine_msg_all->update = false;
	combine_msg_all->mode = MODE_SAFE;
	combine_msg_all->thrust = 0;
	combine_msg_all->roll = 0;
	combine_msg_all->pitch = 0;
	combine_msg_all->yaw = 0;
	combine_msg_all->P = 0;
	combine_msg_all->P1 = 0;
	combine_msg_all->P2 = 0;
	combine_msg_all->msc_flag = 0;

	joystick_msg->update = false;
	joystick_msg->mode = MODE_SAFE;
	joystick_msg->thrust = 0;
	joystick_msg->roll = 0;
	joystick_msg->pitch = 0;
	joystick_msg->yaw = 0;
	joystick_msg->P = 0;
	joystick_msg->P1 = 0;
	joystick_msg->P2 = 0;
	joystick_msg->msc_flag = 0;

	keyboard_msg->update = false;
	keyboard_msg->mode = MODE_SAFE;
	keyboard_msg->thrust = 0;
	keyboard_msg->roll = 0;
	keyboard_msg->pitch = 0;
	keyboard_msg->yaw = 0;
	keyboard_msg->P = 0;
	keyboard_msg->P1 = 0;
	keyboard_msg->P2 = 0;
	keyboard_msg->msc_flag = 0;
}

void InitCommandUpdate(struct msg_combine_all_t* combine_msg_all)
{
	combine_msg_all->update = false;
}

void CommandUpdate(struct msg_combine_all_t* combine_msg_all)
{
	combine_msg_all->update = true;

}

void CommandModeSafe(struct msg_combine_all_t* combine_msg_all)
{
	combine_msg_all->mode = MODE_SAFE;
	combine_msg_all->P = 0;
	combine_msg_all->P1 = 0;
	combine_msg_all->P2 = 0;
}

void CommandModeSafeAll(struct msg_joystick_t* joystick_msg, struct msg_keyboard_t* keyboard_msg, struct msg_combine_all_t* combine_msg_all)
{
	combine_msg_all->mode = MODE_SAFE;
	combine_msg_all->P = 0;
	combine_msg_all->P1 = 0;
	combine_msg_all->P2 = 0;
	
	keyboard_msg->mode = MODE_SAFE;
	keyboard_msg->P = 0;
	keyboard_msg->P1 = 0;
	keyboard_msg->P2 = 0;
	
	joystick_msg->mode = MODE_SAFE;
	joystick_msg->P = 0;
	joystick_msg->P1 = 0;
	joystick_msg->P2 = 0;
}


void CombineCommand(struct msg_combine_all_t* combine_msg_all)
{
	// combine the message if the mode are neither safe mode nor panic mode 
	if(combine_msg_all->mode != MODE_SAFE && combine_msg_all->mode != MODE_PANIC){
		
		// limit the command value, it is not fixed yet, it depends on the bit we play especially for the joystick part
		Bound(combine_msg_all->thrust, MIN_THRUST_COM, MAX_THRUST_COM);
		Bound(combine_msg_all->roll, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
		Bound(combine_msg_all->pitch, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
		Bound(combine_msg_all->yaw, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
	}

	// reset the combine command if the mode is safe mode
	if(combine_msg_all->mode == MODE_SAFE) 
	{
		combine_msg_all->P = 0;
		combine_msg_all->P1 = 0;
		combine_msg_all->P2 = 0;
	}
	
	// update the flag, it indicates the message has been updated
	combine_msg_all->update=FALSE;
}

void CombineCommandAll(struct msg_joystick_t* joystick_msg, struct msg_keyboard_t* keyboard_msg, struct msg_combine_all_t* combine_msg_all)
{
	if(keyboard_msg->update)
	{combine_msg_all->mode = joystick_msg->mode = keyboard_msg->mode;}
	else if(joystick_msg->update)
	{combine_msg_all->mode = keyboard_msg->mode = joystick_msg->mode;}

	// combine the message if the mode are neither safe mode nor panic mode 
	if(combine_msg_all->mode != MODE_SAFE && combine_msg_all->mode != MODE_PANIC){
		
		combine_msg_all->thrust = joystick_msg->thrust + keyboard_msg->thrust;
		combine_msg_all->roll = joystick_msg->roll + keyboard_msg->roll;
		combine_msg_all->pitch = joystick_msg->pitch + keyboard_msg->pitch;
		combine_msg_all->yaw = joystick_msg->yaw + keyboard_msg->yaw;
	
		// limit the command value, it is not fixed yet, it depends on the bit we play especially for the joystick part
		Bound(combine_msg_all->thrust, MIN_THRUST_COM, MAX_THRUST_COM);
		Bound(combine_msg_all->roll, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
		Bound(combine_msg_all->pitch, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
		Bound(combine_msg_all->yaw, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
	}

	// reset the combine command if the mode is safe mode
	if(combine_msg_all->mode == MODE_SAFE) 
	{	
		// update the command
		combine_msg_all->thrust = joystick_msg->thrust + keyboard_msg->thrust;
		combine_msg_all->roll = joystick_msg->roll + keyboard_msg->roll;
		combine_msg_all->pitch = joystick_msg->pitch + keyboard_msg->pitch;
		combine_msg_all->yaw = joystick_msg->yaw + keyboard_msg->yaw;
	
		combine_msg_all->P = keyboard_msg->P = 0;
		combine_msg_all->P1 = keyboard_msg->P1 = 0;
		combine_msg_all->P2 = keyboard_msg->P2 = 0;
	}
	
	// update the flag, it indicates the message has been updated
	combine_msg_all->update=FALSE;
	joystick_msg->update=FALSE;
	keyboard_msg->update=FALSE;	
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
