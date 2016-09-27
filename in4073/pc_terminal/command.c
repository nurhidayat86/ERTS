/*------------------------------------------------------------
 * Command combination
 *
 * 
 *------------------------------------------------------------
 */
#include "command.h"
#include "keyboard.h"

void CombineCommand(struct msg_combine_t* combine_msg, struct msg_keyboard_t* keyboard_msg, struct msg_joystick_t* joystick_msg)
{
	if(keyboard_msg->update)
	{combine_msg->mode = joystick_msg->mode = keyboard_msg->mode;}
	//else
	//{combine_msg->mode = joystick_msg->mode;}

	if(combine_msg->mode != MODE_SAFE && combine_msg->mode != MODE_PANIC){
		
		if((int16_t)(joystick_msg->thrust + keyboard_msg->thrust) > 0)
		{
			combine_msg->thrust = joystick_msg->thrust + keyboard_msg->thrust;
			combine_msg->roll = joystick_msg->roll + keyboard_msg->roll;
			combine_msg->pitch = joystick_msg->pitch + keyboard_msg->pitch;
			combine_msg->yaw = joystick_msg->yaw + keyboard_msg->yaw;
		}
		
		Bound(combine_msg->thrust, MIN_THRUST_COM, MAX_THRUST_COM);
		Bound(combine_msg->roll, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
		Bound(combine_msg->pitch, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
		Bound(combine_msg->yaw, MIN_ATTITUDE_COM, MAX_ATTITUDE_COM);
	}

	if(combine_msg->mode == MODE_SAFE) 
	{
		combine_msg->thrust = keyboard_msg->thrust = 0;	
		combine_msg->roll = keyboard_msg->roll = 0;
		combine_msg->pitch = keyboard_msg->pitch = 0;
		combine_msg->yaw = keyboard_msg->yaw = 0;
	}
	
	combine_msg->update=FALSE;
	joystick_msg->update=FALSE;
	keyboard_msg->update=FALSE;	
}
