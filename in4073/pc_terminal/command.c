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
		combine_msg->thrust = keyboard_msg->thrust = 0;	
		combine_msg->roll = keyboard_msg->roll = 0;
		combine_msg->pitch = keyboard_msg->pitch = 0;
		combine_msg->yaw = keyboard_msg->yaw = 0;
	}
	
	// update the flag, it indicates the message has been updated
	combine_msg->update=FALSE;
	joystick_msg->update=FALSE;
	keyboard_msg->update=FALSE;	
}
