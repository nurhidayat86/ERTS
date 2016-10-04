/*------------------------------------------------------------
 * Keyboard
 * process the keyboard command
 * 
 *------------------------------------------------------------
 */

#include "keyboard.h"

void KeyboardCommand(char c, struct msg_keyboard_t* keyboard_msg, struct msg_tuning_t* tuning_msg, struct msg_joystick_t* joystick_msg)
{
	if(keyboard_msg->mode != MODE_SAFE && keyboard_msg->mode != MODE_PANIC){
		switch(c) {
			// Controls
			case 'a': //thrust up
				if (keyboard_msg->thrust < MAX_THRUST_COM ) {	
					keyboard_msg->thrust+=KEY_INC;
				}
				break;
			case 'z': //thrust down
				if (keyboard_msg->thrust > MIN_THRUST_COM) {
					keyboard_msg->thrust-=KEY_INC;
				}
				break;

			case 'w': // yaw up (right)
				//printf("w");
				if (keyboard_msg->yaw < MAX_ATTITUDE_COM) {
					keyboard_msg->yaw+=KEY_INC;
				}
				break;

			case 'q': // yaw down (left)
				if (keyboard_msg->yaw > MIN_ATTITUDE_COM) {
					keyboard_msg->yaw-=KEY_INC;
				}
				break;
			
			case 66: // down arrow = pitch up
				if (keyboard_msg->pitch < MAX_ATTITUDE_COM) {
					keyboard_msg->pitch+=KEY_INC;
				}
				break;
			
			case 65: // up arrow = pitch down
				if (keyboard_msg->pitch > MIN_ATTITUDE_COM) {
					keyboard_msg->pitch-=KEY_INC;
				}
				break;
			
			case 68: // left arrow = roll up
				if (keyboard_msg->roll < MAX_ATTITUDE_COM) {
					keyboard_msg->roll+=KEY_INC;
				}
				break;

			case 67: // right arrow = roll down
				if (keyboard_msg->roll > MIN_ATTITUDE_COM) {
					keyboard_msg->roll-=KEY_INC;
				}
				break;

			// we can only go to these two modes from the other "dynamic mode"  
			case '0':	// go back to safe mode
				keyboard_msg->mode = MODE_SAFE;
				break;

			case '1':	// go to panic mode 
				keyboard_msg->mode = MODE_PANIC;
				break;

			case 'u':	// increase the yaw gain
				if (tuning_msg->P < 6) {
					tuning_msg->P+=1;
				}
				tuning_msg->update = TRUE;
				break;

			case 'j': // decrease the yaw gain
				if (tuning_msg->P > 0) {
					tuning_msg->P-=1;
				}
				tuning_msg->update = TRUE;
				break;
							
			default:
				break;
		}
	}
	
	// we can go to any other mode if we are in the safe mode
	// abort/exit only can be reach by doing the safe mode first
	if(keyboard_msg->mode == MODE_SAFE && joystick_msg->thrust == 0)
	{
		switch(c) {
			case '0':
				keyboard_msg->mode = MODE_SAFE;
				break;
			case '1':
				keyboard_msg->mode = MODE_PANIC;
				break;
			case '2': 
				keyboard_msg->mode = MODE_MANUAL;
				break;	
			case '3': 
				keyboard_msg->mode = MODE_CALIBRATION;
				break;	
			case '4': 
				keyboard_msg->mode = MODE_YAW;
				break;	
			case '5': 
				keyboard_msg->mode = MODE_FULL;
				break;	
			case '6': 
				keyboard_msg->mode = MODE_RAW;
				break;	
			case '7': 
				keyboard_msg->mode = MODE_HEIGHT;
				break;	
			case 27: 
				keyboard_msg->mode = ESCAPE;

			default:
				break;
		}
	}		
}
