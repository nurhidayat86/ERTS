/*------------------------------------------------------------
 * Keyboard
 * process the keyboard command
 * 
 *------------------------------------------------------------
 */

#include "keyboard.h"

void KeyboardCommand(char c, struct msg_keyboard_t* keyboard_msg)
{
	if(keyboard_msg->mode != MODE_SAFE && keyboard_msg->mode != MODE_PANIC){

		switch(c) {
			// Controls
			case 'a': //thrust up
				if (keyboard_msg->thrust < MAX_THRUST ) {	
					keyboard_msg->thrust+=20;
				}
				break;
			case 'z': //thrust down
				if (keyboard_msg->thrust > MIN_THRUST) {
					keyboard_msg->thrust-=20;
				}
				break;

			case 'w': // yaw up (right)
				//printf("w");
				if (keyboard_msg->yaw < MAX_ATTITUDE) {
					keyboard_msg->yaw+=20;
				}
				break;

			case 'q': // yaw down (left)
				if (keyboard_msg->yaw > MIN_ATTITUDE) {
					keyboard_msg->yaw-=20;
				}
				break;
			
			case 66: // down arrow = pitch up
				if (keyboard_msg->pitch < MAX_ATTITUDE) {
					keyboard_msg->pitch+=20;
				}
			
			case 65: // up arrow = pitch down
				if (keyboard_msg->pitch > MIN_ATTITUDE) {
					keyboard_msg->pitch-=20;
				}
			
			case 68: // left arrow = roll up
				if (keyboard_msg->roll < MAX_ATTITUDE) {
					keyboard_msg->roll+=20;
				}

			case 67: // right arrow = roll down
				if (keyboard_msg->roll > MIN_ATTITUDE) {
					keyboard_msg->roll-=20;
				}

							
			default:
				break;
		}
	}

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
		case 27: 
			keyboard_msg->mode = ESCAPE;

		default:
			break;
	}
		
}
