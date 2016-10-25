/*------------------------------------------------------------
 * Keyboard
 * process the keyboard command
 * 
 *------------------------------------------------------------
 */

#include "keyboard.h"

void initraw_stat()
{
	raw_stat = false;
	log_stat = false;
}

void KeyboardCommandSplit(char c, struct msg_joystick_t* joystick_msg, struct msg_keyboard_t* keyboard_msg)
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
			
			case 67: // right arrow = roll up
				if (keyboard_msg->roll < MAX_ATTITUDE_COM) {
					keyboard_msg->roll+=KEY_INC;
				}
				break;

			case 68: // left arrow = roll down
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
				if(keyboard_msg->mode == MODE_YAW || keyboard_msg->mode == MODE_FULL)
				{
					if (keyboard_msg->P < MAX_P) {keyboard_msg->P+=1;}
					keyboard_msg->update = TRUE;
				}
				break;

			case 'j': // decrease the yaw gain
				if(keyboard_msg->mode == MODE_YAW || keyboard_msg->mode == MODE_FULL)
				{	
					if (keyboard_msg->P > 0) {keyboard_msg->P-=1;}
					keyboard_msg->update = TRUE;
				}
				break;
			
			case 'i':	// increase the P1 gain
				if(keyboard_msg->mode == MODE_FULL)
				{	
					if (keyboard_msg->P1 < MAX_P1) {keyboard_msg->P1+=1;}
					keyboard_msg->update = TRUE;
				}
				break;

			case 'k': // decrease the P1 gain
				if(keyboard_msg->mode == MODE_FULL)
				{	
					if (keyboard_msg->P1 > 0) {keyboard_msg->P1-=1;}
					keyboard_msg->update = TRUE;
				}
				break;
	
			case 'o':	// increase the P2 gain
				if(keyboard_msg->mode == MODE_FULL)
				{	
					if (keyboard_msg->P2 < MAX_P2) {keyboard_msg->P2+=1;}
					keyboard_msg->update = TRUE;
				}
				break;

			case 'l': // decrease the P2 gain
				if(keyboard_msg->mode == MODE_FULL)
				{
					if (keyboard_msg->P2 > 0) {keyboard_msg->P2-=1;}
					keyboard_msg->update = TRUE;
				}
				break;

			case 'n': // toggle the stop sending bool
				if (stop_sending == FALSE) 
				{
					stop_sending = TRUE;
					// printf("stop true %d\n", stop_sending);
				}
				else if (stop_sending == TRUE) {
					stop_sending = FALSE;
					// printf("stop false%d\n", stop_sending);
				}
				// printf("toggle\n");				
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

			/*****************************************************************
			* Toggling RAW and DMP_MODE
			******************************************************************/
			case '6': 
				//keyboard_msg->mode = MODE_RAW;
				if (raw_stat == true) 
				{
					keyboard_msg->msc_flag = RAW_NO_USE;
					raw_stat = false;
					//printf("masuk ke false");
				}
				else if (raw_stat == false) 
				{
					keyboard_msg->msc_flag = RAW_USE;
					raw_stat = true;
					//printf("masuk ke true");
				}
				break;
			/*****************************************************************/
			case '7': 
				keyboard_msg->mode = MODE_HEIGHT;
				break;	
			case 27: 
				keyboard_msg->mode = ESCAPE;
				break;

			default:
				break;
		}
	}	

	switch(c) {
	// additional
		case 'm': // start flexible logging 
			if (log_stat == true) 
			{
				keyboard_msg->msc_flag = LOG_NO_USE;
				log_stat = false;
			}
			else if (log_stat == false) {
				keyboard_msg->msc_flag = LOG_USE;
				log_stat = true;
			}
		break;
	}	
}


void KeyboardCommand(char c, struct msg_combine_all_t* combine_msg_all)
{
	if(combine_msg_all->mode != MODE_SAFE && combine_msg_all->mode != MODE_PANIC){
		switch(c) {
			// Controls
			case 'a': //thrust up
				if (combine_msg_all->thrust < MAX_THRUST_COM ) {	
					combine_msg_all->thrust+=KEY_INC;
				}
				break;
			case 'z': //thrust down
				if (combine_msg_all->thrust > MIN_THRUST_COM) {
					combine_msg_all->thrust-=KEY_INC;
				}
				break;

			case 'w': // yaw up (right)
				if (combine_msg_all->yaw < MAX_ATTITUDE_COM) {
					combine_msg_all->yaw+=KEY_INC;
				}
				break;

			case 'q': // yaw down (left)
				if (combine_msg_all->yaw > MIN_ATTITUDE_COM) {
					combine_msg_all->yaw-=KEY_INC;
				}
				break;
			
			case 66: // down arrow = pitch up
				if (combine_msg_all->pitch < MAX_ATTITUDE_COM) {
					combine_msg_all->pitch+=KEY_INC;
				}
				break;
			
			case 65: // up arrow = pitch down
				if (combine_msg_all->pitch > MIN_ATTITUDE_COM) {
					combine_msg_all->pitch-=KEY_INC;
				}
				break;
			
			case 67: // right arrow = roll up
				if (combine_msg_all->roll < MAX_ATTITUDE_COM) {
					combine_msg_all->roll+=KEY_INC;
				}
				break;

			case 68: // left arrow = roll down
				if (combine_msg_all->roll > MIN_ATTITUDE_COM) {
					combine_msg_all->roll-=KEY_INC;
				}
				break;

			// we can only go to these two modes from the other "dynamic mode"  
			case '0':	// go back to safe mode
				combine_msg_all->mode = MODE_SAFE;
				break;

			case '1':	// go to panic mode 
				combine_msg_all->mode = MODE_PANIC;
				break;

			case 'u':	// increase the yaw gain
				if(combine_msg_all->mode == MODE_YAW || combine_msg_all->mode == MODE_FULL)
				{
					if (combine_msg_all->P < MAX_P) {combine_msg_all->P+=1;}
					combine_msg_all->update = TRUE;
				}
				break;

			case 'j': // decrease the yaw gain
				if(combine_msg_all->mode == MODE_YAW || combine_msg_all->mode == MODE_FULL)
				{	
					if (combine_msg_all->P > 0) {combine_msg_all->P-=1;}
					combine_msg_all->update = TRUE;
				}
				break;
			
			case 'i':	// increase the P1 gain
				if(combine_msg_all->mode == MODE_FULL)
				{	
					if (combine_msg_all->P1 < MAX_P1) {combine_msg_all->P1+=1;}
					combine_msg_all->update = TRUE;
				}
				break;

			case 'k': // decrease the P1 gain
				if(combine_msg_all->mode == MODE_FULL)
				{	
					if (combine_msg_all->P1 > 0) {combine_msg_all->P1-=1;}
					combine_msg_all->update = TRUE;
				}
				break;
	
			case 'o':	// increase the P2 gain
				if(combine_msg_all->mode == MODE_FULL)
				{	
					if (combine_msg_all->P2 < MAX_P2) {combine_msg_all->P2+=1;}
					combine_msg_all->update = TRUE;
				}
				break;

			case 'l': // decrease the P2 gain
				if(combine_msg_all->mode == MODE_FULL)
				{
					if (combine_msg_all->P2 > 0) {combine_msg_all->P2-=1;}
					combine_msg_all->update = TRUE;
				}
				break;

			case 'n': // toggle the stop sending bool
				if (stop_sending == FALSE) 
				{
					stop_sending = TRUE;
					// printf("stop true %d\n", stop_sending);
				}
				else if (stop_sending == TRUE) {
					stop_sending = FALSE;
					// printf("stop false%d\n", stop_sending);
				}
				// printf("toggle\n");				
			break;

			default:
				break;
		}
	}
	
	// we can go to any other mode if we are in the safe mode
	// abort/exit only can be reach by doing the safe mode first
	if(combine_msg_all->mode == MODE_SAFE && combine_msg_all->thrust == 0)
	{
		switch(c) {
			case '0':
				combine_msg_all->mode = MODE_SAFE;
				break;
			case '1':
				combine_msg_all->mode = MODE_PANIC;
				break;
			case '2': 
				combine_msg_all->mode = MODE_MANUAL;
				break;	
			case '3': 
				combine_msg_all->mode = MODE_CALIBRATION;
				break;	
			case '4': 
				combine_msg_all->mode = MODE_YAW;
				break;	
			case '5': 
				combine_msg_all->mode = MODE_FULL;
				break;

			/*****************************************************************
			* Toggling RAW and DMP_MODE
			******************************************************************/
			case '6': 
				//combine_msg_all->mode = MODE_RAW;
				if (raw_stat == true) 
				{
					combine_msg_all->msc_flag = RAW_NO_USE;
					raw_stat = false;
					//printf("masuk ke false");
				}
				else if (raw_stat == false) 
				{
					combine_msg_all->msc_flag = RAW_USE;
					raw_stat = true;
					//printf("masuk ke true");
				}
				break;
			/*****************************************************************/
			case '7': 
				combine_msg_all->mode = MODE_HEIGHT;
				break;	
			case 27: 
				combine_msg_all->mode = ESCAPE;
				break;

			default:
				break;
		}
	}	

	switch(c) {
	// additional
		case 'm': // start flexible logging 
			if (log_stat == true) 
			{
				combine_msg_all->msc_flag = LOG_NO_USE;
				log_stat = false;
			}
			else if (log_stat == false) {
				combine_msg_all->msc_flag = LOG_USE;
				log_stat = true;
			}
		break;
	}
	
}
