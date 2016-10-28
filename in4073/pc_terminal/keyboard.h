#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "term.h"
#include "../protocol.h"

#define MAX_THRUST_COM 8192
#define MIN_THRUST_COM 0
#define MAX_ATTITUDE_COM 1024
#define MIN_ATTITUDE_COM -MAX_ATTITUDE_COM

#define KEY_INC 16

extern bool stop_sending;
bool raw_stat;
bool log_stat;

void KeyboardCommandSplit(char c, struct msg_joystick_t* joystick_msg, struct msg_keyboard_t* keyboard_msg);
void KeyboardCommand(char c, struct msg_combine_all_t* combine_msg_all);
void initraw_stat(); //initiate raw_stat value

#endif /* _KEYBOARD_H_ */