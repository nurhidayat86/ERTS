#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "term.h"
#include "../protocol.h"

#define MAX_THRUST_COM 8192
#define MIN_THRUST_COM 0
#define MAX_ATTITUDE_COM 8192
#define MIN_ATTITUDE_COM -MAX_ATTITUDE_COM

#define KEY_INC 64

extern bool stop_sending;

void KeyboardCommand(char c, struct msg_keyboard_t* keyboard_msg, struct msg_tuning_t* tuning_msg, struct msg_joystick_t* joystick_msg);

#endif /* _KEYBOARD_H_ */