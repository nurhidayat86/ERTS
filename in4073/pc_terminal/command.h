#ifndef _COMMAND_H_
#define _COMMAND_H_

# include "../protocol.h"

#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }

void CombineCommand(struct msg_combine_t* combine_msg, struct msg_keyboard_t* keyboard_msg, struct msg_joystick_t* joystick_msg);

#endif /* _COMMAND_H_ */