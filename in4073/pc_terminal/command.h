#ifndef _COMMAND_H_
#define _COMMAND_H_

# include "../protocol.h"

#define PANIC_TIME 2*1000 // here in ms
#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }

void InitCommand(struct msg_combine_all_t* combine_msg_all, struct msg_combine_t* combine_msg, struct msg_keyboard_t* keyboard_msg, struct msg_joystick_t* joystick_msg, struct msg_tuning_t* tuning_msg);
void CombineCommand(struct msg_combine_all_t* combine_msg_all, struct msg_combine_t* combine_msg, struct msg_keyboard_t* keyboard_msg, struct msg_joystick_t* joystick_msg, struct msg_tuning_t* tuning_msg);
void SendCommand(struct msg_combine_t* combine_msg);
void SendCommandTuning(struct msg_tuning_t* tuning_msg);
void SendCommandAll(struct msg_combine_all_t* combine_msg_all);

#endif /* _COMMAND_H_ */