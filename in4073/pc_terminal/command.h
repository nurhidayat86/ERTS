#ifndef _COMMAND_H_
#define _COMMAND_H_

# include "../protocol.h"

#define PANIC_TIME 2*1000 // here in ms
#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }

void InitCommand(struct msg_combine_all_t* combine_msg_all);
void InitCommandAll(struct msg_joystick_t* joystick_msg, struct msg_keyboard_t* keyboard_msg, struct msg_combine_all_t* combine_msg_all);

void InitCommandUpdate(struct msg_combine_all_t* combine_msg_all);

void CommandUpdate(struct msg_combine_all_t* combine_msg_all);

void CommandModeSafe(struct msg_combine_all_t* combine_msg_all);
void CommandModeSafeAll(struct msg_joystick_t* joystick_msg, struct msg_keyboard_t* keyboard_msg, struct msg_combine_all_t* combine_msg_all);

void SendCommandAll(struct msg_combine_all_t* combine_msg_all);

void CombineCommand(struct msg_combine_all_t* combine_msg_all);
void CombineCommandAll(struct msg_joystick_t* joystick_msg, struct msg_keyboard_t* keyboard_msg, struct msg_combine_all_t* combine_msg_all);


#endif /* _COMMAND_H_ */