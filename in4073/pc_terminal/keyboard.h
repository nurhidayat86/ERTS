#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "term.h"
#include "../protocol.h"

#define MAX_THRUST 65535
#define MIN_THRUST 0
#define MAX_ATTITUDE 32767
#define MIN_ATTITUDE -MAX_ATTITUDE

enum control_mode_t {
  MODE_SAFE,
  MODE_PANIC,
  MODE_MANUAL,
  MODE_CALIBRATION,
  MODE_YAW,
  MODE_FULL,
  MODE_RAW,
  MODE_HEIGHT,
  ESCAPE
};

void KeyboardCommand(char c, struct msg_keyboard_t* keyboard_msg);

#endif /* _KEYBOARD_H_ */