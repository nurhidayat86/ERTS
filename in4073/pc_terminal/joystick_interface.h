#ifndef _JOYSTICK_INTERFACE_H
#define _JOYSTICK_INTERFACE_H

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "joystick.h"
#include "../protocol.h"

#define NAME_LENGTH 128
#define JOY_THRUST_OFF 32767

void init_joystick(int* fd);
void JoystickCommand(int fd, struct js_event js, struct msg_joystick_t* joystick_msg);

#endif /* _JOYSTICK_INTERFACE_H */