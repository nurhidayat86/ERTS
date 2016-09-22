/*------------------------------------------------------------------
 *  joystick.c
 *  joystick interface
 *
 *------------------------------------------------------------------
 */

#include "joystick_interface.h"

void init_joystick(int* fd)
{
	unsigned char axes = 2;
	unsigned char buttons = 2;
	int version = 0x000800;
	char name[NAME_LENGTH] = "Unknown";

	if ((*fd = open("/dev/input/js0", O_RDONLY)) < 0) 
	{
		perror("joystick");
		exit(1);
	}

	ioctl(*fd, JSIOCGVERSION, &version);
	ioctl(*fd, JSIOCGAXES, &axes);
	ioctl(*fd, JSIOCGBUTTONS, &buttons);
	ioctl(*fd, JSIOCGNAME(NAME_LENGTH), name);

	printf("Joystick (%s) has %d axes and %d buttons. Driver version is %d.%d.%d.\n",
		name, axes, buttons, version >> 16, (version >> 8) & 0xff, version & 0xff);
	printf("Testing ... (interrupt to exit)\n");

	fcntl(*fd, F_SETFL, O_NONBLOCK);
}

void JoystickCommand(int fd, struct js_event js, struct msg_joystick_t* joystick_msg)
{
	// int axis[4];
	// char button[8];
	// while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
	// 	button[js.number] = js.value;
	// 	axis[js.number] = js.value;
	 	//printf("before assigning \n");

		//here
		////mapping to output
		// joystick_msg.roll = axis[0];
		// joystick_msg.pitch = axis[1];
		// joystick_msg.yaw = axis[2];
		// joystick_msg.thrust = axis[3];
		// joystick_msg.mode = (button[0] << 7) | (button[1] << 6) | (button[2] << 5) | (button[3] << 4) | (button[4] << 3) | (button[5] << 2) | (button[6] << 1) | (button[7]);
		// joystick_msg.update = true;
	//}
}