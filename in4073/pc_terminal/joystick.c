/*------------------------------------------------------------------
 *  joystick.c
 *  joystick interface
 *
 *------------------------------------------------------------------
 */

#include "joystick_interface.h"

/*------------------------------------------------------------------
 *  Generic example code
 *  function: Joystick initiation
 *	adapted from: Generic example code
 -------------------------------------------------------------------
*/

void init_joystick(int* fd)
{
	unsigned char axes = 2;
	unsigned char buttons = 2;
	int version = 0x000800;
	char name[NAME_LENGTH] = "Unknown";

	if ((*fd = open("/dev/input/js1", O_RDONLY)) < 0) 
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
/*-------------------------------------------------------------------
*/
