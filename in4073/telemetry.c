#include "in4073.h"

void fill_telemetry() {
	telemetry_msg.telemetry_time = get_time_us();
	telemetry_msg.control_mode = control_mode;
	telemetry_msg.thrust = thrust;
	telemetry_msg.roll = roll;
 	telemetry_msg.pitch = pitch;
 	telemetry_msg.yaw = yaw;
 	telemetry_msg.engine[0] = ae[0];
 	telemetry_msg.engine[1] = ae[1];
 	telemetry_msg.engine[2] = ae[2];
 	telemetry_msg.engine[3] = ae[3];
 	telemetry_msg.update = true;
}

void send_telemetry() {
	printf("control_mode: %6d| thrust: %6d| roll: %6d| pitch: %6d| yaw: %6d| engine[0]: %6d| engine[1]: %6d| engine[2]: %6d| engine[3]: %6d|\n",
		telemetry_msg.control_mode, telemetry_msg.thrust, telemetry_msg.roll, telemetry_msg.pitch, telemetry_msg.yaw, telemetry_msg.engine[0], telemetry_msg.engine[1],
		 telemetry_msg.engine[2], telemetry_msg.engine[3]);
}