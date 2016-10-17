#include <inttypes.h>
#include <stdbool.h>


void comm_check(uint16_t comm_duration, uint32_t *total_dur, bool *update_flag)
{

	if(*update_flag == true)
	{
		//*total_dur = 0;
		*update_flag = false;
	}
	else
	{
		*total_dur += comm_duration;
	}

}