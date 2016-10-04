#include "kalman.h"

void kalman_filter(int16_t sp, int16_t measured_phi, int16_t *p2phi, int16_t *estimated_p, int16_t *estimated_phi) {
	//setting up constant
	uint8_t e = 0;
	uint8_t b = 0;
	uint8_t c1 = 0;
	uint8_t c2 = c1 << 32 // times by 1024;

	estimated_p = sp - b;
	estimated_phi = estimated_phi + p*p2phi;
	estimated_phi = estimated_phi - (e/c1);
	b = b + (e/c2);
}

int16_t configure_p2phi(int16_t p, int16_t phi) {
	if (p == 0)
		return 1;
	else
		return phi/p;
}