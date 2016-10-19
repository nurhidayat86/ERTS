#include <inttypes.h>
#include <stdio.h>

int16_t p2phi(int16_t p);
//void kalman(int16_t sp, int16_t sq, int16_t sax, int16_t say, int16_t measured_phi, int16_t measured_thetha, uint16_t c1phi, uint16_t c2phi, uint16_t c1theta, uint16_t c2theta, 
//	int16_t *estimated_p, int16_t *estimated_q, int16_t *estimated_phi, int16_t *estimated_theta, int16_t *bp, int16_t *bq);
void kalman(int16_t sp, int16_t sq, int16_t sax, int16_t say, uint16_t c1phi, uint16_t c2phi, uint16_t c1theta, uint16_t c2theta, 
	int16_t *estimated_p, int16_t *estimated_q, int16_t *estimated_phi, int16_t *estimated_theta, int16_t *bp, int16_t *bq);

int16_t iir_butter_fs256_fc10(int16_t NewSample);
int16_t iir_butter_10(int16_t NewSample);
int16_t iir_butter_15(int16_t NewSample);
int16_t iir_butter_20(int16_t NewSample);
int16_t iir_butter_25(int16_t NewSample);
