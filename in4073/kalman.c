#include "kalman.h"

int16_t p2phi(int16_t p) {
	if (p <= -16384) return -1476;
	else if (p >= 16384) return 1476;
	else
	{
		return p;
	}
}
/*************************************************************************************************************************************************
* Kalman filter
* p2phi = 5683 >>10
* frequency sampling = 256 hz arround 3.8 ms
* best performance at c1 = 64/128/256 (higher --> slower (more damping ratio))
* c2 = 1024*c1
* need to perform bitwise operation to make it faster
**************************************************************************************************************************************************/
void kalman(int16_t sp, int16_t sq, int16_t sax, int16_t say, uint16_t c1phi, uint16_t c2phi, uint16_t c1theta, uint16_t c2theta, 
	int16_t *estimated_p, int16_t *estimated_q, int16_t *estimated_phi, int16_t *estimated_theta) 
{
	int32_t temp_estimated_p,  temp_estimated_q,  temp_sp, temp_sq, temp_sax, temp_say;
	static int32_t bp, bq, ephi, temp_estimated_phi, temp_estimated_theta, etheta;

	temp_sp = sp<<16;
	temp_say = say<<16;
	temp_estimated_p = (temp_sp - bp);
	temp_estimated_phi = temp_estimated_phi + ((temp_estimated_p*177)>>12);
	ephi = temp_estimated_phi - temp_say;
	temp_estimated_phi = temp_estimated_phi - (ephi>>c1phi);
	bp = bp + (ephi>>c2phi);
	*estimated_phi = temp_estimated_phi >> 16;
	*estimated_p = temp_estimated_p >> 16;

	temp_sq = sq<<16;
	temp_sax = sax<<16;
	temp_estimated_q = (temp_sq-bq);
	temp_estimated_theta = temp_estimated_theta + ((temp_estimated_q*177)>>12);
	etheta = temp_estimated_theta - temp_sax;
	temp_estimated_theta = temp_estimated_theta - (etheta>>c1theta);
	bq = bq + (etheta>>c2theta);
	*estimated_theta = temp_estimated_theta >> 16;
	*estimated_q = temp_estimated_q >> 16;
}
/***************************************************************************************************************************************************/

int16_t iir_butter_fs256_fc10(int16_t NewSample) {

	// Noef 2
	// DCgain 64

	uint8_t n;
	int16_t ACoef[3], BCoef[3];
	static int32_t y[3], x[3];

	// b on matlab
	ACoef[0] = 13409;
	ACoef[1] = 26819;
	ACoef[2] = 13409;

	// a on matlab
	BCoef[0] = 16384;
	BCoef[1] = -27125;
	BCoef[2] = 11579;

	//shift the old samples
	for(n=2; n>0; n--) 
	{
	   x[n] = x[n-1];
	   y[n] = y[n-1];
	}

	//Calculate the new output
	x[0] = NewSample;
	y[0] = ACoef[0] * x[0];

	for(n=1; n<=2; n++)
	    y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];

	y[0] /= BCoef[0];

	return (y[0]>>6);
}

int16_t iir_butter_10_256_8b(int16_t NewSample) {
uint8_t n;
int16_t ACoef[3], BCoef[3];
static int32_t y[3], x[3];

ACoef[0] = 52;
ACoef[1] = 104;
ACoef[2] = 52;

BCoef[0] = 64;
BCoef[1] = -105;
BCoef[2] = 45;

//shift the old samples
for(n=2; n>0; n--) {
   x[n] = x[n-1];
   y[n] = y[n-1];
}

//Calculate the new output
x[0] = NewSample;
y[0] = ACoef[0] * x[0];

for(n=1; n<=2; n++)
    y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];

y[0] /= BCoef[0];

return (y[0] >>6);
}

int16_t iir_butter_10_256_16b(int16_t NewSample) {
uint8_t n;
int16_t ACoef[3], BCoef[3];
static int32_t y[3], x[3];

ACoef[0] = 13409;
ACoef[1] = 26819;
ACoef[2] = 13409;

BCoef[0] = 16384;
BCoef[1] = -27125;
BCoef[2] = 11579;

//shift the old samples
for(n=2; n>0; n--) {
   x[n] = x[n-1];
   y[n] = y[n-1];
}

//Calculate the new output
x[0] = NewSample;
y[0] = ACoef[0] * x[0];

for(n=1; n<=2; n++)
    y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];

y[0] /= BCoef[0];

return (y[0] >>6);
}