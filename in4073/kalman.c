#include "kalman.h"

int16_t p2phi(int16_t p) {
	if (p <= -16384) return -1476;
	else if (p >= 16384) return 1476;
	else
	{
		return p;
	}
}

void kalman(int16_t sp, int16_t sq, int16_t sax, int16_t say, uint16_t c1phi, uint16_t c2phi, uint16_t c1theta, uint16_t c2theta, 
	int16_t *estimated_p, int16_t *estimated_q, int16_t *estimated_phi, int16_t *estimated_theta, int16_t *bp, int16_t *bq) {
	int16_t ephi, etheta;

	*estimated_p = (sp - *bp);
	// *estimated_phi = *estimated_phi + ((*estimated_p*5683)>>19);
	*estimated_phi = *estimated_phi + (*estimated_p>>3);
	ephi = *estimated_phi - say;
	*estimated_phi = *estimated_phi - (ephi/c1phi);
	*bp = *bp + (ephi/c2phi);

	//*estimated_q = -(sq - *bq);
	*estimated_q = (sq - *bq);
	//*estimated_theta = *estimated_theta + ((*estimated_q*5683)>>19);
	*estimated_theta = *estimated_theta + (*estimated_q>>3);
	etheta = *estimated_theta - sax;
	*estimated_theta = *estimated_theta - (etheta/c1theta);
	*bq = *bq + (etheta/c2theta);
}

int16_t iir_butter_10(int16_t NewSample) {

//NCoef 2
//DCgain 8

uint8_t n;
int16_t ACoef[3], BCoef[3];
int32_t y[3], x[3];

x[0]=0;x[1]=0;x[2]=0;
y[0]=0;y[1]=0;y[2]=0;

ACoef[0] = 8841;
ACoef[1] = 17682;
ACoef[2] = 8841;

BCoef[0] = 16384;
BCoef[1] = -18726;
BCoef[2] = 6763;

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

return (y[0] / 8) <<8;
}

int16_t iir_butter_15(int16_t NewSample) {

//NCoef 2
//DCgain 8

uint8_t n;
int16_t ACoef[3], BCoef[3];
int32_t y[3], x[3];

x[0]=0;x[1]=0;x[2]=0;
y[0]=0;y[1]=0;y[2]=0;

ACoef[0] = 8592;
ACoef[1] = 17184;
ACoef[2] = 8592;

BCoef[0] = 32768;
BCoef[1] = -24503;
BCoef[2] = 8919;

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

return (y[0] / 2);
}

int16_t iir_butter_20(int16_t NewSample) {

//NCoef 2
//DCgain 8

uint8_t n;
int16_t ACoef[3], BCoef[3];
int32_t y[3], x[3];

x[0]=0;x[1]=0;x[2]=0;
y[0]=0;y[1]=0;y[2]=0;

ACoef[0] = 13537;
ACoef[1] = 27075;
ACoef[2] = 13537;

BCoef[0] = 32768;
BCoef[1] = -12108;
BCoef[2] = 6416;

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

return (y[0] / 2);
}

int16_t iir_butter_25(int16_t NewSample) {

//NCoef 2
//DCgain 8

uint8_t n;
int16_t ACoef[3], BCoef[3];
int32_t y[3], x[3];

x[0]=0;x[1]=0;x[2]=0;
y[0]=0;y[1]=0;y[2]=0;

ACoef[0] = 9597;
ACoef[1] = 19195;
ACoef[2] = 9597;

BCoef[0] = 32768;
BCoef[1] = 0;
BCoef[2] = 5622;

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

return y[0];
}
