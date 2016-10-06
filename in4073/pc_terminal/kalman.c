void kalman(int16_t sp, int16_t sphi, int16_t *p, int16_t *phi,)
{
	int16_t b=0,p2phi=1;
	int16_t c1;
	int16_t c2=1000*c1;

	p = sp - b;
	phi = phi + p*P2PHI;
	e = phi - sphi;
	phi = phi - e/c1;
	b = b + e/c2;
}