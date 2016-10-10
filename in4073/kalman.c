


int16_t p2phi(int16_t p) {
	int16_t result;
	if (p <= -16384) return -1476;
	else if (p >= 16384) return 1476;
	else return P2PHI[p+16384];
}

void kalman(int16_t sp, int16_t sq, int16_t sr, int16_t sax, int16_t say, int16_t saz, int16_t measured_phi, int16_t measured_thetha, int16_t measured_psi, int16_t c1phi, int16_t c2phi, int16_t c1theta, int16_t c2theta, 
	int16_t c1psi, int16_t c2psi, *estimated_p, *estimated_q, *estimated_r, *estimated_phi, *estimated_theta, *estimated_psi, *bp, *bq, *br) {
	int16_t ephi, etheta, epsi;

	estimated_p = sp - bp;
	estimated_phi = estimated_phi + p2phi(sax);
	ephi = estimated_phi - measured_phi;
	estimated_phi = estimated_phi - (ephi/c1phi);
	bp = bp + (ephi/c2phi);

	estimated_q = sq - bq;
	estimated_theta = estimated_theta + p2phi(say);
	etheta = estimated_theta - measured_theta;
	estimated_theta = estimated_theta - (etheta/c1theta);
	bq = bq + (etheta/c2theta);

	estimated_r = sr - br;
	estimated_psi = estimated_psi + p2phi(saz);
	epsi = estimated_psi - measured_psi;
	estimated_psi = estimated_psi - (etheta/c1psi);
	br = br + (etheta/c2psi);
}

