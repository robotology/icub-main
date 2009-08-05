#ifndef __SYMBOLICMODELHH__
#define __SYMBOLICMODELHH__

void computeTendonsLength(double &d1, double &d2, double &d3, double Roll, double Pitch);
void computeModifiedPitchRoll(double Yaw, double Roll, double Pitch, double &Roll_hat, double &Pitch_hat);
void computeOriginalPitchRoll(double Yaw, double &Roll, double &Pitch, double Roll_hat, double Pitch_hat);

#endif //h