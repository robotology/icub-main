#include "PositionEstimator.h"

class SoundSourceEstimation
{
	private:
		PositionEstimator pos;
		double pan,tilt;
		double map[6];

	public:
		SoundSourceEstimation();
		~SoundSourceEstimation();

		int control(double ITD, double notch);
		int set_map(double new_map[]);
		double get_pan();
		double get_tilt();

};
