

#include <stdio.h>
#include <cstdlib>
#include <stdlib.h>

//#include <conio.h> // Windows specific, not needed

#include <math.h>
#include <time.h>

#include "Sensor.h"
#include "FeatureExtractor1.h"
#include "YARPChicoEars.h"


class SoundDetector:public Sensor
{
	private:
		FeatureExtractor1 featureextractor;
		YARPChicoEars* sound;
		int file_nr;

	public:
		SoundDetector();
		~SoundDetector();
		int detect_features();
		int save_sound();
};
