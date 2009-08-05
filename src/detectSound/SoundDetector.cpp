#include "SoundDetector.h"


SoundDetector::SoundDetector()//Class constructor
{	
	
	sound=new YARPChicoEars("/ears","default","/soundgrab/o:sound");
	sound->init();
	file_nr=0;
}


SoundDetector::~SoundDetector()//Class Destructor
{
	sound->close();
}

int SoundDetector::detect_features()
{
	
	double left_ear[2048], right_ear[2048];
	double ITD, notch_frequency, features[2];
	double Threshold_left;
	double Threshold_right;
	int nr_of_loops=1;

	int features_detected=0;


	features[0]=0.0;
	features[1]=0.0;

	//printf("entering detect_features");

	for(int loops=0;loops<nr_of_loops;loops++)
	{
				sound->grab();								//to grab sound from the remote terminal
				
				sound->separateEars(left_ear, right_ear);	//separate the sound samples from the two ears
				


				

				Threshold_left = 0.0;
				Threshold_right = 0.0;
					
				for (int j=0; j<2000; j++)
				{
					Threshold_left = Threshold_left + pow(left_ear[j], 2);
					Threshold_right = Threshold_right + pow(right_ear[j], 2);
				}

				Threshold_left = Threshold_left/2000;
				Threshold_right= Threshold_right/2000;
			//	printf("\nThreshold_left=%f, Threshold_right=%f\n", Threshold_left, Threshold_right);


			
	if((Threshold_left < 0.00005) || (Threshold_right < 0.00005))
	{
		features_detected=0;
	}
	else
	{
			ITD=featureextractor.ITD(left_ear,right_ear);
			notch_frequency=featureextractor.notch(left_ear, right_ear);
			if(notch_frequency==13230 || notch_frequency==17640)
			{
				features_detected=0;
			}
			else
			{
				features_detected=1;
			}

	}
	//	}while(notch_frequency==13230 || notch_frequency==17640);

		features[0]+=ITD;
		features[1]+=notch_frequency;
	}

	

	feature[0] = features[0]/nr_of_loops;
	feature[1] = features[1]/nr_of_loops;


	return features_detected;
}

int SoundDetector::save_sound()
{
	
	double left_ear[2048], right_ear[2048];
//	sound->grab();								//to grab sound from the remote terminal
	sound->separateEars(left_ear, right_ear);	//separate the sound samples from the two ears

	//writing sound for debug
	FILE *Chan0;
	FILE *Chan1;
	char filename[50];

	file_nr++;

	sprintf(filename, "sound_left%d.txt", file_nr);
	Chan0 = fopen( filename, "w" );

	sprintf(filename, "sound_right%d.txt", file_nr);
	Chan1 = fopen( filename, "w" );
		
	int i;
	for (i=0;i<2048;i++)
        fprintf(Chan0, "%1.6f\n ", left_ear[i]);
        

    for (i=0;i<2048;i++)
        fprintf(Chan1, "%1.6f \n", right_ear[i]);

	fclose(Chan0);
	fclose(Chan1);


	return 0;
}

