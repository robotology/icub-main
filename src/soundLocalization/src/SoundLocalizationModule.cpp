// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/SoundLocalizationModule.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace iCub::contrib;

SoundLocalizationModule::SoundLocalizationModule(){

	Fs=44100;

	map[0] = 120.3;//1.1;
	map[1] = 0.0; 
	map[2] = 0.0;
	map[3] = 0.0; 
	map[4] = 0.03;
	map[5] = -320; //-5.50;

	count=0;
	count2=0;
	numsamples=2000;
	normalizedDoubleSamplesChan0 = (double*) malloc(sizeof(double) * (8192/4));
	normalizedDoubleSamplesChan1 = (double*) malloc(sizeof(double) * (8192/4));
	fft = new YARPFft(numsamples, numsamples);
}

SoundLocalizationModule::~SoundLocalizationModule(){ 
	free(normalizedDoubleSamplesChan0);
	free(normalizedDoubleSamplesChan1);

}


bool SoundLocalizationModule::open(Searchable& config){
   
    if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --file configFile.ini\n");
        return false;
    }

    // Configuration Example:
    // do like this to read configuration values:
    // First param: Key to look for in config object,
    // Second param: Default value to use if key not found in config object
    // Third param: A comment about the configuration parameter (will be printed automatically to terminal on module startup)
    double example = config.check("horizontalViewAngle",
                                Value(360.0),
                                "Width of input saliency map corresponds to how many degrees? (double)").asDouble();
    _levelthresh = config.check("levelthresh", 
		                        Value(0.001),
                                "Threshold to discard low-power sounds.(double)").asDouble();

	_tiltvarlim = config.check("tiltvarlim",
                                Value(6.0),
                                "Threshold to discard high variance tilt estimates (double)").asDouble();

    bool ok = true;
    ok &= _prtVctLoc.open(getName("o:loc"));
    ok &= _prtVctLoc2.open(getName("o:loc2"));
    ok &= _prtSoundIn.open(getName("i:sound"));
    _configPort.open(getName("conf"));
    attach(_configPort, true);
    return ok;
}

bool SoundLocalizationModule::close(){
    
    // delete here any allocated data (like a deconstructor)
    // delete [] myDataArray
    _prtVctLoc.close();
    _prtVctLoc2.close();
    _prtSoundIn.close();
    return true;
}

bool SoundLocalizationModule::interruptModule(){
    _prtVctLoc.interrupt();
    _prtVctLoc2.interrupt();
    _prtSoundIn.interrupt();
    return true;
}

bool SoundLocalizationModule::updateModule(){

    // this loop is repeatedly called (as fast as possible) as long as it returns true 
  
    // this reads sound from the sound port
    // if no sound available the method is blocking
    Sound *sound = _prtSoundIn.read();

//	cout << "channels: " << sound->getChannels() << endl;

    // in case of module closing seems to happen sometimes (we just restart the loop if sound == NULL)
    if (sound == NULL)
        return true;
    // get a reference to a vector managed by the port
    // this is the vector streamed by the next _prtVctLoc.write() call
    VectorOf<double> &vctLoc = _prtVctLoc.prepare();
    VectorOf<double> &vctLoc2 = _prtVctLoc2.prepare();

    // balbiblabu do stuff here, call sound localization class ...
	int i,j;
	double val;

	// Separate sound from the two ears

	double left_ear[2048], right_ear[2048];

	for (i=0;i<(8192/4);i++)
	{
		for(j = 0; j < sound->getChannels(); j++)
		{
			val=(double)sound->get(i,j)/32768;
			if(j==0) {
				normalizedDoubleSamplesChan0[i]=val;
				//fprintf(outFile, "%1.6f ", val);
			}
			else {
				normalizedDoubleSamplesChan1[i]=val;
				//fprintf(outFile, "%1.6f ", val);
			}
			
		}
	//	fprintf(outFile, "\n");
	}
	memcpy(right_ear,normalizedDoubleSamplesChan1,sizeof(double)*(8192/4));
	memcpy(left_ear,normalizedDoubleSamplesChan0,sizeof(double)*(8192/4));



//Read test sound

//	ReadSoundFromFile(left_ear, "C:\\iCub\\src\\soundlocalization\\testsound\\chico\\sound_left45_-45.txt", 2048);
//	ReadSoundFromFile(right_ear, "C:\\iCub\\src\\soundlocalization\\testsound\\chico\\sound_right45_-45.txt", 2048);



	// Calculate intensity

	double intensity_left = 0.0;
	double intensity_right = 0.0;
					
	for (int j=0; j<2000; j++)
	{
		intensity_left = intensity_left + pow(left_ear[j], 2);
		intensity_right = intensity_right + pow(right_ear[j], 2);
	}

	intensity_left = intensity_left/2000;
	intensity_right= intensity_right/2000;

	//if intensity is too low we shouldn't waste time trying to calculate features...
	//intesity > 0.01
	cout << "Intensity left : " << intensity_left << endl;
	cout << "Intensity right: " << intensity_right<< endl;

	//must have something in both ears
	if((intensity_left > _levelthresh/10.0) && (intensity_right > _levelthresh/10.0))
	{

		//Save sound to file for testing
/*		count++;
		char file_name[255];
		sprintf(file_name, "C:\\iCub\\src\\soundlocalization\\testsound\\test_left%d.txt", count);
		SaveSoundToFile(left_ear, file_name, 2048);
		sprintf(file_name, "C:\\iCub\\src\\soundlocalization\\testsound\\test_right%d.txt", count);
		SaveSoundToFile(right_ear, file_name, 2048);
*/
		// Calculate features
		double var_pan;//=5.0;
        double var_tilt;//=10.0;
		double time_difference, notch_frequency;

		if(ITD(left_ear,right_ear, time_difference, var_pan))
		{

			notch(left_ear, right_ear, notch_frequency, var_tilt);

			//		cout << "Time difference: " << time_difference << endl;
			//		cout << "Notch frequency: " << (int)notch_frequency << endl;
	
			// Map features to position
			
			double pan, tilt;
			pan = -(map[0]*time_difference + map[1]*notch_frequency + map[2]);
			var_pan=var_pan*sqrt(map[0]);
			tilt =map[3]*time_difference + map[4]*notch_frequency + map[5];
			var_tilt=var_tilt*sqrt(map[4]);
			if(var_tilt> _tiltvarlim )
				tilt=0;
	
			double intensity=(intensity_left+intensity_right)/2;
			cout << "pan : " << pan << endl;
			cout << "pan var : " << var_pan << endl;
			cout << "tilt: " << tilt << endl;
			cout << "tilt var: " << var_tilt << endl;
			cout << "intensity: " << intensity << endl; 
			//0.01
			
			if(intensity>_levelthresh)
			{
				cout << "Sending position..." << endl;
				vctLoc.resize(5);
				vctLoc[0] = 1.5*pan;
				vctLoc[1] = 5*var_pan;
				vctLoc[2] = 1.5*tilt;
				vctLoc[3] = 5*var_tilt;
				vctLoc[4] = intensity;

/*
				vctLoc.resize(8);
				vctLoc[0] = tilt;
				vctLoc[1] = 0;
				vctLoc[2] = pan;
				vctLoc[3] = 0;
				vctLoc[4] = 0;
				vctLoc[5] = 0;
				vctLoc[6] = 0;
				vctLoc[7] = 0;
*/
				count2++;
				vctLoc2.resize(5);
				vctLoc2[0] = 1.5*pan;
				vctLoc2[1] = 1.5*tilt;
				vctLoc2[2] = 'r';
				vctLoc2[3] = 0;
				vctLoc2[4] = count2;


				_prtVctLoc.write();
				_prtVctLoc2.write();

				// changed by pase since it won't compile (it was "Sleep(5000)"
				yarp::os::Time::delay(5.0);
				cout << "READY " << endl;
			}
		}
	}
    return true;
}

bool SoundLocalizationModule::ILD(double *left_ear, double *right_ear, double &ild)//InterAural Level Difference
{
	
	double sum1=0, sum2=0;
	for(int i=0; i<numsamples; i++)
	{
		sum1=sum1+left_ear[i]*left_ear[i];
		sum2=sum2+right_ear[i]*right_ear[i];
	}
	ild=sum1-sum2;
	
	return true;
}



bool SoundLocalizationModule::ITD(double *left_ear, double *right_ear, double &itd, double &var)//InterAural Time Difference
{
	int i;
//	double itd;
	double CrossCorr[4000];
	cross_corr(left_ear, right_ear, CrossCorr); //arrays are left_ear=having the samples from the left ear
	//right_ear having samples from the right ear
	//CrossCorr array has the final cross correlation for the two ears
	i=maximum(CrossCorr);//i is the index in the cross correlation array having the maximum absolute value
//	printf("\nITD = %d\n",i);
	if (i<1000)
		itd = (double)(i/44.1);
	else
		itd=(double)((i-2000)/44.1);
//	printf("%fms \n",itd);

	//calculate variance

	double s2=0.0;
	double n=0.0;
	int index_low=i-30;
	int index_high=i+30;
	int i2;
	for (int l=index_low; l<index_high; l++)
	{
		i2=l;
		if(i2<0)
			i2=2000+l;

        s2=s2+(l-i)*(l-i)*fabs(CrossCorr[i2]);
		n=n+fabs(CrossCorr[i2]);
	}

	double epsilon=0.000000001;
	if(n<epsilon)
		return false;

	s2=s2/n;
	var=sqrt(s2);

	var=var/44.1;

	return true;
}



int SoundLocalizationModule::maximum(double *CrossCorr)	//finds the index from the cross correlation array having the maximum value
{
	double max_value=0.0;
	int index=0;
//	max_value=0.0; //CrossCorr[0];
//	index=0;
//	for(int i=1; i<numsamples; i++)
	for(int i=0; i<=25; i++)
	{
		if(max_value<CrossCorr[i])
		{
			max_value=CrossCorr[i];
			index=i;
		}//if
	}//for

	for(int i=numsamples-1; i>=numsamples-25; i--)
	{
		if(max_value<CrossCorr[i])
		{
			max_value=CrossCorr[i];
			index=i;
		}//if
	}//for

	return index;
}



void SoundLocalizationModule::cross_corr(double *left_ear, double *right_ear, double *CrossCorr)	//the output array CrossCorr has the 
//absolute value(modulus) of cross correlation
//and not the value of the cross corelation function
{																	
	
	double Re_left[2000], Im_left[2000], Re_right[2000], Im_right[2000];
	
	for (int i=0; i<numsamples; i++)
	{
		Re_left[i]=left_ear[i];
		Im_left[i]=0.0;
		Re_right[i]=right_ear[i];
		Im_right[i]=0.0;
	}

/*	for (i=numsamples; i<2*numsamples; i++)
	{
		Re_left[i]=0.0;
		Im_left[i]=0.0;
		Re_right[i]=0.0;
		Im_right[i]=0.0;
	}
*/	
	int j=1;
	
	int dim[1];
	dim[0] = 2000;
	//for documentation on fft refer to YARPFft.cpp
	fft->Fft(1, dim, Re_left, Im_left, 1,-1);//take fourier transform of the two signal arrays
	fft->Fft(1, dim, Re_right, Im_right, 1,-1);
	
	
	double Corr_Re[2000], Corr_Im[2000];
	//cross correlation = fft(left ear signal) * conj(fft(right ear signal))
	for (int i=0; i<numsamples; i++)
	{
		Corr_Re[i] = Re_left[i]*Re_right[i] + Im_left[i]*Im_right[i];
		Corr_Im[i] = Im_left[i]*Re_right[i] - Re_left[i]*Im_right[i];
	}
	
	
	fft->Fft(1, dim, Corr_Re, Corr_Im, -1,-1);	//now cross correlation = ifft(fft1*conj(fft2))
	
	for(int i=0; i<numsamples; i++)
	{
//		CrossCorr[i]=fabs(Corr_Re[i]);
		CrossCorr[i]=fabs(Corr_Re[i]);
	}
}


bool SoundLocalizationModule::notch(double *left_ear, double *right_ear, double &freq, double &var)//calculate the position of notches
{																	//returns the frequency at which the notch occurs
	const int R=290;
	const int N=2000-R;
//	double Re_left[2000], Im_left[2000], Re_right[2000], Im_right[2000];
	double Re_left[N], Im_left[N], Re_right[N], Im_right[N];
	double fft_right[2000], fft_left[2000];
	double ISD[2000];

	//Calculate the interaural spectral difference

	for(int i=0; i<2000; i++)
		ISD[i]=0;

	for (int j=0; j<R; j++)
	{
		for (int i=0; i<N; i++)
		{
			Re_left[i]=left_ear[i+j];
			Im_left[i]=0.0;
			Re_right[i]=right_ear[i+j];
			Im_right[i]=0.0;
		}
	
		int dim[1];
		dim[0] = N;
		
		fft->Fft(1, dim, Re_left, Im_left, 1,-1);//take fourier transform of the two signal arrays
		fft->Fft(1, dim, Re_right, Im_right, 1,-1);

		//since it is inplace fourier calculation we have not sent the original arrays left_ear and right_ear but copied them
		//to another array as it can be required later
	
		double epsilon=0.000000001;
		for (int i=0; i<N; i++)
		{
			fft_right[i]= sqrt(pow(Re_right[i], 2) + pow(Im_right[i], 2)); //Get absolute value
			if(fft_right[i]<epsilon)
				fft_right[i]=epsilon;
			fft_left[i]=sqrt(pow(Re_left[i], 2) + pow(Im_left[i], 2));
			if(fft_left[i]<epsilon)
				fft_left[i]=epsilon;
//			ISD[i]=ISD[i]+fft_left[i]/fft_right[i];
			ISD[i]=ISD[i]+fft_right[i]/fft_left[i];
		}

	}

	for(int i=0; i<N; i++)
		ISD[i]=ISD[i]/R;


	//We expect a notch between 8000 and 13000 Hz
	//Do a gaussian fitting 

	int index_low=(int)(N*8000/Fs);
    int index_high=(int)(N*13000/Fs);
    
	double mu=0.0;
    double s2=0.0;
	int n=0;

    for (int i=index_low; i<index_high; i++)
	{
        mu=mu+i*(int)(ISD[i]+0.5);
        n=n+(int)(ISD[i]+0.5);
	}
    mu=mu/n;

    int diff=(int)(N*1750/Fs);
	index_low=(int)mu-diff;
    index_high=(int)mu+diff;
    
	mu=0.0;
    n=0;

    for (int i=index_low; i<index_high; i++)
	{
        mu=mu+i*(int)(ISD[i]+0.5);
        n=n+(int)(ISD[i]+0.5);
	}
    mu=mu/n;

	for (int i=index_low; i<index_high; i++)
	{
        s2=s2+(i-mu)*(i-mu)*(int)(ISD[i]+0.5);
	}
    s2=s2/(n-1);

	mu=mu/N*Fs;

	freq=mu;
	var=sqrt(s2);

	return true;
}

bool SoundLocalizationModule::ReadSoundFromFile(double *ear, string file_name, int nr_of_samples)
{
	ifstream file;

	file.open(file_name.c_str());

	if(!file)
	{
		cout << "Error: Couldn't open file: " << file_name;
		return false;
	}

	double x;
	int n=0;

	while(file>>x && n<nr_of_samples)
	{
		ear[n]=x;
//		cout << x << endl;
		n++;
	}

	if(n<nr_of_samples)
	{
		cout << "Error: Found EOF at pos: " << n;
		return false;
	}

	return true;
}

bool SoundLocalizationModule::SaveSoundToFile(double *ear, char *file_name, int nr_of_samples)
{ 

	FILE *fp1;
	if( (fp1 = fopen( file_name, "w" )) == NULL )
	{
		printf( "Couln't open file\n" );
		return false;
	}
	
	for(int i=0; i<nr_of_samples;i++)
	{
		fprintf(fp1,"%0.6f ",ear[i]);	//for
	}

	fclose(fp1);

	return true;
}
