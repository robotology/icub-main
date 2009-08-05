#include "FeatureExtractor1.h"


/*****************************************
******************************************
**	FIND TILT AND PAN					**
**  "KEEP INITIAL VALUES OF TILT AND PAN**
**  AS ZERO"							**
******************************************
*****************************************/


FeatureExtractor1::FeatureExtractor1()
{
	numsamples = 2000;//the no. of samples for each ear that we acquire through yarp
//	dim[0]=4000;
	fft = new YARPFft(numsamples, numsamples);

/*	if( (fp_ISD = fopen( "features.txt", "a+" )) == NULL )
		printf( "The file 'features.txt' was not opened\n" );
	else
		printf( "The file 'features.txt' was opened\n" );
	
*/				

}


FeatureExtractor1::~FeatureExtractor1()   //class destructor
{
//	fclose(fp_ISD);
}


double FeatureExtractor1::ILD(double *left_ear, double *right_ear)//InterAural Level Difference
{
	
	double ild, sum1=0, sum2=0;
	for(int i=0; i<numsamples; i++)
	{
		sum1=sum1+left_ear[i]*left_ear[i];
		sum2=sum2+right_ear[i]*right_ear[i];
	}
	ild=sum1-sum2;
	
	return ild;
}



double FeatureExtractor1::ITD(double *left_ear, double *right_ear)//InterAural Time Difference
{
	int i;
	double itd;
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
	return itd;
}



int FeatureExtractor1::maximum(double *CrossCorr)	//finds the index from the cross correlation array having the maximum value
{
	double max_value;
	int index;
	max_value=CrossCorr[0];
	index=0;
	for(int i=1; i<numsamples; i++)
	{
		if(max_value<CrossCorr[i])
		{
			max_value=CrossCorr[i];
			index=i;
		}//if
	}//for
	
	return index;
}



void FeatureExtractor1::cross_corr(double *left_ear, double *right_ear, double *CrossCorr)	//the output array CrossCorr has the 
//absolute value(modulus) of cross correlation
//and not the value of the cross corelation function
{																	
  int i; // deal with MSVC 6 scoping problem
	double Re_left[2000], Im_left[2000], Re_right[2000], Im_right[2000];
	
	for (i=0; i<numsamples; i++)
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
	for (i=0; i<numsamples; i++)
	{
		Corr_Re[i] = Re_left[i]*Re_right[i] + Im_left[i]*Im_right[i];
		Corr_Im[i] = Im_left[i]*Re_right[i] - Re_left[i]*Im_right[i];
	}
	
	
	fft->Fft(1, dim, Corr_Re, Corr_Im, -1,-1);	//now cross correlation = ifft(fft1*conj(fft2))
	
	for(i=0; i<numsamples; i++)
//		CrossCorr[i]=fabs(Corr_Re[i]);
	CrossCorr[i]=Corr_Re[i];
}




int FeatureExtractor1::polyfit(double *ISD)//polynomial Curve Fitting
{
	
	double a_data[1000];
	
	double mean=500.5, std=288.8194;
	int i, j;
	double chisq;
	const int NoX=1000;
	const int NoF=13;
	double F[NoF];//F contains the final coefficients for the approximation
	

	gsl_matrix *X, *cov;
	gsl_vector *y, *w, *c;
	gsl_multifit_linear_workspace *work = gsl_multifit_linear_alloc (NoX, NoF);
	X = gsl_matrix_alloc (NoX, NoF);
	y = gsl_vector_alloc (NoX);
	w = gsl_vector_alloc (NoX);
	c = gsl_vector_alloc (NoF);
	cov = gsl_matrix_alloc (NoF, NoF);
	
	
	
	for (i=0; i<1000; i++)
	{
		a_data[i]=((i+1)-mean)/std;
	}
	
	
	for (i=0; i<NoX; i++) {
		for (j=0; j<NoF; j++) {
			gsl_matrix_set (X, i, j, pow(a_data[i], j));
		}
		gsl_vector_set (y, i, ISD[i]);
	}
	
	gsl_multifit_linear (X, y, c, cov, &chisq, work);
	gsl_multifit_linear_free (work);
	
	for (i=0; i<NoF; i++) {
		F[i] = gsl_vector_get (c, i);
	}		
	

	for (i=0; i<1000; i++) 
	{
		ISD[i]=0;		
		for (j=0; j<13; j++)
			ISD[i]=ISD[i]+F[j]*pow(a_data[i],j);		//the array ISD finally contains the values approximated with the polynomial approximation
	}  	

	return 0;


}




double FeatureExtractor1::notch(double *left_ear, double *right_ear)//calculate the position of notches
{		
  // Deal with MSVC 6 scoping problem -paulfitz
  int i;
															//returns the frequency at which the notch occurs
	double max_val, fft_right[2000], fft_left[2000], notch_freq;
	double ISD[2000];
	double Re_left[2000], Im_left[2000], Re_right[2000], Im_right[2000];
	
	for (i=0; i<numsamples; i++)
	{
		Re_left[i]=left_ear[i];
		Im_left[i]=0.0;
		Re_right[i]=right_ear[i];
		Im_right[i]=0.0;
	}
	
	int dim[1];
	dim[0] = 2000;
	fft->Fft(1, dim, Re_left, Im_left, 1,-1);//take fourier transform of the two signal arrays
	fft->Fft(1, dim, Re_right, Im_right, 1,-1);
	//since it is inplace fourier calculation we have not sent the original arrays left_ear and right_ear but copied them
	//to another array as it can be required later
	
	for (i=0; i<numsamples; i++)
	{
		fft_right[i]= sqrt(pow(Re_right[i], 2) + pow(Im_right[i], 2));//Get absolute value
		fft_left[i]=sqrt(pow(Re_left[i], 2) + pow(Im_left[i], 2));
		ISD[i]=	(10*log(fft_right[i])-10*log(fft_left[i]))/log(10);
	}
	
	FILE *fp1;
	if( (fp1 = fopen( "ISD_nofit.txt", "w" )) == NULL )
		printf( "The file 'ISD.txt' was not opened\n" );
	else
		printf( "The file 'ISD.txt' was opened\n" );
	
	for(i=0; i<1000;i++)
		fprintf(fp1,"%0.3f ",ISD[i]);	//for

	fclose(fp1);
	

	//do polynomial curve fitting
	int a;
	a=polyfit(ISD);
///	for (int b=0; b<1000; b++)
///		fprintf(fp_ISD, "%.2f\n",ISD[b]);

//////// Open for write 
	FILE *fp;
	if( (fp = fopen( "ISD.txt", "w" )) == NULL )
		printf( "The file 'ISD.txt' was not opened\n" );
	else
		printf( "The file 'ISD.txt' was opened\n" );
	
	for(i=0; i<1000;i++)
		fprintf(fp,"%0.3f ",ISD[i]);	//for

	fclose(fp);
	

	max_val=ISD[600];
	
	int index_max=600;
	
	
	//find the position of maxima in terms of sample no.
	for(int k=600; k<800; k++)
	{
		if(max_val<ISD[k])
		{
			max_val=ISD[k];
			index_max=k;
		}//if

	}//for 
	
//printf("max_value = %.2f	",max_val);
/*
	int index_min=600;
	min_val=ISD[600];
	for(k=600; k<800; k++)
	{
		if(min_val>ISD[k])
		{
			min_val=ISD[k];
			index_min=k;
		}//if

	}//for 
*/
  
//	printf("min_value = %.2f\n",min_val);

//	if ((max_val-min_val)<2)
//		notch_freq = -1;
//	else
//		notch_freq=(index_max+1)*22050/1000;

//	index_max = index_max+1-700;
//	printf("index_max = %d\n",index_max);
//	notch_freq=0.0;
//	notch_freq=(double)(index_max/100.0);
//	printf("notch_freq = %0.2f\n",notch_freq);

	
//	printf("index_max+1=%d\n", index_max+1);

	notch_freq=(index_max+1)*22050/1000;
	return notch_freq;

}



