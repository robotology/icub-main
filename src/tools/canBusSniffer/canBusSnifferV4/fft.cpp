#include "fft.h"
#include "stdlib.h"

double norm_abs (double real, double img)
{
	return sqrt(real*real+img*img);
}

double calculate_mean (double *in,int len)
{
	double mean=0;
	int i=0;
	for (i=0; i<len; i++)
	{
		mean+=in[i];
	}
	mean/=len;
	return mean;
}

fft_samples::fft_samples()
{
	int i=0;
	for (i=0;i<BUFFER_SIZE;i++)	samples_fft[i]=0;
	samples_fft_counter=0;
	buffer_full=0;
}

bool fft_samples::add_sample(double sample)
{
	if (buffer_full) return buffer_full;

	if (samples_fft_counter>=BUFFER_SIZE)
	{
		buffer_full=1;
		samples_fft_counter=0;
	}
	else
	{
		samples_fft [samples_fft_counter]=sample;
		samples_fft_counter++;
	}
	return buffer_full;
}

fft_cyclic_sample_buffer::fft_cyclic_sample_buffer()
{
	index_i=0;
	index_f=BUFFER_SIZE;
	int i=0;
	added_samples=0;
	for (i=0; i<BUFFER_SIZE; i++) samples [i]=0;
}

bool fft_cyclic_sample_buffer::add_sample(double sample)
{
	samples [index_i]=sample;
	index_i++;
	index_f++;
	added_samples++;
	if (index_i>=BUFFER_SIZE) index_i=0;
	if (index_f>=BUFFER_SIZE) index_f=0;

	if (added_samples>1000) 
	{
		added_samples=0;
		return true;
	}
	else
		return false;
}

void HM_WINDOW(double *sample, int N_sample, double *HS)
{
	// Hamming window
	for(int i=0;i<N_sample;i++)
		HS[i]=(0.53836-0.46164*cos(2*PI*(i)/(N_sample)))*sample[i];
	
}

fft_performer::fft_performer()
{
	in=(fftw_complex*)fftw_malloc ( sizeof ( fftw_complex ) * BUFFER_SIZE );
	out=(fftw_complex*)fftw_malloc ( sizeof ( fftw_complex ) * BUFFER_SIZE );
 
}

fft_performer::~fft_performer()
{
	fftw_free ( in );
    fftw_free ( out );
	//fftw_destroy_plan ( plan_forward );
}

void fft_performer::do_fft(fft_cyclic_sample_buffer& input)
{
	fft_samples s;
	int i=0,j=0;
	for (i=0, j=input.index_i; i<BUFFER_SIZE; i++, j++)
	{
		if (j>=BUFFER_SIZE) j=0;
		s.samples_fft[i]=input.samples[j];
	}
	this->do_fft(s);
}

void fft_performer::do_fft(fft_samples& input)
{
  /*
  #define CONV_SIN 6.28/360*0.36

  for ( i = 0; i < n; i++ )
  {
    in[i][0] = 1*sin(5*i*CONV_SIN)+3*sin(22*i*CONV_SIN);
    in[i][1] = 0;
  }

  for ( i = 0; i < n; i++ )
  {
    printf ( "  %3d  %12f  %12f\n", i, in[i][0], in[i][1] );
  }
  */

  int i=0;
  //double mean=calculate_mean (input.samples_fft,BUFFER_SIZE);
  double mean=calculate_mean (input.samples_fft,BUFFER_SIZE);



  for ( i = 0; i < BUFFER_SIZE; i++ )
  {
	input.samples_fft[i]-=mean;
  }


  double HS[BUFFER_SIZE];
    for ( i = 0; i < BUFFER_SIZE; i++ )
	{
		HS[i]=0;
	}
  HM_WINDOW(input.samples_fft, BUFFER_SIZE, HS);

  for ( i = 0; i < BUFFER_SIZE; i++ )
  {
  //  in[i][0] = HS[i];
    in[i][0] = input.samples_fft[i];
    in[i][1] = 0;
  }

  plan_forward = fftw_plan_dft_1d ( BUFFER_SIZE, in, out, FFTW_FORWARD, FFTW_ESTIMATE );
  fftw_execute ( plan_forward );

  double fft[BUFFER_SIZE];
  for ( i = 0; i < BUFFER_SIZE; i++ )
  {
    fft[i] = 2*norm_abs(out[i][0],out[i][1])/BUFFER_SIZE ;
  }

  double freq[BUFFER_SIZE/2];

  for ( i = 0; i < BUFFER_SIZE/2; i++ )
  {
	  freq[i] = float(i)/(float(BUFFER_SIZE)/2);	  
	  freq[i] = F_SAMPLE/2*freq[i];
  }

  system("cls");
  int j=0; 
  int blocks=0;
  for ( i = 0; i < 110/*n/2*/; i++ )
  //for ( i = 400; i < 499/*n/2*/; i++ )
  {
		if (fft[i]>120)
			printf ( "*"); 
		else
			printf ( " "); 
	 	printf ( "  %12.2f %12.2f	",  freq[i], fft[i]);
		blocks=fft[i]/2;
		if (blocks>100) blocks = 100;
		for (j=0;j<blocks;j++) 
		{
			printf ( "%c",178);
		}
		for (;j<100;j++)
		{
			//printf ( ".");
		}
		printf ( "\n");
  }

  input.buffer_full=0;
  input.samples_fft_counter=0;
  return;
}

