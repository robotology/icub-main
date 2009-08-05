#ifndef FFT_H
#define FFT_H

#define BUFFER_SIZE 1000      // BUFFER SIZE
#define F_SAMPLE 1000         // SAMPLING FREQUENCY
#define PI 3.1416  

# include <fftw3.h>
# include <math.h>

class fft_samples
{
public:
	double samples_fft [BUFFER_SIZE];
	int    samples_fft_counter;
	bool   buffer_full;
	fft_samples();
	bool add_sample(double sample);
};

class fft_cyclic_sample_buffer
{
public:
	double samples [BUFFER_SIZE];
	int	index_i;
	int index_f;
    int added_samples;
	fft_cyclic_sample_buffer();
	bool add_sample(double sample);
};

class fft_performer
{
  fftw_complex *in;
  fftw_complex *out;
  fftw_plan plan_forward;

public:
	fft_performer();
	~fft_performer();
	void do_fft(fft_samples& input);
	void do_fft(fft_cyclic_sample_buffer& input);
};

#endif