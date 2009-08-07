/* xspectrum - displays the power spectrum of a signal as a function of time
 * Copyright (C) 2000 Vincent Arkesteijn <v.j.arkesteijn@student.utwente.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as 
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */

#include <math.h>
#include <assert.h>


#define SKIP_THIS


#ifndef SKIP_THIS

#include <fftw3.h>
#include "fft.h"

typedef double fftw_real;

class FFTHelper {
public:
  fftw_plan fft_plan;
  int N;
  fftw_real *fft_inbuf, *fft_outbuf;

  ~FFTHelper() {
    if (fft_inbuf!=NULL) {
      fft_close();
    }
  }

  void fft_init (int n) {
    fft_inbuf  = (fftw_real *) fftw_malloc (n*sizeof(fftw_real));
    fft_outbuf = (fftw_real *) fftw_malloc (n*sizeof(fftw_real));
    fft_plan = fftw_plan_r2r_1d(n, fft_inbuf, fft_outbuf,
				FFTW_R2HC, FFTW_ESTIMATE);
    
    assert(fft_inbuf!=NULL);
    assert(fft_outbuf!=NULL);
    N = n;
  }
  
  void fft_do (float *audio_buf, float *psd_buf, float *phase_buf) {
    int i;
    
    for (i=0; i<N; i++)
      fft_inbuf[i] = audio_buf[i];
    
    fftw_execute(fft_plan);
    //rfftw_one (fft_plan, fft_inbuf, fft_outbuf);
    
    if (psd_buf) {
      /* From the fftw tutorial: */
      psd_buf[0] = fft_outbuf[0]*fft_outbuf[0];
      for (i=1; i<(N+1)/2; i++)
	psd_buf[i] = fft_outbuf[i]*fft_outbuf[i] + fft_outbuf[N-i]*fft_outbuf[N-i];
      if (N%2==0)
	psd_buf[N/2] = fft_outbuf[N/2]*fft_outbuf[N/2];
    }
    
    if (phase_buf) {
      phase_buf[0] = 0;
      for (i=1; i<(N+1)/2; i++)
	phase_buf[i] = atan2 (fft_outbuf[i], fft_outbuf[N-i]);
      if (N%2==0)
	phase_buf[N/2] = 0;
    }
  }
  
  void fft_close () {
    fftw_destroy_plan(fft_plan);
    fftw_free (fft_inbuf);
    fftw_free (fft_outbuf);
  }
};



#define SYS (*((FFTHelper*)system_resources))

FFT::FFT(int n) {
  system_resources = NULL;
  system_resources = new FFTHelper;
  assert(system_resources!=NULL);
  SYS.fft_init(n);
}

FFT::~FFT() {
  if (system_resources!=NULL) {
    delete &SYS;
    system_resources = NULL;
  }
}


void FFT::apply(float *input, float *mag, float *phase) {
  SYS.fft_do(input,mag,phase);
}

#endif
