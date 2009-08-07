// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


//#define TO_HARDWARE

#include <stdio.h>
#include <math.h>
#include "structs.h"

#include "listen.h"

#include "yarpy.h"

#define LOOP_INPUT

#define EXTERNAL_CONTROL



////////////////////////////////////////////////////////////////
// AUDIO STUFF
////////////////////////////////////////////////////////////////


#include <stdio.h>	// for printf(), perror() 
#include <fcntl.h>	// for open() 
#include <stdlib.h>	// for exit() 
#include <math.h>	// for sin()

#ifdef TO_HARDWARE
#include <unistd.h>	// for write(), close() 
#include <sys/ioctl.h>	// for ioctl()
#include <sys/soundcard.h>
#endif

#define SAMPLE_RATE	22000
#define SAMPLES 512
//#define SAMPLE_RATE	48000
#define NUM_SAMPLES	SAMPLE_RATE
#define TWO_PI		(3.14159 * 2.0)

static int dev = -1;
static int mute = 0;

void setMute(int m) {
  mute = m;
}

void audio_init() {
#ifdef TO_HARDWARE
  // open the audio device and set its playback parameters
  int	bits = 8, chns = 1, rate = SAMPLE_RATE;
  dev = open( "/dev/dsp", O_WRONLY );
  if ( dev < 0 ) { perror( "/dev/dsp" ); exit(1); }
  if (( ioctl( dev, SNDCTL_DSP_SETFMT, &bits ) < 0 )
      ||( ioctl( dev, SNDCTL_DSP_CHANNELS, &chns ) < 0 )
      ||( ioctl( dev, SNDCTL_DSP_SPEED, &rate ) < 0 ))
    { perror( "audio format not supported\n" ); exit(1); }
#endif
}


unsigned char	pulse_code[ NUM_SAMPLES*10 ];
int pulse_at = 0;

void audio_emit(int n) {
  static int first = 1;
  if (first) {
#ifdef TO_HARDWARE
    audio_init();
#endif
    first = 0;
  }

  if (pulse_at<SAMPLES) {
#ifdef TO_HARDWARE
    pulse_code[pulse_at] = (unsigned char)n;
    pulse_at++;
#else
    ((int16_t*)pulse_code)[pulse_at] = n;
    pulse_at++;
#endif
  } else {
    // now playback the sound
    unsigned char	*pcm = pulse_code;
    int morebytes = pulse_at;
    plisten(pcm,morebytes);
#ifdef TO_HARDWARE
    while ( morebytes > 0 )
      {
	int	nbytes = write( dev, pcm, morebytes );
	morebytes -= nbytes;
	pcm += nbytes;
      }
#endif
    pulse_at = 0;
  }
    //close( dev );		// close the audio playback device
}










////////////////////////////////////////////////////////////////
// INTERFACE STUFF
////////////////////////////////////////////////////////////////



static TRMData fake_data;
static TRMParameters fake_parameters;

int isRemote() {
  return remote;
}

INPUT *remoteInput(INPUT *input) {
  //return input;

  static int first = 1;
  static INPUT inputs[2];
  int i;
  if (!isRemote()) {
    return input;
  }
  if (first) {
    yarpy();
    printf("Remote input procedure initializing\n");
    for (i=0; i<2; i++) {
      inputs[i].previous = inputs[i].next = &inputs[(i+1)%2];
    }
    if (input!=NULL) {
      fake_parameters = input->parameters;
    }
    fake_data.inputHead = input;
    fake_data.inputTail = NULL; // invariant is broken for now since not needed
    first = 0;
#ifdef LOOP_INPUT
    // leave input unchanged
#else
    input = &inputs[0];
#endif
  }
  // must set up input

#ifdef LOOP_INPUT
  if (input==NULL) {
    input = fake_data.inputHead;
  }
  //input->parameters.glotPitch *= 0.9;
  //input->parameters.glotVol *= 0.9;
  //input->parameters.radius[6] *= 0.7;
  TRMParameters *param = &(input->parameters);

  /*
  //param->glotPitch = 5;
  //param->glotVol = 100;
  //param->aspVol *= 1.1;
  //param->fricVol = 0;
  param->aspVol = 0;
  param->fricVol = 0;
  param->fricPos = 0;
  param->fricCF = 0;
  param->fricBW = 0;
  int ii;
  static double r = 0;
  r+=0.01;
  param->glotPitch = sin(r*1.7)+1;
  param->glotVol = 50*(sin(r)+1);
  for (ii=0; ii<TOTAL_REGIONS; ii++) {
    if (ii>=7) {
      param->radius[ii] = sin(r)+1;
    } else {
      param->radius[ii] = 1;
    }
  }
  param->velum = 0;
  */

#ifdef EXTERNAL_CONTROL
  TRMParameters *param_fake = &(fake_data.inputHead->parameters);
  setParams(param_fake);
  static INPUT *save_input = NULL;
  if (getParam(7,0)<50) {
    if (save_input==NULL) {
      save_input = input;
    }
    param = param_fake;
    input = fake_data.inputHead;
    //param = &(input->parameters);
    param->glotVol *= 100;
    // there's an annoying buzz when everything is turned down
    if (param->glotVol<=1 && param->aspVol<=0.01 && param->fricVol<=0.01) {
      setMute(1);
    } else {
      setMute(0);
    }
  } else {
    if (save_input!=NULL) {
      printf("Switching to alternative source\n");
      input = save_input;
      save_input = NULL;
    }
  }
#endif
  
#else
  input->parameters = fake_parameters;
#endif

  return input;
}

void remoteOutput(double signal) {
  static long int at = 0;
  at++;
  if (mute) {
    signal = 0;
  }

#ifdef TO_HARDWARE
  long int v = (long int)(128+signal*100000.0);
  if (v>255) v = 255;
  if (v<0) v = 0;
  audio_emit((int)v);
#else
  long int v = (long int)(signal*1000000.0);
  if (v>30000) v = 30000;
  if (v<-30000) v = -30000;
  //printf("missing emit\n");
  audio_emit((int)v);
#endif

  static long int top = -1, bot = 256;
  static double sig_top = -1e32, sig_bot = 1e32;
  if (v>top) top = v;
  if (v<bot) bot = v;
  if (signal>sig_top) sig_top = signal;
  if (signal<sig_bot) sig_bot = signal;
  if (at%100000==0) {
    //fprintf(stderr,"%g\n", signal);
    //fprintf(stderr,"At %ld    (input %g to %g) (output %ld to %ld)\n", at, sig_bot, sig_top, bot, top);
    top = -1;
    bot = 256;
    sig_top = -1e32;
    sig_bot = 1e32;
  }
}
