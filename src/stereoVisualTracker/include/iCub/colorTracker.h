#ifndef __COLORTRACKER_H__
#define __COLORTRACKER_H__

#include "public.h"
#include "colordetect.h"

#ifndef MAX_OBJECTS
#define MAX_OBJECTS 3
#endif

class ColorTracker;

typedef struct{
  ColorTracker *finder;
  FrameGrabber *grabber;
} mouseParam_t;


class ColorTracker :public ColorDetect{

 public:
  CvRect selection;
  int select_object;
  FrameGrabber *grabber;
  CvPoint origin;

  static int nb_member;
  static int displayed_color;
  static int nb_cams;
  static ColorTracker *members[2*MAX_OBJECTS];
  
  ColorTracker();
  void SetGrabber(FrameGrabber *grab){grabber=grab;}
  int IncrementSigma(f32 inc=0.5);
  int DecrementSigma(f32 inc=0.5);
  int IncrementThreshold(f32 inc=10);
  int DecrementThreshold(f32 inc=10);
  static mouseParam_t GetSelectionParams(int index=0);


};



#endif
