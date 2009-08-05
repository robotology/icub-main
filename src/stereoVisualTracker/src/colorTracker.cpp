#include "colorTracker.h"


int ColorTracker::nb_member=0;
int ColorTracker::displayed_color=0;
int ColorTracker::nb_cams=2;
ColorTracker *ColorTracker::members[2*MAX_OBJECTS];
ColorTracker::ColorTracker():ColorDetect()
{
  select_object=0;
  ColorTracker::members[nb_member++]=this;
}


int  ColorTracker::IncrementSigma(f32 inc){
  if(inc>0){
    sigma += inc;
    return 1;
  }
  return 0;
}

int  ColorTracker::DecrementSigma(f32 inc){  
  if(sigma>inc){
    sigma -= inc;
    return 1;
  }
  else{
    return 0;
  }
}


mouseParam_t ColorTracker::GetSelectionParams(int index){
   mouseParam_t ret;
   if(nb_member){
     //   ret.grabber =member[0].grabber
     ret.finder =  ColorTracker::members[nb_cams*displayed_color+index];
   }
   return ret;
}

