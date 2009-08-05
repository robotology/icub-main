#include "mydraw.h"

void iCub::contrib::primateVision::MyDrawLine(Ipp8u*im,int psb_,IppiSize ss,int px,int py,Ipp32f col){

  for (int x=0;x<ss.width;x++){
    im[x+ py*psb_] = (int)(col*255.0);
  }

  for (int y=0;y<ss.height;y++){
    im[px+ y*psb_] = (int)(col*255.0);
  }

}
