#include <iCub/blobFinderThread.h>

#include <iostream>
using namespace std;

blobFinderThread::blobFinderThread(){
    reinit_flag=false;
}


void blobFinderThread::reinitialise(int weight, int height){
    img=new ImageOf<PixelRgb>;
    img->resize(weight,height);
}

