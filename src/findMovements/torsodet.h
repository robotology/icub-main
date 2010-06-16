
#ifndef __TORSODET_H__
#define __TORSODET_H__

#include <cv.h>

class TorsoDetector
{
protected:
    CvHaarClassifierCascade *pCascade;
    CvMemStorage            *pStorage;    

public:
    TorsoDetector();
    bool init(const char *haarCascadePath);    
    CvSeq *detect(IplImage *pImg);
    ~TorsoDetector();
};

#endif

