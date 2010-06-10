
#ifndef __FACEDET_H__
#define __FACEDET_H__

#include <cv.h>

class faceDetector
{
protected:
    CvHaarClassifierCascade *pCascade;
    CvMemStorage            *pStorage;    

public:
    faceDetector();
    bool init(const char *haarCascadePath);    
    CvSeq *detect(IplImage *pImg);
    ~faceDetector();
};

#endif

