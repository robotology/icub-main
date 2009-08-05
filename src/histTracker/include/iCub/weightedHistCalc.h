#include <cv.h>
#include <cxmisc.h>
//#include "cxmisc.h"
#if CV_MAJOR_VERSION < 1 && CV_MINOR_VERSION != 9 && CV_SUBMINOR_VERSION != 9 
//#if CV_MAJOR_VERSION = 1 && CV_MINOR_VERSION != 0 && CV_SUBMINOR_VERSION != 0
//#define CV_VERSION          "1.0.0"
typedef union Cv32suf
{
    int i;
    unsigned u;
    float f;
}
Cv32suf;
#endif

#ifndef CV_MAJOR_VERSION //|| CV_MINOR_VERSION || CV_SUBMINOR_VERSION
typedef union Cv32suf
{
    int i;
    unsigned u;
    float f;
}
Cv32suf;
#endif

CVAPI(void)  cvCalcArrHistKernelWeighted( CvArr** arr, CvHistogram* hist, const CvArr *kernelMask,CvArr ** binTable,
                            int accumulate CV_DEFAULT(0),
                            const CvArr* mask CV_DEFAULT(NULL) );

CV_INLINE  void  cvCalcHistKernelWeighted( IplImage** image, CvHistogram* hist, const CvArr *kernelMask, IplImage ** binTable,
                             int accumulate CV_DEFAULT(0),
                             const CvArr* mask CV_DEFAULT(NULL) )
{
    cvCalcArrHistKernelWeighted( (CvArr**)image, hist,kernelMask, (CvArr**)binTable, accumulate, mask );
}
