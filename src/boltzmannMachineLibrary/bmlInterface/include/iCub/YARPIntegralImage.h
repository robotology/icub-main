
#ifndef __YARPINTEGRALIMAGE__
#define __YARPINTEGRALIMAGE__

//#include <yarp/YARPImage.h>
//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

#include <math.h>

class YARPIntegralImage
{
public:
    YARPIntegralImage();
    YARPIntegralImage(int nR, int nC, int sfovea = 0);
    ~YARPIntegralImage();

    void resize(int nR, int nC, int sfovea = 0);

    inline void get(ImageOf<PixelMono> &out);
    inline float get(int c, int r)
    { return _integralImg(c,r)/_max; }

    inline double getMean(int maxX, int minX, int maxY, int minY);
    inline double getMeanLp(int maxT, int minT, int maxR, int minR);
    
    inline double getSaliency(int maxX, int minX, int maxY, int minY);
    inline double getSaliencyLp(int maxT, int minT, int maxR, int minR);
    
    int computeCartesian(ImageOf<PixelMono> &input);
    int computeCartesian(ImageOf<PixelMonoSigned> &input);
    int computeLp(ImageOf<PixelMono> &input);
    int computeLp(ImageOf<PixelMonoSigned> &input);

private:
    void _resize(int nC, int nR, int sfovea);
    ImageOf<PixelFloat> _integralImg;
    ImageOf<PixelFloat> _rowSum;
    int _nRows;
    int _nCols;
    int _nfovea;
    float _max;
};

inline void YARPIntegralImage::get(ImageOf<PixelMono> &out)
{
    int r,c;
    float *src;
    unsigned char *dst;
    for(r = 0; r < _nRows; r++)
    {
        //src = (float *) _integralImg.GetArray()[r];
        src = (float *) _integralImg.getRawImage()[r];
        //dst = (unsigned char *) out.GetArray()[r];
        dst = (unsigned char *) out.getRawImage()[r];
        for(c = 0; c < _nCols; c++)
        {
            *dst = (PixelMono)(((*src)*255)/_max + 0.5);
            src++;
            dst++;
        }
    }
}

inline double YARPIntegralImage::getMeanLp(int maxT, int minT, int maxR, int minR)
{
    double tmp1, tmp2, tmp3, tmp4;
    
    if (minR < 0)
        minR = 0;
                
    if (maxR > _nRows-1)
        maxR = _nRows-1;
        
    if ( (minT>=0) && (maxT<_nCols) )
    {
        tmp1 = get(minT, minR);
        tmp2 = -get(maxT, minR);
        tmp3 = -get(minT, maxR);
        tmp4 = get(maxT, maxR);
    }
    else if ( (minT<0) && (maxT<_nCols) )
    {
        tmp1 = get(0, minR);
        tmp2 = -get(maxT, minR);
        tmp3 = -get(0, maxR);
        tmp4 = get(maxT, maxR);
        
        tmp1 += get(_nCols-1+minT, minR);
        tmp2 += -get(_nCols-1, minR);
        tmp3 += -get(_nCols-1+minT, maxR);
        tmp4 += get(_nCols-1, maxR);
    }
    else if ( (minT>0) && (maxT>=_nCols) )
    {
        tmp1 = get(minT, minR);
        tmp2 = -get(_nCols-1, minR);
        tmp3 = -get(minT, maxR);
        tmp4 = get(_nCols-1, maxR);

        tmp1 += get(0, minR);
        tmp2 += -get(maxT-_nCols, minR);
        tmp3 += -get(0, maxR);
        tmp4 += get(maxT-_nCols, maxR);
    }
    else
        return 0;		// case not supported

    return (tmp4 + tmp3 + tmp1 + tmp2)/((maxR-minR+1)*(maxT-minT+1));
}

inline double YARPIntegralImage::getSaliencyLp(int maxT, int minT, int maxR, int minR)
{
    double tmp1, tmp2, tmp3, tmp4; 
    
    if (minR < 0)
        minR = 0;
                
    if (maxR > _nRows-1)
        maxR = _nRows-1;
        
    if ( (minT>=0) && (maxT<_nCols) )
    {
        tmp1 = get(minT, minR);
        tmp2 = -get(maxT, minR);
        tmp3 = -get(minT, maxR);
        tmp4 = get(maxT, maxR);
    }
    else if ( (minT<0) && (maxT<_nCols) )
    {
        tmp1 = get(0, minR);
        tmp2 = -get(maxT, minR);
        tmp3 = -get(0, maxR);
        tmp4 = get(maxT, maxR);
        
        tmp1 += get(_nCols-1+minT, minR);
        tmp2 += -get(_nCols-1, minR);
        tmp3 += -get(_nCols-1+minT, maxR);
        tmp4 += get(_nCols-1, maxR);
    }
    else if ( (minT>0) && (maxT>=_nCols) )
    {
        tmp1 = get(minT, minR);
        tmp2 = -get(_nCols-1, minR);
        tmp3 = -get(minT, maxR);
        tmp4 = get(_nCols-1, maxR);

        tmp1 += get(0, minR);
        tmp2 += -get(maxT-_nCols, minR);
        tmp3 += -get(0, maxR);
        tmp4 += get(maxT-_nCols, maxR);
    }
    else
        return 0;		// case not supported

    return (tmp4 + tmp3 + tmp1 + tmp2);
}

inline double YARPIntegralImage::getMean(int maxX, int minX, int maxY, int minY)
{
    double tmp1; 
    double tmp2; 
    double tmp3; 
    double tmp4; 

    if (minX < 0)
        minX = 0;
    if (minY < 0)
        minY = 0;
    if (maxX > _nCols-1)
        maxX = _nCols-1;
    if (maxY > _nRows-1)
        maxY = _nRows-1;

    tmp1 = get(minX, minY);
    tmp2 = get(maxX, minY);
    tmp3 = get(minX, maxY);
    tmp4 = get(maxX, maxY);

    return (tmp4 + tmp1 - (tmp2+tmp3))/((maxX-minX+1)*(maxY-minY+1));
}

inline double YARPIntegralImage::getSaliency(int maxX, int minX, int maxY, int minY)
{
    double tmp1; 
    double tmp2; 
    double tmp3; 
    double tmp4; 

    if (minX < 0)
        minX = 0;
    if (minY < 0)
        minY = 0;
    if (maxX > _nCols-1)
        maxX = _nCols-1;
    if (maxY > _nRows-1)
        maxY = _nRows-1;

    tmp1 = get(minX, minY);
    tmp2 = get(maxX, minY);
    tmp3 = get(minX, maxY);
    tmp4 = get(maxX, maxY);

    return (tmp4 + tmp1 - (tmp2+tmp3));
}

#endif



//----- end-of-file --- ( next line intentionally left blank ) ------------------
