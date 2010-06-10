// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file ScaleSpace.h
 * \brief Implements a Gaussian Scale Space for floating point images.
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */


#ifndef _SCALESPACE_H_
#define _SCALESPACE_H_

#include <iCub/vis/FastGauss.h>

class ScaleSpace
{
private:
    int _width;                     ///The width the images
    int _height;                    ///The height of the images
    int _levels;                    ///The number of levels
    double *_scales;                 ///The scale value (gaussian std dev) for each level
    bool _allocated;	            ///Boolean variable indicating if the object has been allocated
    float **_scalespace;            ///The scale space (array of floating point images)   
    FastGauss *_filters;             ///The gaussian filters (one for each scale)
public:
    ///Returns the number of lines / height of the images
    int GetHeigth() {return _height;};  
    int GetLines() {return _height;};  
    ///Returns the number of columns / width of the images (level0) 
    int GetWidth() {return _width;}; 
	int GetCols() {return _width;};           
    ///Returns the number of levels (N)
    int GetLevels() {return _levels;};   
    ///Returns true if memory is allocated
    bool IsAllocated() {return _allocated;};   

    ScaleSpace(void);      
    virtual ~ScaleSpace(void);

    ///Allocates memory for the pyramid
    bool AllocateResources(int lines, int cols, int levels, double *scales );
    ///Releases memory
	bool FreeResources();    
       
    /// Builds a certain level of the scale space. All levels are independent
    bool BuildLevel(int level, float *in);
    ///Builds all levels.
	bool BuildAll(float *in); 
    ///Returns the pointer to the image at a certain level 
    float* GetLevel(int level);
};






#endif /*_SCALESPACE_H_*/
