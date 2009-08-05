// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file ScaleSpace.cpp
 * \brief Implements a Gaussian Scale Space for floating point images.
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */


#include "iCub/ScaleSpace.h"

#include <string.h>
#include <stdlib.h>
#include <cmath>
//#include <malloc.h>

ScaleSpace::ScaleSpace()
{
    _width = _height = _levels = 0;
    _allocated = false;
    _scales = NULL;
    _scalespace = NULL;
    _filters = NULL;
}

ScaleSpace::~ScaleSpace()
{
    FreeResources();
}

bool ScaleSpace::AllocateResources(int lines, int cols, int levels, double *scales)
{
    int i;
    if(lines < 10)  //image too small
        return false;
    if(cols < 10)   //image too small
        return false;
    if(levels < 1)  //
        return false;

    if(_allocated)
        FreeResources();
    _width = cols;
    _height = lines;
    _levels = levels;

    _scales = (double*)malloc(_levels*sizeof(double));
    if(scales == 0) return false;
    _scalespace = (float**)malloc(_levels*sizeof(float*));
    if(_scalespace == 0) return false;
    _filters = new FastGauss[levels];
    if(_filters == 0) return false;
    for(i=0; i < levels; i++)
    {
        _scales[i] = scales[i];
        _filters[i].AllocateResources(lines, cols, _scales[i]);
        _scalespace[i] = (float*)malloc(_width*_height*sizeof(float));
        if(_scalespace[i] == 0) return false;
    }
    _allocated = true;
    return true;
}

bool ScaleSpace::FreeResources()
{
    int i;
    if(!_allocated)
        return true;
    delete [] _filters;
    free(_scales);
    for(i=0; i < _levels; i++)
        free(_scalespace[i]);
    free(_scalespace);
    _allocated = false;
    return true;
}

///Build one level of the scale space
bool ScaleSpace::BuildLevel(int level, float *in)
{
    if(!_allocated)
        return false;
    if( level < 0 || level >= _levels )
        return false;
    return _filters[level].GaussFilt(in, _scalespace[level]);
}

///Builds all levels.
bool ScaleSpace::BuildAll(float *in) 
{
    int i;
    bool retval = true;
    if(!_allocated)
        return false;
    for(i = 0; i < _levels; i++)
        retval = (retval && _filters[i].GaussFilt(in, _scalespace[i]));
    return retval;
}
///Returns the pointer to the image at a certain level 
float* ScaleSpace::GetLevel(int level)
{
    if(!_allocated)
        return 0;
    if( level < 0 || level >= _levels )
        return 0;
    return _scalespace[level];
}
