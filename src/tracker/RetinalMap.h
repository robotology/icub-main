// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick, Alex Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef RETINALMAP_INC
#define RETINALMAP_INC

#include <yarp/os/Searchable.h>


/**
 * Map from image coordinates to a normalized coordinate.
 */
class RetinalMap {
private:
    int _input_lines;
    int _input_cols;
    
    double _fx;            
    double _fy;           
    double _cx;
    double _cy;
    bool _angle;
    bool valid;
    int _w;
    int _h;

public:
    RetinalMap() {
        valid = false;
        _angle = false;
    }

    bool open(yarp::os::Searchable& config) {
        // source:  Alex Bernardino src/camShift/camshift.cpp
        // Defaults will correspond to a view field of 90 deg.
        _h = _input_lines = config.check("h", 480, "Input image lines").asInt();
        _w = _input_cols = config.check("w", 640, "Input image columns").asInt();
        _fx = config.check("fx", 320, "Focal distance (on horizontal pixel size units)").asDouble();
        _fy = config.check("fy", 240, "Focal distance (on vertical pixel size units)").asDouble();
        _cx = config.check("cx", 320, "Image center (on horizontal pixel size units)").asDouble();
        _cy = config.check("cy", 240, "Image center (on vertical pixel size units)").asDouble();
        _angle = config.check("angle", "Output positions in azimuth-elevation [degrees]");
        valid = true;
        return valid;
    }


    /**
     * Make note of current image size (as opposed to size at calibration)
     */
    void setImage(int w, int h) {
        _w = w;
        _h = h;
    }

    /**
     * Take an input (ix,iy) in image coordinates, and produce
     * an output in the remapped coordinates.
     */
    void remap(double ix, double iy, double& ox, double& oy) {
        // source:  Alex Bernardino src/camShift/camshift.cpp
        _angle = true;
        printf("Pixel %g %g image size %d %d, angle flag %d\n", 
               ix, iy, _w, _h, _angle);
        if (_w!=_input_cols) {
            ix /= _w;
            ix *= _input_cols;
        }
        if (_h!=_input_lines) {
            iy /= _h;
            iy *= _input_lines;
        }
        if(!_angle) {
            // output pixel coordinates
            ox = (ix - _cx)/_fx;  
            oy = (iy - _cy)/_fy;
        } else {
            //convert to spherical coords
            double _x = (ix - _cx)/_fx;
            double _y = (iy - _cy)/_fy;
            double _rho = sqrt(_x*_x+_y*_y+1);
            ox = -atan(_x)*180/3.141529; //azimuth  
            oy = -asin(_y/_rho)*180/3.141529; //elevation
        }
    }

    bool isValid() {
        return valid;
    }
};


#endif

