// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef CAMERA_INC
#define CAMERA_INC

#include <yarp/os/Searchable.h>

#ifndef pi
    #define pi 3.1415926
#endif

/**
*   Defines the geometry of a pinhole camera. 
*   Assumes radial distortion has been compensated. Check module CamCalib. 
*   Reference frames and camera paramenters are as defined in the OpenCV library:
*   
*   The camera reference frame is oriented as follows:
*   * X axis parallel to the image plane pointing right    
*   * Y axis parallel to the image plane pointing down
*   * Z axis orthogonal to the image plane pointing forward.
*
*   Let a point in 3D be written in this reference frame, with coordinates
*   (X,Y,Z), in meters.
*   The pinhole camera model then defines the metric coordinates as:
*   * X' = X/Z    
*   * Y' = Y/Z
*   These are the horizontal and vertical coordinates of a point in a 
*   virtual image plane 1m away from the camera.
*   
*
*   The camera forms the image by sampling the light information according to 
*   the geometry of the sensor:
*   * u = fx*X' + cx
*   * v = fy*Y' + cy
*
*   u is the horizontal image coordinate, starting at the top-left corner and
*   increasing to the right:
*
*   v is the vertical image coordinate, starting at the top-left corner and increasing down.
*
*   fx, fy are the horizontal and vertical focal distances in pixel units
*   cx, cy are the image plane center coordinates in pixel units.
*   
*   
*   Additionally, for making, image coordinates centered and independent of image size
*   we define the normalized image coordinates:
*       x = 2*u/w - 1
*       y = 2*v/h - 1
*   Thus, the normalized image coordinates (both x and y) are in the range [-1,1], being 0 the center of the image, -1 the left/top boundaries, and 1 the right/bottom boundaries.
*   The x coordinate starts at the image center and grows to the right. The y coordinate starts at the image center and grows to the bottom.
*   To go back to image pixels, the following expressions are used:
*       u = w*(x+1)/2
*       v = h*(y+1)/2 
*/

class Camera {
private:
    int    _calib_sx;     /** the camera width in pixels (image width at calibration)*/
    int    _calib_sy;     /** the camera height in pixels (image height at calibration)*/
    double _calib_fx;     /** the horizontal focal distance in horizontal pixel units  */       
    double _calib_fy;     /** the vertical focal distance in vertical pixel units */     
    double _calib_cx;     /** the horizontal center of coordinates in horizontal pixel units */
    double _calib_cy;     /** the vertical center of coordinates in vertical pixel units */
    
    /** the following parameters are equivalent to the previous, but are 
    *   scaled to reflect image sizes different from calibration. */
    int    _curr_sx;     
    int    _curr_sy;     
    double _curr_fx;     
    double _curr_fy;     
    double _curr_cx;     
    double _curr_cy;     
    
public:
    Camera() {
        /** some defaults corresponding to a view field of 90 deg. */;
        _calib_sx = _curr_sx = 640;
        _calib_sy = _curr_sy = 480;
        _calib_fx = _curr_fx = 320;
        _calib_fy = _curr_fy = 240;
        _calib_cx = _curr_cx = 320;
        _calib_cy = _curr_cy = 240;
    }

    /***/
    bool open(yarp::os::Searchable& config) {
        // Defaults will correspond to a view field of 90 deg.
        _calib_sy = _curr_sy = config.check("h", 480, "Input plane height").asInt();
        if(_calib_sy <= 0) return false;
        _calib_sx = _curr_sx = config.check("w", 640, "Input plane width").asInt();
        if(_calib_sx <= 0) return false;
        _calib_fx = _curr_fx = config.check("fx", 320, "Focal distance (on horizontal pixel size units)").asDouble();
        if(_calib_fx <= 0) return false;
        _calib_fy = _curr_fy = config.check("fy", 240, "Focal distance (on vertical pixel size units)").asDouble();
        if(_calib_fy <= 0) return false;
        _calib_cx = _curr_cx = config.check("cx", 320, "Image center (on horizontal pixel size units)").asDouble();
        if(_calib_cx <= 0) return false;
        _calib_cy = _curr_cy = config.check("cy", 240, "Image center (on vertical pixel size units)").asDouble();
        if(_calib_cy <= 0) return false;
        return true;
    }


    /**
     * Make note of current image size (as opposed to size at calibration)
     */
    bool setImageSize(int w, int h) {
        if( w <= 0 || h <= 0)
            return false;
        _curr_sx = w;
        _curr_sy = h;
        double scale_x = (double)_curr_sx/_calib_sx;
        double scale_y = (double)_curr_sy/_calib_sy;
        _curr_fx = _calib_fx*scale_x;
        _curr_cx = _calib_cx*scale_x;
        _curr_fy = _calib_fy*scale_y;
        _curr_cy = _calib_cy*scale_y;
        return true;
    }

    /**
     * Take an input (u,v) in image coordinates, and produce
     * an output in the metrix coordinates (X,Y).
     */
    void pixel2metric(double u, double v, double& X, double& Y) 
    {
        X = (u - _curr_cx)/_curr_fx;  
        Y = (v - _curr_cy)/_curr_fy;
    }

    /* The inverse of the above */
    void metric2pixel(double X, double Y, double& u, double& v) 
    {
        u = X*_curr_fx + _curr_cx;  
        v = X*_curr_fy + _curr_cy;
    }

    /**
     * Take an input (X,Y) in metric coordinates, and produce
     * an output in azimuth-elevation angles.
     * This represents a rotation of -azimuth around the camera 
     * frame Y axis, followed by a rotation of -elevation around 
     * the current X axis.
     * Thus the azimuthal angle grows to the left of the visual field and
     * the elevation angle grows to the above of the visual field.
     * Angles are represented in degrees 
     */

    void metric2angle(double X, double Y, double& a, double& e) 
    {
            double _rho = sqrt(X*X+Y*Y+1);
            a = -atan(X)*180/pi; //azimuth  
            e = -asin(Y/_rho)*180/pi; //elevation
    }

    /** The inverse of the above */
    void angle2metric(double a, double e, double& X, double& Y) 
    {
            X = -tan(a*pi/180);
            Y = -tan(e*pi/180)*sqrt(X*X+1);
    }

    /**
     * Take an input (u,v) in pixel coordinates, and produce
     * an output in azimuth-elevation angles.
     * This represents a rotation of azimuth around the camera 
     * frame Y axis, followed by a rotation of elevation around 
     * the current X axis.
     * Angles are represented in degrees
     */
    void pixel2angle(double u, double v, double& a, double &e) 
    {
        double X, Y;
        pixel2metric(u,v,X,Y);
        metric2angle(X,Y,a,e);
    }

    /** The inverse of the above */
    void angle2pixel(double a, double e, double& u, double &v) 
    {
        double X, Y;
        angle2metric(a,e,X,Y);
        metric2pixel(X,Y,u,v);
    }

    /**
     * Take an input (u,v) in pixel coordinates, and produce
     * an output in normalized pixel coordinates.
     * The coordinates become independent of image size
     */
    void pixel2norm(double u, double v, double& x, double& y )
    {
        x = 2*u/_curr_sx - 1;
        y = 2*v/_curr_sy - 1;
    }

    /* The inverse of the above */
    void norm2pixel(double x, double y, double& u, double& v )
    {
        u = _curr_sx*(x+1)/2;
        v = _curr_sy*(y+1)/2;
    }

    /**
     * Take an input (x,y) normalized pixel coordinates, and produce
     * an output in azimuth-elevation angles.
     * Angles are represented in degrees
     */

    void norm2angle(double x, double y, double& a, double& e )
    {
        double u,v;
        norm2pixel(x,y,u,v);
        pixel2angle(u,v,a,e);
    }

    /**
     * Take an input (x,y) normalized pixel coordinates, and produce
     * an output in metrix X,Y coordinates.
     */

    void norm2metric(double x, double y, double& X, double& Y )
    {
        double u,v;
        norm2pixel(x,y,u,v);
        pixel2metric(u,v,X,Y);
    }


    inline int height() {return _curr_sy;};
    inline int width() {return _curr_sx;};
    inline int cal_height() {return _calib_sy;};
    inline int cal_width() {return _calib_sx;};

    inline double fx() {return _curr_fx;};
    inline double fy() {return _curr_fy;};
    inline double cx() {return _curr_cx;};
    inline double cy() {return _curr_cy;};
    inline double cal_fx() {return _calib_fx;};
    inline double cal_fy() {return _calib_fy;};
    inline double cal_cx() {return _calib_cx;};
    inline double cal_cy() {return _calib_cy;};
};


#endif

