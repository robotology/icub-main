// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          2007 Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#ifndef __HISTOGRAM_HPP__B
#define __HISTOGRAM_HPP__B

#include "image_wrapper.hpp"

#include <iostream>
using namespace std;

#include <cv.h>


class Histogram {
public:
    Histogram(int nBins, float range_min, float range_max, bool interpolation = false);
    ~Histogram();

    void fill(IplImage const *source, IplImage const *weight, bool reset = 1);
    float operator[](int i) const;
    float bin_center(int bin) const;
    int number_of_bins() const;
    float bin_size() const;
    float const *get_data();

    void show_contents() const;
    static void test_class();

private:
    float range[2];
    float *bin;
    int nBins;

    void (Histogram::* fill_fp)(IplImage const *, IplImage const *, bool);
    void fill_interpolation(IplImage const *source, IplImage const *weight, bool reset);
    void fill_no_interpolation(IplImage const *source, IplImage const *weight, bool reset);
};


class OrientationHistogram {
public:
    OrientationHistogram(int nBins, bool interpolation = false );
    ~OrientationHistogram();

    void fill(IplImage const *source, IplImage const *weight, bool reset = 1);
    float operator[](int i) const;
    float bin_center(int bin) const;
    int number_of_bins() const;
    float bin_size() const;
    float const *get_data();

    void show_contents() const;
    static void test_class();

private:
    float *bin;
    int nBins;

    void (OrientationHistogram::* fill_fp)(IplImage const *, IplImage const *, bool);
    void fill_interpolation(IplImage const *source, IplImage const *weight, bool reset);
    void fill_no_interpolation(IplImage const *source, IplImage const *weight, bool reset);
};


template<unsigned nBins>
class THistogram {
public:
    THistogram(float range_min, float range_max, bool interpolation = false);
    ~THistogram();

    void fill(IplImage const *source, IplImage const *weight, bool reset = 1);
    float operator[](int i) const;
    float bin_center(int bin) const;
    int number_of_bins() const;
    float bin_size() const;
    float const *get_data();

    void show_contents() const;
    static void test_class();

private:
    float range[2];
    float bin[nBins];

    void (THistogram<nBins>::* fill_fp)(IplImage const *, IplImage const *, bool);
    void fill_interpolation(IplImage const *source, IplImage const *weight, bool reset);
    void fill_no_interpolation(IplImage const *source, IplImage const *weight, bool reset);
};


template<unsigned nBins>
class OrientationTHistogram {
public:
    OrientationTHistogram( bool interpolation = false);
    ~OrientationTHistogram();

    void fill(IplImage const *source, IplImage const *weight, bool reset = 1);
    float operator[](int i) const;
    float bin_center(int bin) const;
    int number_of_bins() const;
    float bin_size() const;
    float const *get_data();

    void show_contents() const;
    static void test_class();

private:
    float bin[nBins];

    void (OrientationTHistogram<nBins>::* fill_fp)(IplImage const *, IplImage const *, bool);
    void fill_interpolation(IplImage const *source, IplImage const *weight, bool reset);
    void fill_no_interpolation(IplImage const *source, IplImage const *weight, bool reset);
};



/*************  class THistogram  ****************/

template<unsigned nBins>
THistogram<nBins>::THistogram(float range_min, float range_max, bool interpolation) {
    fill_fp = ( interpolation ? &THistogram<nBins>::fill_interpolation : &THistogram<nBins>::fill_no_interpolation );

    range[0] = range_min;
    range[1] = range_max;
}

template<unsigned nBins>
THistogram<nBins>::~THistogram() {}

template<unsigned nBins>
inline void THistogram<nBins>::fill(IplImage const *source, IplImage const *weight, bool reset) {
    (this->*fill_fp)(source, weight, reset);
}


template<unsigned nBins>
void THistogram<nBins>::fill_interpolation(IplImage const *source, IplImage const *weight, bool reset) {
    int i, j, b;
    CvRect roi_s = cvGetImageROI( source );
    CvRect roi_w = cvGetImageROI( weight );
    float b_size = bin_size();
    float c = bin_center(0);
    float dist,temp;

    if( reset )
        for(i = 0; i < nBins; i++)
            bin[i] = 0.0;

    for(i = 0; i < roi_s.height; i++)
        for(j = 0; j < roi_s.width; j++) {
            temp = getFloat(source, i+roi_s.y ,j+roi_s.x);
            b = static_cast<int>((temp - c)/b_size );
            dist = (temp - bin_center(b))/b_size;

            temp = getFloat(weight, i+roi_w.y, j+roi_w.x);
            if( dist < 0 )
                bin[b] += (1.0 + dist) * temp;
            else if( b == nBins-1 )
                bin[b] += (1.0 - dist) * temp;
            else {
                bin[b] += (1.0 - dist) * temp;
                bin[b+1] += dist * temp;
            }
        }
    /*
    {			cout << "(" << i+roi.y << "," << j+roi.x << ") = " << getFloat(source,i+roi.y,j+roi.x);
    			cout << "\tBin = " << floor( (getFloat(source,i+roi.y,j+roi.x)-range[0])/bin_size ) << endl;
    }*/
}

template<unsigned nBins>
void THistogram<nBins>::fill_no_interpolation(IplImage const *source, IplImage const *weight, bool reset) {
    int i, j;
    CvRect roi = cvGetImageROI( source );
    float b_size = bin_size();

    if( reset )
        for(i = 0; i < nBins; i++)
            bin[i] = 0.0;

    for(i = 0; i < roi.height; i++)
        for(j = 0; j < roi.width; j++)
            bin[static_cast<int>( (getFloat(source,i+roi.y,j+roi.x)-range[0])/b_size )] += getFloat(weight,i,j);
    /*
    {			cout << "(" << i+roi.y << "," << j+roi.x << ") = " << getFloat(source,i+roi.y,j+roi.x);
    			cout << "\tBin = " << floor( (getFloat(source,i+roi.y,j+roi.x)-range[0])/bin_size ) << endl;
    }*/
}

template<unsigned nBins>
inline float THistogram<nBins>::operator[](int i) const {
    return bin[i];
}

template<unsigned nBins>
inline float THistogram<nBins>::bin_center(int i) const {
    return (i+0.5)*bin_size() + range[0];
}

template<unsigned nBins>
inline int THistogram<nBins>::number_of_bins() const {
    return nBins;
}

template<unsigned nBins>
inline float THistogram<nBins>::bin_size() const {
    return (range[1] - range[0])/nBins;
}

template<unsigned nBins>
void THistogram<nBins>::show_contents() const {
    cout << "Bin: \tContent" << endl;
    for(int i = 0; i < nBins; i++)
    cout << i << "\t" << bin[i] << endl;
    cout << endl;
}

template<unsigned nBins>
inline float const *THistogram<nBins>::get_data() {
    return bin;
}

template<unsigned nBins>
void THistogram<nBins>::test_class() {
    int size_roi = 20;
    IplImage *img1 = cvCreateImage( cvSize(size_roi,size_roi), IPL_DEPTH_32F, 1 );
    cvSet( img1, cvScalarAll(1.0) );

    int size_image = 100;
    IplImage *img2 = cvCreateImage( cvSize(size_image,size_image), IPL_DEPTH_32F, 1 );
    CvRNG seed = cvRNG();
    cvRandArr( &seed, img2, CV_RAND_UNI, cvScalarAll(-10.0), cvScalarAll(20.0) );

    CvRect roi = cvRect( 25, 28, size_roi, size_roi );
    cvSetImageROI( img2, roi );

    THistogram<12> histogram(-30,30);
    histogram.show_contents();
    histogram.fill(img2, img1);
    histogram.show_contents();

    for(int i = 0; i < nBins; i++)
        cout << histogram.bin_center(i) << " -> " << histogram[i] << endl;
}



/********  class OrientationTHistogram  **********/

template<unsigned nBins>
OrientationTHistogram<nBins>::OrientationTHistogram(bool interpolation) {
    fill_fp = ( interpolation ? &OrientationTHistogram<nBins>::fill_interpolation : &OrientationTHistogram<nBins>::fill_no_interpolation );
}

template<unsigned nBins>
OrientationTHistogram<nBins>::~OrientationTHistogram() {}

template<unsigned nBins>
inline void OrientationTHistogram<nBins>::fill(IplImage const *source, IplImage const *weight, bool reset) {
    (this->*fill_fp)(source, weight, reset);
}

template<unsigned nBins>
void OrientationTHistogram<nBins>::fill_interpolation(IplImage const *source, IplImage const *weight, bool reset) {
    int i, j, b;
    CvRect roi_s = cvGetImageROI( source );
    CvRect roi_w = cvGetImageROI( weight );
    float b_size = bin_size();
    float c = bin_center(0);
    float dist,temp;

    if( reset )
        for(i = 0; i < nBins; i++)
            bin[i] = 0.0;

    for(i = 0; i < roi_s.height; i++)
        for(j = 0; j < roi_s.width; j++) {
            temp = getFloat(source, i+roi_s.y ,j+roi_s.x);
            b = static_cast<int>((temp - c)/b_size );
            dist = (temp - bin_center(b))/b_size;

            temp = getFloat(weight, i+roi_w.y, j+roi_w.x);
            if( dist < 0 ) {
                bin[b] += (1.0 + dist) * temp;
                bin[nBins-1] += -dist * temp;		//circular histogram...
            } else if( b == nBins-1 ) {
                bin[b] += (1.0 - dist) * temp;
                bin[0] += dist * temp;		//circular histogram...
            } else {
                bin[b] += (1.0 - dist) * temp;
                bin[b+1] += dist * temp;
            }
        }
}

template<unsigned nBins>
void OrientationTHistogram<nBins>::fill_no_interpolation(IplImage const *source, IplImage const *weight, bool reset) {
    int i, j;
    float b_size = bin_size();
    CvRect roi_s = cvGetImageROI( source );
    CvRect roi_w = cvGetImageROI( weight );

    if( reset )
        for(i = 0; i < nBins; i++)
            bin[i] = 0.0;

    for(i = 0; i < roi_s.height; i++)
        for(j = 0; j < roi_s.width; j++)
            bin[static_cast<int>( getFloat(source,i+roi_s.y,j+roi_s.x)/b_size )] +=
                getFloat(weight,i+roi_w.y,j+roi_w.x);
}


template<unsigned nBins>
inline float OrientationTHistogram<nBins>::operator[](int i) const {
    return bin[i];
}

template<unsigned nBins>
inline float OrientationTHistogram<nBins>::bin_center(int i) const {
    return (i+0.5)*bin_size();
}

template<unsigned nBins>
inline int OrientationTHistogram<nBins>::number_of_bins() const {
    return nBins;
}

template<unsigned nBins>
inline float OrientationTHistogram<nBins>::bin_size() const {
    return 360.0/nBins;
}

template<unsigned nBins>
inline float const *OrientationTHistogram<nBins>::get_data() {
    return bin;
}

template<unsigned nBins>
void OrientationTHistogram<nBins>::show_contents() const {
    cout << "Bin: \tContent" << endl;
    for(int i = 0; i < nBins; i++)
    cout << i << "\t" << bin[i] << endl;
    cout << endl;
}

template<unsigned nBins>
void OrientationTHistogram<nBins>::test_class() {
    CvRNG seed = cvRNG();
    CvRect roi;

    int size_roi = 20;
    IplImage *img1 = cvCreateImage( cvSize(size_roi,size_roi), IPL_DEPTH_32F, 1 );
    cvSet( img1, cvScalarAll(1.0) );

    int size_image = 100;
    IplImage *img2 = cvCreateImage( cvSize(size_image,size_image), IPL_DEPTH_32F, 1 );

    cvSetImageROI( img2, cvRect( 0, 0, size_image/2, size_image ) );
    cvRandArr( &seed, img2, CV_RAND_UNI, cvScalarAll(0.0), cvScalarAll(180.0) );
    cvSetImageROI( img2, cvRect( size_image/2, 0, size_image/2, size_image ) );
    cvRandArr( &seed, img2, CV_RAND_UNI, cvScalarAll(180.0), cvScalarAll(360.0) );


    OrientationTHistogram<12> histogram;

    roi = cvRect( 25, 28, size_roi, size_roi );
    cvSetImageROI( img2, roi );
    histogram.fill(img2, img1);
    histogram.show_contents();

    roi = cvRect( 51, 66, size_roi, size_roi );
    cvSetImageROI( img2, roi );
    histogram.fill(img2, img1);
    histogram.show_contents();

    roi = cvRect( 40, 66, size_roi, size_roi );
    cvSetImageROI( img2, roi );
    histogram.fill(img2, img1);
    histogram.show_contents();

    roi = cvRect( 48, 66, size_roi, size_roi );
    cvSetImageROI( img2, roi );
    histogram.fill(img2, img1);
    for(int i = 0; i < nBins; i++)
        cout << histogram.bin_center(i) << " -> " << histogram[i] << endl;
}

#endif
