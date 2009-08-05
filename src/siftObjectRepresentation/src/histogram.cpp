// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          2007 Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#include "histogram.hpp"


/*************  class Histogram  ****************/

Histogram::Histogram(int nb, float range_min, float range_max, bool interpolation) {
    fill_fp = ( interpolation ? &Histogram::fill_interpolation : &Histogram::fill_no_interpolation );

    range[0] = range_min;
    range[1] = range_max;

    nBins = nb;
    bin = new float[nBins];
}

Histogram::~Histogram() {
    delete[] bin;
}

inline void Histogram::fill(IplImage const *source, IplImage const *weight, bool reset) {
    (this->*fill_fp)(source, weight, reset);
}

void Histogram::fill_interpolation(IplImage const *source, IplImage const *weight, bool reset) {
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


void Histogram::fill_no_interpolation(IplImage const *source, IplImage const *weight, bool reset) {
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


inline float Histogram::operator[](int i) const {
    return bin[i];
}

inline float Histogram::bin_center(int i) const {
    return (i+0.5)*bin_size() + range[0];
}

inline int Histogram::number_of_bins() const {
    return nBins;
}

inline float Histogram::bin_size() const {
    return (range[1] - range[0])/nBins;
}

inline float const *Histogram::get_data() {
    return bin;
}

void Histogram::show_contents() const {
    cout << "Bin: \tContent" << endl;
    for(int i = 0; i < nBins; i++)
    cout << i << "\t" << bin[i] << endl;
    cout << endl;
}

void Histogram::test_class() {
    int size_roi = 20;
    IplImage *img1 = cvCreateImage( cvSize(size_roi,size_roi), IPL_DEPTH_32F, 1 );
    cvSet( img1, cvScalarAll(1.0) );

    int size_image = 100;
    IplImage *img2 = cvCreateImage( cvSize(size_image,size_image), IPL_DEPTH_32F, 1 );
    CvRNG seed = cvRNG();
    cvRandArr( &seed, img2, CV_RAND_UNI, cvScalarAll(-10.0), cvScalarAll(20.0) );

    CvRect roi = cvRect( 25, 28, size_roi, size_roi );
    cvSetImageROI( img2, roi );

    int nBins = 12;
    Histogram histogram1(nBins,-30,30);
    histogram1.fill(img2, img1);
    histogram1.show_contents();
    for(int i = 0; i < nBins; i++)
        cout << histogram1.bin_center(i) << " -> " << histogram1[i] << endl;

    Histogram histogram2(nBins,-30,30,1);
    histogram2.fill(img2, img1, 1);
    histogram2.show_contents();
    for(int i = 0; i < nBins; i++)
        cout << histogram2.bin_center(i) << " -> " << histogram2[i] << endl;

}



/********  class OrientationHistogram  **********/

OrientationHistogram::OrientationHistogram(int nb, bool interpolation) {
    fill_fp = ( interpolation ? &OrientationHistogram::fill_interpolation : &OrientationHistogram::fill_no_interpolation );

    nBins = nb;
    bin = new float[nBins];
}

OrientationHistogram::~OrientationHistogram() {
    delete[] bin;
}

inline void OrientationHistogram::fill(IplImage const *source, IplImage const *weight, bool reset) {
    (this->*fill_fp)(source, weight, reset);
}

void OrientationHistogram::fill_interpolation(IplImage const *source, IplImage const *weight, bool reset) {
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


void OrientationHistogram::fill_no_interpolation(IplImage const *source, IplImage const *weight, bool reset) {
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


inline float OrientationHistogram::operator[](int i) const {
    return bin[i];
}

inline float OrientationHistogram::bin_center(int i) const {
    return (i+0.5)*bin_size();
}

inline int OrientationHistogram::number_of_bins() const {
    return nBins;
}

inline float OrientationHistogram::bin_size() const {
    return 360.0/nBins;
}

float const *OrientationHistogram::get_data() {
    return bin;
}


void OrientationHistogram::show_contents() const {
    cout << "Bin: \tContent" << endl;
    for(int i = 0; i < nBins; i++)
    cout << i << "\t" << bin[i] << endl;
    cout << endl;
}

void OrientationHistogram::test_class() {
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

    int nBins = 12;
    OrientationHistogram histogram(nBins);

    roi = cvRect( 25, 28, size_roi, size_roi );
    cvSetImageROI( img2, roi );
    histogram.fill(img2, img1);
    histogram.show_contents();
    OrientationHistogram histogram1(nBins,1);
    histogram1.fill(img2, img1);
    histogram1.show_contents();

    roi = cvRect( 51, 66, size_roi, size_roi );
    cvSetImageROI( img2, roi );
    histogram.fill(img2, img1);
    histogram.show_contents();
    histogram1.fill(img2, img1);
    histogram1.show_contents();

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
