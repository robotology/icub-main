// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          2007 Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#include "sift.hpp"
#include "image_wrapper.hpp"
#include "common.hpp"
#include "keypoint.hpp"

//Rob Hess sift features
//extern "C"{ //fazer isto quando as funções estao definidas em *.c
#include "siftRH.h"
#include "kdtree.h"
#include "imgfeatures.h"
//}

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>

#include <highgui.h>

#define IDX(O,S) (total_scales*(O) + (S))


#define BDD_SIFT_MESSAGE
#ifdef BDD_SIFT_MESSAGE
#define MESSAGE( A ) ( cout << "SIFT: " <<  A << endl )
#else
#define MESSAGE( A )
#endif

#include <time.h>
#include <algorithm>
#include <vector>

#include "common_functions.h"

//timestamp
#include <ace/Time_Value.h>
#include <ace/Date_Time.h>

/********  Static ********/

SIFT_Image::Param SIFT_Image::param;
bool SIFT_Image::initialized = false;
// int max_number_per_image = 175;

bool SIFT_Image::init( string const &filename ) {
    //Default values
    param.depth = IPL_DEPTH_32F;
    param.max_keypoints = 20000;
    param.delete_images_after_processing = true;

    param.scales_per_octave = SIFT_INTVLS;
    param.prior_sigma = SIFT_INIT_SIGMA;
    param.initial_sigma = SIFT_SIGMA;

    param.m_contrast = SIFT_CONTR_THR;
    param.edge_ratio = SIFT_CURV_THR;

    param.orientation_window_size = 9;
    param.orientation_sigma_factor = SIFT_ORI_SIG_FCTR;
    param.orientation_nbins = SIFT_ORI_HIST_BINS;
    param.multiple_peaks_factor = SIFT_ORI_PEAK_RATIO;

    param.descriptor_bin_size = SIFT_DESCR_WIDTH;
    param.descriptor_n_bins = 4; 
    param.descriptor_n_orientations = 8;
    param.descriptor_lin_interpolation = true;
    param.descriptor_sigma_factor = 0.5;
    param.descriptor_mag_threshold = SIFT_DESCR_MAG_THR;

    //param.max_number_per_image=175;

    Keypoint::set_descriptor_size( SQUARE(param.descriptor_n_bins)*param.descriptor_n_orientations );
    initialized = true;



    if( filename == "" )
        return true;

    // Read from file
    ifstream file;
    file.open( filename.c_str() );
    if( !file.is_open() ) {
        MESSAGE("Error reading configuration file" << filename);
        return false;
    }

    string buffer;
    MESSAGE("Loading parameters from " << filename);
    while( NOT (file >> buffer).eof() ) {
        if( !strcmp(buffer.c_str(), "scales_per_octave") ) {
            if( file >> param.scales_per_octave )
                MESSAGE("\t scales_per_octave: " << param.scales_per_octave);
        } else if( !strcmp(buffer.c_str(), "prior_sigma") ) {
            if( file >> param.prior_sigma )
                MESSAGE("\t prior_sigma: " << param.prior_sigma);
        } else if( !strcmp(buffer.c_str(), "initial_sigma") ) {
            if( file >> param.initial_sigma )
                MESSAGE("\t initial_sigma: " << param.initial_sigma);
        } else if( !strcmp(buffer.c_str(), "m_contrast") ) {
            if( file >> param.m_contrast )
                MESSAGE("\t m_contrast: " << param.m_contrast);
        } else if( !strcmp(buffer.c_str(), "edge_ratio") ) {
            if( file >> param.edge_ratio )
                MESSAGE("\t edge_ratio: " << param.edge_ratio);
        } else if( !strcmp(buffer.c_str(), "orientation_window_size") ) {
            if( file >> param.orientation_window_size )
                MESSAGE("\t orientation_window_size: " << param.orientation_window_size);
        } else if( !strcmp(buffer.c_str(), "orientation_sigma_factor") ) {
            if( file >> param.orientation_sigma_factor )
                MESSAGE("\t orientation_sigma_factor: " << param.orientation_sigma_factor);
        } else if( !strcmp(buffer.c_str(), "orientation_nbins") ) {
            if( file >> param.orientation_nbins )
                MESSAGE("\t orientation_nbins: " << param.orientation_nbins);
        } else if( !strcmp(buffer.c_str(), "multiple_peaks_factor") ) {
            if( file >> param.multiple_peaks_factor )
                MESSAGE("\t multiple_peaks_factor: " << param.multiple_peaks_factor);
        } else if( !strcmp(buffer.c_str(), "descriptor_bin_size") ) {
            if( file >> param.descriptor_bin_size )
                MESSAGE("\t descriptor_bin_size: " << param.descriptor_bin_size);
        } else if( !strcmp(buffer.c_str(), "descriptor_n_bins") ) {
            if( file >> param.descriptor_n_bins )
                MESSAGE("\t descriptor_n_bins: " << param.descriptor_n_bins);
        } else if( !strcmp(buffer.c_str(), "descriptor_n_orientations") ) {
            if( file >> param.descriptor_n_orientations )
                MESSAGE("\t descriptor_n_orientations: " << param.descriptor_n_orientations);
        } else if( !strcmp(buffer.c_str(), "descriptor_lin_interpolation") ) {
            if( file >> param.descriptor_lin_interpolation )
                MESSAGE("\t descriptor_lin_interpolation: " << param.descriptor_lin_interpolation);
        } else if( !strcmp(buffer.c_str(), "descriptor_sigma_factor") ) {
            if( file >> param.descriptor_sigma_factor )
                MESSAGE("\t descriptor_sigma_factor: " << param.descriptor_sigma_factor);
        } else if( !strcmp(buffer.c_str(), "descriptor_mag_threshold") ) {
            if( file >> param.descriptor_mag_threshold )
                MESSAGE("\t descriptor_mag_threshold: " << param.descriptor_mag_threshold);
        } else if( !strcmp(buffer.c_str(), "delete_images_after_processing") ) {
            if( file >> param.delete_images_after_processing )
                MESSAGE("\t delete_images_after_processing: " << param.delete_images_after_processing);
        }
        // 		else if( !strcmp(buffer.c_str(), "max_number_per_image") )
        // 		{
        // 			if( file >> param.max_number_per_image )
        // 				MESSAGE("\t max_number_per_image: " << param.max_number_per_image);
        // 		}

        file.clear();   //Just to override eventual garbage at the end of the line
        file.ignore(100,'\n');
    }

    file.close();

    Keypoint::set_descriptor_size( SQUARE(param.descriptor_n_bins)*param.descriptor_n_orientations );

    return true;
}

void SIFT_Image::test_class() {
    SIFT_Image::init();
    SIFT_Image img;
    //	img.load("images/lena.jpg");
    img.load("images/baboon.jpg");
    //	img.load("images/pic3.png");
    //	cvWaitKey(0);

    img.show("Original Image");
    img.show_gaussians("Gaussians");
    img.show_diff_of_gaussians("DOG's");
    img.show_gradient( "Gradient" );
    //	cvWaitKey(0);
}


/********  Public ********/

void SIFT_Image::copy(SIFT_Image const &other) { ///MEMORY LEAK in here, dunno where
    // 	cout << "in SIFT_Image::copy(SIFT_Image const &other)"<< endl;//DEBUG
    image = other.image;
    image_color = other.image_color;
    if( image )
        image = cvCloneImage( other.image );
    if( image_color )
        image_color = cvCloneImage( other.image_color );
    
    gaussians = other.gaussians;
    DOG = other.DOG;
    gradient_mag = other.gradient_mag;
    gradient_theta = other.gradient_theta;

    keypoints  = other.keypoints;

    n_keypoints = other.n_keypoints;
    n_high_contrast_keypoints = other.n_high_contrast_keypoints;
    n_stable_keypoints = other.n_stable_keypoints;
    n_total_keypoints = other.n_total_keypoints;

    number_of_octaves = other.number_of_octaves;
    total_scales = other.total_scales;
    sigma = other.sigma;
    k = other.k;

    label = other.label;
    frame_time = other.frame_time;
    sec = other.sec;
    usec = other.usec;

    int n = number_of_octaves*total_scales;
    if( gaussians ) {
        gaussians = new IplImage *[n];
        for(int i = 0; i < n; i++)
            gaussians[i] =  cvCloneImage( other.gaussians[i] );
    }

    n = number_of_octaves*(total_scales-1);
    if( DOG ) {
        DOG = new IplImage *[n];
        for(int i = 0; i < n; i++)
            DOG[i] =  cvCloneImage( other.DOG[i] );
    }

    n = number_of_octaves*(total_scales-3);
    if( gradient_mag ) {
        gradient_mag = new IplImage *[n];
        gradient_theta = new IplImage *[n];
        for(int i = 0; i < n; i++) {
            gradient_mag[i] =  cvCloneImage( other.gradient_mag[i] );
            gradient_theta[i] =  cvCloneImage( other.gradient_theta[i] );
        }
    }

    this->number_of_features = other.number_of_features;
    this->features = (feature *) calloc( this->number_of_features, sizeof(struct feature) );
    if(this->features == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}
/*    this->keypoint_indexes = (int *) calloc( this->number_of_features, sizeof(int) );
    if(this->keypoint_indexes == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}*/
    for(int dur = 0; dur < this->number_of_features; dur++) {
        clone_feature((this->features + dur), (other.features + dur));
//         this->keypoint_indexes[dur] = other.keypoint_indexes[dur];
    }
    this->kd_image_root = kdtree_build( this->features, this->number_of_features );
    ///on database loading, this function is called twice, is this correct?
    //cout << "out SIFT_Image::copy(SIFT_Image const &other)"<< endl; 
}

void SIFT_Image::destroy() {
    cvReleaseImage(&image );
    cvReleaseImage(&image_color);

    if( !param.delete_images_after_processing )
        delete_aux_images();

    kdtree_release( this->kd_image_root );
    if(this->features != NULL) {
        for(int i = 0; i < this->number_of_features; i++) {
            if((this->features + i)->feature_data != NULL)
                free((this->features + i)->feature_data);
        }
        free(this->features);
//         free(this->keypoint_indexes);
    }
}

void SIFT_Image::start() {
    image = 0;
    image_color = 0;
    gaussians = 0;
    DOG = 0;
    gradient_mag = 0;
    gradient_theta = 0;

    n_keypoints = 0;
    n_high_contrast_keypoints = 0;
    n_stable_keypoints = 0;
    n_total_keypoints = 0;

    features = NULL;
    number_of_features = 0;
    kd_image_root = NULL;
//     keypoint_indexes = NULL;

    label.clear();
    frame_time = -1;
    sec = -1;
    usec = -1;


    if( !initialized )
        init();

    keypoints.reset(param.max_keypoints);
}


void SIFT_Image::restart() {
    destroy();
    start();
}


SIFT_Image::SIFT_Image( SIFT_Image const &other) {
    copy( other );
}


SIFT_Image const &SIFT_Image::operator=(SIFT_Image const &other) {
    if (this != &other) {
        destroy();
        copy(other);
    }
    return *this;
}


SIFT_Image::SIFT_Image() {
    start();
    //MESSAGE("created an object :D");
}

SIFT_Image::SIFT_Image( string const &filename ) {
    start();
    load( filename );
}

SIFT_Image::SIFT_Image( IplImage const *img ) {
    start();
    load( (IplImage *) img );
}

void SIFT_Image::add_image(IplImage const *img) {
    this->image = (IplImage*) img;
}

SIFT_Image::~SIFT_Image() {
    destroy();
}

bool SIFT_Image::load( string const &filename ) {
    IplImage *img;
    if( (img = cvLoadImage( filename.c_str() )) == 0 ) {
        cerr << "Could not load image file: " << filename << endl;
        return false;
    }

    load( (IplImage * const) img );
    cvReleaseImage( &img );

    MESSAGE("Loaded " << filename << ".");
    return true;
}

bool SIFT_Image::load( IplImage const *img ) {
    if( img == 0 )
        return false;

    if( image != 0 ) //image already loaded
        restart();

    frame_time = (double)cvGetTickCount();
    {
        ACE_Time_Value t = ACE_OS::gettimeofday();
        sec = t.sec();
        usec = t.usec();
    }

    image = new_image( cvGetSize(img) );
    this->image_color = cvCloneImage(img);

    if( img->nChannels > 1 ) {
        IplImage *tmp = cvCreateImage( cvGetSize(image), param.depth, img->nChannels );
        cvConvertScale( img, tmp, 1.0/255 );
        cvCvtColor( tmp, image, CV_BGR2GRAY );
        cvReleaseImage( &tmp );
    } else
        cvConvertScale( img, image, 1.0/255 );

    extract_features();
    /*	if(features == NULL) cout << "features == NULL" << endl;//DEBUG
    	else if(kd_image_root == NULL) cout << "kd_image_root == NULL" << endl;//DEBUG*/
    // 	else
    // 	{
    // 		cout <<"x="<< features->x <<" y="<< features->y <<" scl="<< features->scl <<" ori="<< features->ori << endl; //DEBUG
    //
    // 	cout << "kd_image_root->n=" << kd_image_root->n;
    // 	cout << " feature: "<<" x="<< kd_image_root->features->x <<" y="<< kd_image_root->features->y <<" scl="<< kd_image_root->features->scl <<" ori="<< kd_image_root->features->ori << endl;; //DEBUG
    //
    // 	cout << "kd_image_root->kd_left->n=" << kd_image_root->kd_left->n << endl;
    // 	cout << " feat: "<<" x="<< kd_image_root->kd_left->features->x <<" y="<< kd_image_root->kd_left->features->y <<" scl="<< kd_image_root->kd_left->features->scl <<" ori="<< kd_image_root->kd_left->features->ori << endl;; //DEBUG
    //
    // 	cout << "kd_image_root->kd_right->n=" << kd_image_root->kd_right->n << endl;
    // 	cout << " feat: "<<" x="<< kd_image_root->kd_right->features->x <<" y="<< kd_image_root->kd_right->features->y <<" scl="<< kd_image_root->kd_right->features->scl <<" ori="<< kd_image_root->kd_right->features->ori << endl;; //DEBUG
    // 	}
    return true;
}

// bool SIFT_Image::load( string const &filename , int trim) //So not to change the originals, we trim only here
// {
// 	IplImage *img;
// 	if( (img = cvLoadImage( filename.c_str() )) == 0 )
// 	{
// 		cerr << "Could not load image file: " << filename << endl;
// 		return false;
// 	}
//
// 	load( img , trim);
// 	cvReleaseImage( &img );
//
// 	MESSAGE("Loaded " << filename << ".");
// 	return true;
// }

// bool SIFT_Image::load( IplImage const *img , int trim) //So not to change the originals, we trim only here
// {
// 	if( img == 0 )
// 		return false;
//
// 	if( image != 0 ) //image already loaded
// 		restart();
//
// 	image = new_image( cvGetSize(img) );
//
// 	if( img->nChannels > 1 )
// 	{
// 		IplImage *tmp = cvCreateImage( cvGetSize(image), param.depth, img->nChannels );
// 		cvConvertScale( img, tmp, 1.0/255 );
// 		cvCvtColor( tmp, image, CV_BGR2GRAY );
// 		cvReleaseImage( &tmp );
// 	}
// 	else
// 		cvConvertScale( img, image, 1.0/255 );
//
// 	extract_features(trim);
// 	return true;
// }

SIFT_Image::operator IplImage const &() {
    return *image;
}

IplImage const *SIFT_Image::get_image() const {
    return image;
}

IplImage const *SIFT_Image::get_image_color() const {
    return image_color;
}


Database<Keypoint> &SIFT_Image::get_keypoints() {
    return keypoints;
}



void SIFT_Image::show( string const &window_name ) {
    if( image_color == 0 )
        return;

    cvNamedWindow( window_name.c_str() );
    show_and_wait_key( window_name, this->image_color );
    cvDestroyWindow( window_name.c_str() );
}

void SIFT_Image::show_gaussians( string const &window_name ) {
    if( gaussians == 0 )
        return;

    cvNamedWindow( window_name.c_str() );
    for(int f = 0; f < number_of_octaves*total_scales; f++)
        show_and_wait_key( window_name, gaussians[f] );
    cvDestroyWindow( window_name.c_str() );
}

void SIFT_Image::show_diff_of_gaussians( string const &window_name ) {
    if( DOG == 0 )
        return;


    cvNamedWindow( window_name.c_str() );
    for(int f = 0; f < number_of_octaves*(total_scales-1); f++) {
        IplImage *tmp = new_image(cvSize(DOG[f]->width,DOG[f]->height));
        cvConvertScale( DOG[f], tmp, 1.0/(k - 1));
        show_and_wait_key( window_name, DOG[f] );
        cvReleaseImage(&tmp);
    }
    cvDestroyWindow( window_name.c_str() );
}

void SIFT_Image::show_gradient( string const &window_name ) {
    if( gradient_mag == 0 || gradient_theta == 0 )
        return;

    cvNamedWindow( window_name.c_str() );
    for(int f = 0; f < number_of_octaves*(total_scales-3); f++)
        show_and_wait_key( window_name, gradient_mag[f] );
    for(int f = 0; f < number_of_octaves*(total_scales-3); f++)
        show_and_wait_key( window_name, gradient_theta[f] );
    cvDestroyWindow( window_name.c_str() );
}


/*************************/

// void SIFT_Image::show_and_wait_key( string const &window_name, IplImage *img ) //feito pra mostrar this->image, q é tratada ao entrar
// {
// 	cvResetImageROI( img );
// 	IplImage *tmp = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_8U, 1 );
// 	cvConvertScaleAbs( img, tmp, 255 );
// 	cvShowImage(window_name.c_str(), tmp );
//
// 	cvWaitKey(0);
// 	cvReleaseImage( &tmp );
// }

void SIFT_Image::show_and_wait_key( string const &window_name, IplImage *img ) //feito pra mostrar this->image_color, q não é tratada ao entrar
{
    cvShowImage(window_name.c_str(), img );

    cvWaitKey(0);
}

IplImage *SIFT_Image::new_image( CvSize s ) {
    return cvCreateImage( s, param.depth, 1 );
}


void SIFT_Image::create_aux_images() {
    int s;
    int max_size = MAX(image->width, image->height);
    CvSize o_size;

    number_of_octaves = 2 + static_cast<int>(floor(
                            log(static_cast<double>(max_size) / (2*param.orientation_window_size)) / log(2) ));
    total_scales = param.scales_per_octave + 3;
    k = pow(2, 1.0/param.scales_per_octave);
    //amount of blurring necessary to obtain initial_sigma
    sigma = sqrt( SQR(param.initial_sigma) - 4*SQR(param.prior_sigma) );

    //allocate array of images (no memory to the images yet)
    gaussians = new IplImage *[number_of_octaves*total_scales];
    DOG = new IplImage *[number_of_octaves*(total_scales-1)];
    gradient_mag = new IplImage *[number_of_octaves*(param.scales_per_octave)];
    gradient_theta = new IplImage *[number_of_octaves*(param.scales_per_octave)];

    for(int o = 0; o < number_of_octaves; o++) {
        //start with an image with twice the size
        o_size = cvSize(static_cast<int>(ldexp(image->width,1-o)), static_cast<int>(ldexp(image->height,1-o)));

        for(s = 0;
                s < total_scales-3;
                s++) {
            gaussians[total_scales*o + s]
            = new_image( o_size );
            DOG[(total_scales-1)*o + s] = new_image( o_size );
            gradient_mag[param.scales_per_octave*o + s] = new_image( o_size );
            gradient_theta[param.scales_per_octave*o + s] = new_image( o_size );
        }
        for(s = total_scales-3;
                s < total_scales-1;
                s++) {
            gaussians[total_scales*o + s]
            = new_image( o_size );
            DOG[(total_scales-1)*o + s] = new_image( o_size );
        }
        s = total_scales-1;
        gaussians[total_scales*o + s] = new_image( o_size );
    }
}


void SIFT_Image::delete_aux_images() {
    int n = number_of_octaves*total_scales;
    for(int i = 0; i < n; i++)
        cvReleaseImage( gaussians + i );

    n = number_of_octaves*(total_scales-1);
    for(int i = 0; i < n; i++)
        cvReleaseImage( DOG + i );

    n = number_of_octaves*(total_scales-3);
    for(int i = 0; i < n; i++) {
        cvReleaseImage( gradient_mag + i );
        cvReleaseImage( gradient_theta + i );
    }

    delete[]	gaussians;
    delete[] DOG;
    delete[] gradient_mag;
    delete[] gradient_theta;

    gaussians = DOG = gradient_mag = gradient_theta = 0;
}


int SIFT_Image::add_to_database(Keypoint const &k ) {
    int i;

    if( (i = keypoints.insert_record( k )) == -1) {
        MESSAGE( "Resizing keypoint database... previous capacity=" << keypoints.get_capacity());
        keypoints.resize(2*keypoints.get_capacity());
        MESSAGE( "Current capacity=" << keypoints.get_capacity());

        if( (i = keypoints.insert_record( k )) == -1) {
            cerr << "SIFT_Image::add_to_database: could not allocate memory. Quiting " << endl;
            exit(1);
        }
    }

    return i;
}


void SIFT_Image::set_keypoint_th(Keypoint &k, float theta ) {
    k.th = theta;
}


void SIFT_Image::adjust_keypoint(Keypoint &k, float dx, float dy, float ds ) {
    k.s = pow( 2, k.octave-1 + (k.scale-1.0 + ds)/param.scales_per_octave );
    k.x = ldexp( k.col + dx, k.octave-1 );
    k.y = ldexp( k.row + dy, k.octave-1 );
}

/*************************/

void SIFT_Image::extract_features() {
//     time_t start, end, start_all;
    double dif=0;
    struct feature* feat, *aux;
    int n;
    Keypoint kp;

    double t_start = (double)cvGetTickCount();
    // 	MESSAGE("Extracting features");

#define COMPARE 0
    if(COMPARE){ ///put COMPARE to 1 to let it calculate the Bruno Damas sifts and show how long it took, to compare times
        double t_start_all = (double)cvGetTickCount();
        //  time(&start);
        //  MESSAGE("Extracting features");
    
        //  MESSAGE( "\tCreating aux images...");
        create_aux_images();
        /*  time(&end);
        dif = end - start;
        MESSAGE("it took " << dif << " seconds.");
        
        time(&start);   */
        //  MESSAGE( "\tCreating pyramids...");
        build_gaussians();
        build_DOG();
        build_gradients();
        /*  time(&end);
        dif = end - start;
        MESSAGE("it took " << dif << " seconds.");*/
    
    
        //  time(&start);
        //  MESSAGE( "\tFinding keypoints...");
        find_keypoints();
        //  MESSAGE( "\t\tFound " << n_keypoints << " keypoints...");
        remove_unstable_keypoints();
        //  MESSAGE( "\t\tRemoved " << n_keypoints-n_high_contrast_keypoints << " low contrast keypoints");
        //  MESSAGE( "\t\tRemoved " << n_high_contrast_keypoints-n_stable_keypoints << " edge keypoints");
    
    
        orientation_assignment();
        //  MESSAGE( "\t\tOrientation assignment duplicated " << n_total_keypoints-n_stable_keypoints << " keypoints.");
    
        keypoints.shrink_size();
        //  MESSAGE( "\tA total number of " << n_total_keypoints << " keypoints was found." );
        //  time(&end);
        //  dif = end - start;
        //  MESSAGE("it took " << dif << " seconds.");
        //
        //  time(&start);
        //  MESSAGE( "\tBuilding descriptors...");
        keypoint_descriptor();
        //  time(&end);
        //  dif = end - start;
        //  MESSAGE("it took " << dif << " seconds.");
    
        if( param.delete_images_after_processing ) {
    //         MESSAGE( "\tDeleting aux images...");
            delete_aux_images();
        }
    
        //  MESSAGE( "" );
        double t_end_all = (double)cvGetTickCount() - t_start_all;
        printf( "SIFT:: Extracted KEYPOINTS in %gms\n", t_end_all/(cvGetTickFrequency()*1000.) );
        
        print_all_keypoints();
        draw_keypoints(CROSS);

    }

    
    
    ///Rob Hess keypoints or as he calls them "features"
    // 	display_image(image); //DEBUG
    n = sift_features( (IplImage*) image_color, &feat );
    double t_end1 = (double)cvGetTickCount() - t_start;
    //  	printf( "SIFT:: exec time of sift_features()= %gms\n", t_end1/(cvGetTickFrequency()*1000.) );
    //   	MESSAGE("Found " << n << " features.");
    this->features = feat;
    this->number_of_features = n;
    double t_start1 = (double)cvGetTickCount();
    this->kd_image_root = kdtree_build( feat, n );
    t_end1 = (double)cvGetTickCount() - t_start1;
    //  	printf( "SIFT:: exec time of kdtree_build()= %gms\n", t_end1/(cvGetTickFrequency()*1000.) );

//     this->keypoint_indexes = (int *) calloc( this->number_of_features, sizeof(int) );
//     if(this->keypoint_indexes == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}

    double t_start2 = (double)cvGetTickCount();
    keypoints.reset(param.max_keypoints);

    draw_features(RECTANGLE);
    int index;
    for(int i = 0; i < n; i++ ) {
        aux = feat + i;

        kp = feature2keypoint(aux);
        

        index = add_to_database( kp );
        aux->keypoint_index = index;
//         this->keypoint_indexes[i] = index;


        //db_index = get_current();
        // 		MESSAGE("Out feature2keypoint: kp.s = " << kp.s << "DB index: " << db_index); //DEBUG
    }
    /*	double t_end2 = (double)cvGetTickCount() - t_start2;*/

    
    double t_end = (double)cvGetTickCount() - t_start;
//         printf( "SIFT:: Extracted %d features in %gms\n", n, t_end/(cvGetTickFrequency()*1000.) );
        // 	printf( "SIFT:: exec time of passing features to keypoints()= %gms\n", t_end2/(cvGetTickFrequency()*1000.) );

 
        
}

        //cout<< kp <<endl; ///print keypoint
           ///Find closest feature in the database(computed by Luis Damas code) and compare
//         int k_i;
//         float minim = 99999999.;
//         int best_match = -1;
//         Keypoint Kaux;
//         for(int i = 0, k_i = keypoints.go_to_first(); i < keypoints.get_size(); i++, k_i = keypoints.go_to_next() ) {
//             if(kp.distance( keypoints[k_i] ) < minim){
//                 minim = kp.distance( keypoints[k_i]);
//                 best_match = k_i;
//             }
//         }
//         cout << kp << " !! "<< keypoints[best_match]<< " = " << minim<<endl;

        
        

void SIFT_Image::print_all_keypoints(){
    cout<<"octave scale row col s x y th"<<endl;
    int k_i;
    for(int i = 0, k_i = keypoints.go_to_first(); i < keypoints.get_size(); i++, k_i = keypoints.go_to_next() ) {
        cout<< keypoints[k_i] <<endl;
    }
    cout<<"printing ended."<<endl;
}
// void SIFT_Image::extract_features(int trim) //so not to change the original extract functions, we trim only in this one
// {
// 	time_t start, end, start_all;
// 	double dif=0;
//
// 	time(&start_all);
// // 	time(&start);
// 	MESSAGE("Extracting features");
//
// 	MESSAGE( "\tCreating aux images...");
// 	create_aux_images();
// /*	time(&end);
// 	dif = end - start;
// 	MESSAGE("it took " << dif << " seconds.");
//
// 	time(&start);	*/
// 	MESSAGE( "\tCreating pyramids...");
// 	build_gaussians();
// 	build_DOG();
// 	build_gradients();
// /*	time(&end);
// 	dif = end - start;
// 	MESSAGE("it took " << dif << " seconds.");*/
//
// 	keypoints.reset(param.max_keypoints);
//
// // 	time(&start);
// 	MESSAGE( "\tFinding keypoints...");
// 	find_keypoints();
// 	MESSAGE( "\t\tFound " << n_keypoints << " keypoints...");
// 	remove_unstable_keypoints();
// 	MESSAGE( "\t\tRemoved " << n_keypoints-n_high_contrast_keypoints << " low contrast keypoints");
// 	MESSAGE( "\t\tRemoved " << n_high_contrast_keypoints-n_stable_keypoints << " edge keypoints");
//
// 	if(trim)
// 	{
// 		MESSAGE( "Trimming... There were " << n_stable_keypoints << " keipoints");
// 		trim_keypoints();
// 	}
//
// 	orientation_assignment();
// 	MESSAGE( "\t\tOrientation assignment duplicated " << n_total_keypoints-n_stable_keypoints << " keypoints.");
//
// 	keypoints.shrink_size();
// 	MESSAGE( "\tA total number of " << n_total_keypoints << " keypoints was found." );
// // 	time(&end);
// // 	dif = end - start;
// // 	MESSAGE("it took " << dif << " seconds.");
// //
// // 	time(&start);
// 	MESSAGE( "\tBuilding descriptors...");
// 	keypoint_descriptor();
// // 	time(&end);
// // 	dif = end - start;
// // 	MESSAGE("it took " << dif << " seconds.");
//
// 	if( param.delete_images_after_processing )
// 	{
// 		MESSAGE( "\tDeleting aux images...");
// 		delete_aux_images();
// 	}
//
// 	MESSAGE( "" );
// 	time(&end);
// 	dif = end - start_all;
// 	MESSAGE("The whole process took " << dif << " seconds.");
//
// }

void SIFT_Image::build_gaussians() {
    cvResize( image, gaussians[0] );
    cvSmooth( gaussians[0], gaussians[0], CV_GAUSSIAN, 0, 0, sigma );

    //get the other images (recursively)
    build_next_gaussian(0,1);
}


void SIFT_Image:: build_next_gaussian(int octave, int scale) {
    if( scale == 0 ) {
        cvResize( gaussians[IDX(octave-1,total_scales-3)], gaussians[IDX(octave,0)] );
        build_next_gaussian(octave, 1);
    } else {
        cvSmooth( gaussians[IDX(octave,0)], gaussians[IDX(octave,scale)],
                  CV_GAUSSIAN, 0, 0, sigma*sqrt(pow(k, 2*scale) - 1) );
        if( scale == total_scales-1 )
            if( octave == number_of_octaves-1 )
                return;
            else
                build_next_gaussian(octave+1, 0);
        else
            build_next_gaussian(octave, scale+1);
    }
}

void SIFT_Image::build_DOG() {
    for( int o = 0; o < number_of_octaves; o++ )
        for( int s = 0; s < total_scales-1; s++ )
            //		{
            cvSub( gaussians[IDX(o,s+1)], gaussians[IDX(o,s)], DOG[(total_scales-1)*o + s] );
    //			cvConvertScale( DOG[(total_scales-1)*o + s], DOG[(total_scales-1)*o + s], 1.0/(k - 1));
    //		}
}

void SIFT_Image::build_gradients() {
    int idx1,idx2;

    for( int o = 0; o < number_of_octaves; o++ )
        for( int s = 0; s < param.scales_per_octave; s++ ) {
            idx1 = total_scales*o + s + 1;
            idx2 = param.scales_per_octave*o + s;
            cvSobel( gaussians[idx1], gradient_mag[idx2], 1, 0, 1 );		//temp dx
            cvSobel( gaussians[idx1], gradient_theta[idx2], 0, 1, 1 );	//temp dy
            cvCartToPolar( gradient_mag[idx2], gradient_theta[idx2], gradient_mag[idx2], gradient_theta[idx2], 1 );
        }
}


void SIFT_Image::find_keypoints() {

    int width, height;
    int min_row = (param.orientation_window_size-1)/2;  // =1
    int min_col = min_row;
    int max_row, max_col;
    Keypoint kp;
    int n_keypoints_old = 0;

    for( int octave = 0; octave < number_of_octaves; octave++ ) {
        max_row = DOG[(total_scales-1)*octave]->height - min_row;
        max_col = DOG[(total_scales-1)*octave]->width - min_col;

        for( int scale = 1; scale <= param.scales_per_octave; scale++ )
            for( int row = min_row; row < max_row; row++ )
                for( int col = min_col; col < max_col; col++) {
                    kp.set_pos( octave, scale, row, col );
                    if( iskeypoint( kp ) )
                        add_to_database( kp );
                }
//         MESSAGE("\t\t\tOctave " << octave << ": " << keypoints.get_size() - n_keypoints_old << " keypoints found");
        n_keypoints_old = keypoints.get_size();
        //cout << "Octave " << octave << ": " << keypoints.get_size() << "keypoints" << endl;
    }
    n_keypoints = keypoints.get_size();
}


bool SIFT_Image::iskeypoint( Keypoint const &kp ) {
    int r,c;
    int idx = (total_scales-1)*kp.octave + kp.scale;

    float keypoint = getFloat( DOG[idx], kp.row, kp.col );

    if( getFloat(DOG[idx], kp.row, kp.col-1) > keypoint )
        // looking for minimum
    {
        if( getFloat(DOG[idx], kp.row, kp.col+1) <= keypoint )
            return false;
        for( c = kp.col-1; c <= kp.col+1; c++ )
            if( getFloat(DOG[idx], kp.row-1, c) <= keypoint || getFloat(DOG[idx], kp.row+1, c) <= keypoint )
                return false;
        for( r = kp.row-1; r <= kp.row+1; r++ )
            for( c = kp.col-1; c <= kp.col+1; c++ )
                if( getFloat( DOG[idx-1],r,c) <= keypoint || getFloat( DOG[idx+1],r,c) <= keypoint )
                    return false;
    } else if( getFloat(DOG[idx], kp.row, kp.col-1) < keypoint )
        // looking for maximum
    {
        if( getFloat(DOG[idx], kp.row, kp.col+1) >= keypoint )
            return false;
        for( c = kp.col-1; c <= kp.col+1; c++ )
            if( getFloat(DOG[idx], kp.row-1, c) >= keypoint || getFloat(DOG[idx], kp.row+1, c) >= keypoint )
                return false;
        for( r = kp.row-1; r <= kp.row+1; r++ )
            for( c = kp.col-1; c <= kp.col+1; c++ )
                if( getFloat( DOG[idx-1],r,c) >= keypoint || getFloat( DOG[idx+1],r,c) >= keypoint )
                    return false;
    } else
        return false;

    return true;
}


void SIFT_Image::remove_unstable_keypoints() {
    CvMat *H = cvCreateMat( 3, 3, CV_32FC1 );	//Hessian
    CvMat *J = cvCreateMat( 3, 1, CV_32FC1 );	//Jacobian
    CvMat *X = cvCreateMat( 3, 1, CV_32FC1 );	//Solution of 3d_fitting
    cvSetZero( H );

    for( int k_i = keypoints.go_to_last(); k_i != -1; k_i = keypoints.go_to_previous() ) {
        build_Hessian_and_Jacobian( H, J, keypoints[k_i] );

        if( low_contrast( H, J, X, keypoints[k_i] ) || is_edge( H, J, keypoints[k_i] ) )
            keypoints.remove_record(k_i);
        else
            adjust_keypoint( keypoints[k_i], cvmGet(X,0,0), cvmGet(X,1,0), cvmGet(X,2,0) );
    }

    cvReleaseMat( &H );
    cvReleaseMat( &J );
    cvReleaseMat( &X );

    n_stable_keypoints = keypoints.get_size();
}

// void SIFT_Image::trim_keypoints()
// {
// 	int max = param.max_number_per_image;
//
//
// //	vector<vect_keyp> ;
//
// 	for(int k_i = keypoints.go_to_last(); n_stable_keypoints > max; k_i = keypoints.go_to_previous() )
// 	{
// 		keypoints.remove_record(k_i);
// 		n_stable_keypoints--;
// 	}
// }

void SIFT_Image::build_Hessian_and_Jacobian( CvMat *H, CvMat *J, Keypoint const &kp ) {
    int idx = (total_scales-1)*kp.octave + kp.scale;

    //	Jacobian
    cvmSet(J, 0, 0, 0.5*( getFloat( DOG[idx], kp.row, kp.col+1 ) - getFloat( DOG[idx], kp.row, kp.col-1 )));
    cvmSet(J, 1, 0, 0.5*( getFloat( DOG[idx], kp.row+1, kp.col ) - getFloat( DOG[idx], kp.row-1, kp.col )));
    cvmSet(J, 2, 0, 0.5*( getFloat( DOG[idx+1], kp.row, kp.col ) - getFloat( DOG[idx-1], kp.row, kp.col )));

    // Hessian
    cvmSet(H, 0, 0, getFloat( DOG[idx], kp.row, kp.col+1 ) + getFloat( DOG[idx], kp.row, kp.col-1 )	//dx^2
           - 2*getFloat( DOG[idx], kp.row, kp.col ));
    cvmSet(H, 1, 1, getFloat( DOG[idx], kp.row+1, kp.col ) + getFloat( DOG[idx], kp.row-1, kp.col )	//dy^2
           - 2*getFloat( DOG[idx], kp.row, kp.col ));
    cvmSet(H, 2, 2, getFloat( DOG[idx+1], kp.row, kp.col ) + getFloat( DOG[idx-1], kp.row, kp.col )	//ds^2
           - 2*getFloat( DOG[idx], kp.row, kp.col ));
    cvmSet(H, 0, 1, 0.25*( getFloat( DOG[idx], kp.row+1, kp.col+1 ) + getFloat( DOG[idx], kp.row-1, kp.col-1 )	//dxdy
                           - getFloat( DOG[idx], kp.row-1, kp.col+1 ) - getFloat( DOG[idx], kp.row+1, kp.col-1 )));
    /*
    	cvmSet(H, 0, 2, 0.5*( getFloat( DOG[idx+1], kp.row, kp.col+1 ) + getFloat( DOG[idx-1], kp.row, kp.col-1 )	//dxds
    						 - getFloat( DOG[idx-1], kp.row, kp.col+1 ) - getFloat( DOG[idx+1], kp.row, kp.col-1 )));
    	cvmSet(H, 1, 2, 0.5*( getFloat( DOG[idx+1], kp.row+1, kp.col ) + getFloat( DOG[idx-1], kp.row-1, kp.col )	//dyds
    						 - getFloat( DOG[idx-1], kp.row+1, kp.col ) - getFloat( DOG[idx+1], kp.row-1, kp.col )));
    	cvmSet(H, 1, 0, cvmGet( H, 0, 1 ));	//dydx
    	cvmSet(H, 2, 0, cvmGet( H, 0, 2 ));	//dsdx
    	cvmSet(H, 2, 1, cvmGet( H, 1, 2 ));	//dsdy
    */
}

bool SIFT_Image::low_contrast( CvMat const *H, CvMat const *J, CvMat *X, Keypoint &kp ) //antes: Keypoint const &kp
{
    //	if( cvSolve( H, J, X, CV_LU ) == 0 )
    //	{
    //		cerr << "high_contrast(): cvSolve returned 0" << endl;
    //		return false;
    //	}
    // If Hessian does not have crossed derivatives, it is quicker to
    // solve the linear system this way...
    cvmSet(X, 0, 0, -cvmGet(J, 0, 0)/cvmGet(H, 0, 0) );
    cvmSet(X, 1, 0, -cvmGet(J, 1, 0)/cvmGet(H, 1, 1) );
    cvmSet(X, 2, 0, -cvmGet(J, 2, 0)/cvmGet(H, 2, 2) );

    // Checking contrast in fitted function
    float contrast_value = fabs(getFloat(DOG[(total_scales-1)*kp.octave+kp.scale],kp.row,kp.col) + 0.5*cvDotProduct( J, X ));
    if( contrast_value < param.m_contrast )
        return true;

    n_high_contrast_keypoints++;
    return false;
}

bool SIFT_Image::is_edge( CvMat const *H, CvMat const *J, Keypoint &kp ) //estava Keypoint const &kp, deve ser mais rapido
{
    float edge_ratio_value = ( SQUARE(cvmGet( H, 0, 0 )) + SQUARE(cvmGet( H, 1, 1 )) ) / ( cvmGet( H, 0, 0 )*cvmGet( H, 1, 1 ) - SQUARE(cvmGet( H, 0, 1 )));

    if( edge_ratio_value < (SQUARE(param.edge_ratio+1) / param.edge_ratio ))
        return false;
    else
        return true;
}

void SIFT_Image::orientation_assignment() {
    int initial_keys = keypoints.get_size();
    int hl = (param.orientation_window_size-1)/2;
    int i, j, k_i, idx;
    CvSize window_size = cvSize( param.orientation_window_size, param.orientation_window_size );
    CvRect roi;

    IplImage *filter[param.scales_per_octave];
    IplImage *weight = new_image( window_size );

    OrientationHistogram histogram(param.orientation_nbins);

    int max;
    float max_val;

    //Building weighting
    for(i = 0; i < param.scales_per_octave; i++) {
        filter[i] = new_image( window_size );

        cvSetZero( filter[i] );
        setFloat( filter[i], 1.0, hl, hl );
        cvSmooth(filter[i], filter[i], CV_GAUSSIAN, 0, 0, param.orientation_sigma_factor*param.initial_sigma*pow(k,i+1));
    }

    for( i = 0, k_i = keypoints.go_to_first(); i < initial_keys; i++, k_i = keypoints.go_to_next() ) {
        Keypoint &kp = keypoints[k_i];
        roi = cvRect(kp.col-hl, kp.row-hl, window_size.width, window_size.height );
        idx = param.scales_per_octave*kp.octave + kp.scale-1;  //Scale is expressed in gaussians...

        cvSetImageROI( gradient_mag[idx], roi );
        cvSetImageROI( gradient_theta[idx], roi );

        //Gaussian weight
        cvMul( filter[kp.scale-1], gradient_mag[idx], weight );

        //Filling the histogram
        histogram.fill( gradient_theta[idx], weight );

        //Get histogram max
        max_val = -1.0;
        for(j = 0; j < param.orientation_nbins; j++)
            if( histogram[j] > max_val ) {
                max_val = histogram[j];
                max = j;
            }
        set_keypoint_th(kp, fit_parabola( histogram, (max == 0) ? param.orientation_nbins-1 : max-1,
                                          max, (max == param.orientation_nbins-1) ? 0 : max+1));


        // Find other reasonable high local maxima,
        // and add them to database
        j = 0;
        if( histogram[j] >= max_val*param.multiple_peaks_factor	&& j != max &&
                histogram[j] > histogram[param.orientation_nbins-1] && histogram[j] >= histogram[j+1] ) {
            set_keypoint_th( keypoints[add_to_database(kp)], fit_parabola( histogram, param.orientation_nbins-1,j,j+1) );
            j++;
        }
        for( j++; j < param.orientation_nbins-1; j++ )
            if( histogram[j] >= max_val*param.multiple_peaks_factor	&& j != max &&
                    histogram[j] > histogram[j-1] && histogram[j] >= histogram[j+1] ) {
                set_keypoint_th( keypoints[add_to_database(kp)], fit_parabola( histogram, j-1,j,j+1) );
                j++;
            }
        if( j < param.orientation_nbins && histogram[j] >= max_val*param.multiple_peaks_factor && j != max &&
                histogram[j] > histogram[j-1] && histogram[j] >= histogram[0] )
            set_keypoint_th( keypoints[add_to_database(kp)], fit_parabola( histogram, j-1,j,0) );
    }

    n_total_keypoints  = keypoints.get_size();


    cvReleaseImage( &weight );
    for(i = 0; i < param.scales_per_octave; i++)
        cvReleaseImage( filter + i );


    for( i = 0; i < number_of_octaves*param.scales_per_octave; i++ ) {
        cvResetImageROI( gradient_mag[i] );
        cvResetImageROI( gradient_theta[i] );
    }
}


float SIFT_Image::fit_parabola( OrientationHistogram const &histogram, int left, int max, int right ) {
    float a;
    if( (a = histogram[left] + histogram[right] - 2*histogram[max]) == 0.0 )
        return histogram.bin_center(max);

    return histogram.bin_center(max) + histogram.bin_size() * (histogram[left] - histogram[right])/(2*a);
}



void SIFT_Image::keypoint_descriptor() {
    int idx,pos,j,k;
    float sum;
    CvRect roi;
    int wl = param.descriptor_bin_size*param.descriptor_n_bins;

    float cos_th, sin_th;
    float M[2][3] = {1.0, 0.0, wl/2, 0.0, 1.0, wl/2};
    CvMat mat = cvMat( 2, 3, CV_32FC1, M);

    IplImage *descriptor_mag = new_image( cvSize( wl, wl) );
    IplImage *descriptor_theta = new_image( cvSize( wl, wl) );
    IplImage *gaussian_filter = build_gaussian_weight( wl, &mat );
    IplImage *gaussian_weight = new_image( cvSize( wl, wl) );
    IplImage *mask = cvCreateImage( cvSize( wl, wl), IPL_DEPTH_8U, 1 );

    IplImage *interpolation_window = build_interpolation_window();
    int interpolation_size = interpolation_window->width;
    IplImage *interpolation_weight = new_image( cvSize( interpolation_size, interpolation_size ) );

    OrientationHistogram histogram( param.descriptor_n_orientations, param.descriptor_lin_interpolation );
    int descriptor_size = Keypoint::get_descriptor_size();
    float descriptor[descriptor_size];

    for( int k_i = keypoints.go_to_first(); k_i != -1; k_i = keypoints.go_to_next() ) {
        idx = param.scales_per_octave*keypoints[k_i].octave + keypoints[k_i].scale-1;  //Scale is expressed in gaussians...

        cvResetImageROI( gaussian_weight );
        cvResetImageROI( descriptor_theta );

        // Invariance to Orientation
        cos_th = cos( keypoints[k_i].th * M_PI/180 );
        sin_th = sin( keypoints[k_i].th * M_PI/180 );

        M[0][0] =  cos_th;
        M[0][1] = -sin_th;
        M[1][0] =  sin_th;
        M[1][1] =  cos_th;
        M[0][2] = keypoints[k_i].col;
        M[1][2] = keypoints[k_i].row;

        cvGetQuadrangleSubPix( gradient_mag[idx] , descriptor_mag, &mat );
        cvGetQuadrangleSubPix( gradient_theta[idx] , descriptor_theta, &mat );

        //Normalize direction
        cvSubS( descriptor_theta, cvRealScalar(keypoints[k_i].th), descriptor_theta );
        cvCmpS( descriptor_theta, 0.0, mask, CV_CMP_LT  );
        cvAddS( descriptor_theta, cvRealScalar(360.0), descriptor_theta, mask );

        //Gaussian weight
        cvMul( descriptor_mag, gaussian_filter, gaussian_weight );

        pos = 0;
        for( j = 0; j < param.descriptor_n_bins; j++ )
            for( k = 0; k < param.descriptor_n_bins; k++ ) {
                roi.x = k*param.descriptor_bin_size - (interpolation_size-param.descriptor_bin_size)/2;
                roi.y = j*param.descriptor_bin_size - (interpolation_size-param.descriptor_bin_size)/2;
                roi.width = roi.height = interpolation_size;
                cvSetImageROI( gaussian_weight, roi );
                cvSetImageROI( descriptor_theta, roi );

                roi.x = -roi.x;
                roi.y = -roi.y;
                roi.width = roi.height = wl;
                cvSetImageROI( interpolation_window, roi );
                cvSetImageROI( interpolation_weight, roi );
                cvMul( gaussian_weight, interpolation_window, interpolation_weight );

                //Filling the histogram
                histogram.fill( descriptor_theta, interpolation_weight );

                //Updating the keypoint descriptor
                memcpy( descriptor+pos, histogram.get_data(),
                        param.descriptor_n_orientations*sizeof(float) );
                pos += param.descriptor_n_orientations;
            }

        //Normalization & Threshold
        sum = 0.0;
        for( j = 0; j < descriptor_size; j++ )
            sum += SQUARE(descriptor[j]);
        sum = sqrt(sum);
        for( j = 0; j < descriptor_size; j++ )
            if( (descriptor[j] /= sum) > param.descriptor_mag_threshold )
                descriptor[j] = param.descriptor_mag_threshold;

        //ReNormalization
        sum = 0.0;
        for( j = 0; j < descriptor_size; j++ )
            sum += SQUARE(descriptor[j]);
        sum = sqrt(sum);
        for( j = 0; j < descriptor_size; j++ )
            descriptor[j] /= sum;

        keypoints[k_i].set_descriptor( descriptor );
        //keypoints[k_i].data_dump();
        //getchar();
        //cout << keypoints[k_i].sum_desc() << "\t";
    }
    //cout << endl << "******" << endl;
    cvReleaseImage( &descriptor_mag );
    cvReleaseImage( &descriptor_theta );
    cvReleaseImage( &gaussian_filter );
    cvReleaseImage( &gaussian_weight );
    cvReleaseImage( &mask );
    cvReleaseImage( &interpolation_window );
    cvReleaseImage( &interpolation_weight );
}


IplImage *SIFT_Image::build_gaussian_weight( int wl, CvMat const *mat ) {
    IplImage *gaussian_filter = new_image( cvSize( wl, wl) );

    if( wl % 2 == 0 ) {
        IplImage *filter_tmp = new_image( cvSize( wl+1, wl+1) );
        cvSetZero( filter_tmp );
        setFloat( filter_tmp, 1.0, wl/2, wl/2 );
        cvSmooth( filter_tmp, filter_tmp, CV_GAUSSIAN, 0, 0, wl*param.descriptor_sigma_factor);
        cvGetQuadrangleSubPix( filter_tmp , gaussian_filter, mat );
        cvReleaseImage( &filter_tmp );
    } else {
        cvSetZero( gaussian_filter );
        setFloat( gaussian_filter, 1.0, (wl-1)/2, (wl-1)/2 );
        cvSmooth( gaussian_filter, gaussian_filter, CV_GAUSSIAN, 0, 0, wl*param.descriptor_sigma_factor);
    }

    return gaussian_filter;
}

IplImage *SIFT_Image::build_interpolation_window() {
    if( !param.descriptor_lin_interpolation ) {
        int interpolation_size = param.descriptor_bin_size;
        IplImage *window = new_image( cvSize( interpolation_size, interpolation_size ) );
        cvSet( window, cvRealScalar(1.0) );
        return window;
    }

    float val;
    int interpolation_size = (param.descriptor_bin_size % 2 == 0) ?
                             2*param.descriptor_bin_size : 2*param.descriptor_bin_size-1;
    float c = (interpolation_size-1)/2;

    IplImage *window = new_image( cvSize( interpolation_size, interpolation_size ) );

    for(int i = 0; i < interpolation_size; i++)
        for(int j = 0; j < interpolation_size; j++) {
            val = 1 - hypot(i-c,j-c)/param.descriptor_bin_size;
            setFloat( window, (val < 0) ? 0 : val, i, j );
        }

    return window;
}


void SIFT_Image::show_contents(IplImage const * img) {
    CvRect roi = cvGetImageROI(img);
    for(int i = 0; i < roi.height; i++)
        for(int j = 0; j < roi.width; j++)
            cout << "Image[" << i << "][" << j << "] = " <<cvGetReal2D( img, i, j ) << endl;
}


void SIFT_Image::display_image(IplImage const *img) {
    cvNamedWindow( "Sample" );
    IplImage *resized = new_image(cvSize(256,256));
    cvResize( img, resized, CV_INTER_NN );
    show_and_wait_key("Sample", resized);
    cvDestroyWindow( "Sample" );

    cvReleaseImage(&resized);
}


void SIFT_Image::display_image(Keypoint const &kp) {
    IplImage *descriptor = new_image(cvSize(3,3));

    float AA[2][3];
    AA[0][0] =  1.0;//cos( kp.orientation * M_PI/180 );
    AA[0][1] = 0.0;//-sin( kp.orientation * M_PI/180 );
    AA[1][0] =  0.0;//sin( kp.orientation * M_PI/180 );
    AA[1][1] =  1.0;//cos( kp.orientation * M_PI/180 );
    AA[0][2] = kp.col;
    AA[1][2] = kp.row;
    CvMat mat = cvMat( 2, 3, CV_32FC1, AA);

    cvGetQuadrangleSubPix( gaussians[IDX(kp.octave,kp.scale)] , descriptor, &mat );
    show_contents(descriptor);
    display_image( descriptor );
    cvGetQuadrangleSubPix( DOG[(total_scales-1)*kp.octave+kp.scale] , descriptor, &mat );
    show_contents(descriptor);
    display_image( descriptor );

    cvReleaseImage(&descriptor);
}


void SIFT_Image::test3( IplImage const *img ) {
    for(int tt = 0; tt < 4; tt++) {
        for(int qq = 0; qq < 4; qq++)
            cout << cvGetReal2D( img, tt, qq ) << " ";
        cout << endl;
    }
    getchar();
}

void SIFT_Image::test2( Keypoint const &kp) {
    for( int j = 0; j < Keypoint::get_descriptor_size(); j++ ) {
        if( j % param.descriptor_n_orientations == 0 )
            cout << "Bin " << j/param.descriptor_n_orientations << endl;
        cout << j << "-> " << kp.get_descriptor()[j] << endl;
    }
    cout << endl;

    getchar();
}



void SIFT_Image::test1( Keypoint const &kp) {

    int idx = (total_scales-1)*kp.octave + kp.scale;

    cout  << "****************************************" << endl << "Keypoint: \n" <<
    getFloat(DOG[idx-1], kp.row-1, kp.col-1) << " " << getFloat(DOG[idx-1], kp.row-1, kp.col) <<  " " <<
    getFloat(DOG[idx-1], kp.row-1, kp.col+1) << "   \t" <<
    getFloat(DOG[idx], kp.row-1, kp.col-1) << " " <<  getFloat(DOG[idx], kp.row-1, kp.col) << " " <<
    getFloat(DOG[idx], kp.row-1, kp.col+1) << endl <<

    getFloat(DOG[idx-1], kp.row, kp.col-1) << " " << getFloat(DOG[idx-1], kp.row, kp.col) << " " <<
    getFloat(DOG[idx-1], kp.row, kp.col+1) << "   \t" <<
    getFloat(DOG[idx], kp.row, kp.col-1) << " " <<  getFloat(DOG[idx], kp.row, kp.col) << " " <<
    getFloat(DOG[idx], kp.row, kp.col+1) << endl <<

    getFloat(DOG[idx-1], kp.row+1, kp.col-1) << " " << getFloat(DOG[idx-1], kp.row+1, kp.col) << " " <<
    getFloat(DOG[idx-1], kp.row+1, kp.col+1) << "   \t" <<
    getFloat(DOG[idx], kp.row+1, kp.col-1) << " " <<  getFloat(DOG[idx], kp.row+1, kp.col) << " " <<
    getFloat(DOG[idx], kp.row+1, kp.col+1) << endl << endl <<

    getFloat(DOG[idx+1], kp.row-1, kp.col-1) << " " << getFloat(DOG[idx+1], kp.row-1, kp.col) << " " <<
    getFloat(DOG[idx+1], kp.row-1, kp.col+1) << endl <<
    getFloat(DOG[idx+1], kp.row, kp.col-1) << " " <<  getFloat(DOG[idx+1], kp.row, kp.col) << " " <<
    getFloat(DOG[idx+1], kp.row, kp.col+1) << endl <<
    getFloat(DOG[idx+1], kp.row+1, kp.col-1) << " " << getFloat(DOG[idx+1], kp.row+1, kp.col) << " " <<
    getFloat(DOG[idx+1], kp.row+1, kp.col+1) << endl;

    /*
    	cout << endl << "Hessian: \t\t\t\t\tJacobian:" << endl << 
    	cvmGet( H, 0, 0 ) << " " <<  cvmGet( H, 0, 1 ) << " " << cvmGet( H, 0, 2 )	<< "\t\t" << -cvmGet( J, 0, 0 ) << endl <<
    	cvmGet( H, 1, 0 ) << " " <<  cvmGet( H, 1, 1 ) << " " << cvmGet( H, 1, 2 )	<< "\t\t" << -cvmGet( J, 1, 0 ) << endl <<
    	cvmGet( H, 2, 0 ) << " " <<  cvmGet( H, 2, 1 ) << " " << cvmGet( H, 2, 2 )	<< "\t\t" << -cvmGet( J, 2, 0 ) << endl;
     
    	if( cvSolve( H, J, X, CV_LU ) == 0 )
    		cout << "!!!!!!!!!!!!!!!!" << endl;
     
    	cout << endl << "X = [" <<
    	cvmGet( X, 0, 0 ) << " " <<  cvmGet( X, 1, 0 ) << " " << cvmGet( X, 2, 0 )	<< "]\t\tFit: " <<
    	getFloat(DOG[idx],kp.row,kp.col) - 0.5*cvDotProduct( J, X ) << endl<< endl;
    */
    getchar();
}

void SIFT_Image::draw_features(int draw_type){
    struct feature* feat, *aux;
    feat = this->features;
    for(int i = 0; i < number_of_features; i++ ) {
        aux = feat + i;
        switch(draw_type){
            case RECTANGLE:
                cvRectangle(image_color, cvPoint(cvRound(aux->x)-1, cvRound(aux->y)-1), cvPoint(cvRound(aux->x)+1, cvRound(aux->y)+1), CV_RGB( 255, 255, 0 ));
                break;
            case CROSS:
                cvLine(image_color, cvPoint(cvRound(aux->x)-2, cvRound(aux->y)-2), cvPoint(cvRound(aux->x)+2, cvRound(aux->y)+2), CV_RGB( 0, 255, 255 ));
                cvLine(image_color, cvPoint(cvRound(aux->x)-2, cvRound(aux->y)+2), cvPoint(cvRound(aux->x)+2, cvRound(aux->y)-2), CV_RGB( 0, 255, 255 ));
                break;
            case POINT:
                cvRectangle(image_color, cvPoint(cvRound(aux->x), cvRound(aux->y)), cvPoint(cvRound(aux->x), cvRound(aux->y)), CV_RGB( 255, 0, 255 ));
                break;
        }
    }
}

void SIFT_Image::draw_keypoints(int draw_type){
    int k_i;
    for(int i = 0, k_i = keypoints.go_to_first(); i < keypoints.get_size(); i++, k_i = keypoints.go_to_next() ) {
        switch(draw_type){
            case RECTANGLE:
                cvRectangle(image_color, cvPoint(cvRound(keypoints[k_i].x)-1, cvRound(keypoints[k_i].y)-1), cvPoint(cvRound(keypoints[k_i].x)+1, cvRound(keypoints[k_i].y)+1), CV_RGB( 255, 255, 0 ));
                break;
            case CROSS:
                cvLine(image_color, cvPoint(cvRound(keypoints[k_i].x)-2, cvRound(keypoints[k_i].y)-2), cvPoint(cvRound(keypoints[k_i].x)+2, cvRound(keypoints[k_i].y)+2), CV_RGB( 0, 255, 255 ));
                cvLine(image_color, cvPoint(cvRound(keypoints[k_i].x)-2, cvRound(keypoints[k_i].y)+2), cvPoint(cvRound(keypoints[k_i].x)+2, cvRound(keypoints[k_i].y)-2), CV_RGB( 0, 255, 255 ));
                break;
            case POINT:
                cvRectangle(image_color, cvPoint(cvRound(keypoints[k_i].x), cvRound(keypoints[k_i].y)), cvPoint(cvRound(keypoints[k_i].x), cvRound(keypoints[k_i].y)), CV_RGB( 255, 0, 255 ));
                break;
        }
    }
}

// void cvLine( CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color,
//              int thickness=1, int line_type=8, int shift=0 );

// void cvRectangle( CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color,
//                   int thickness=1, int line_type=8, int shift=0 );
// 
//  inline CvPoint cvPoint( int x, int y );
// 
//     CvScalar color = CV_RGB( 255, 0, 0 );
