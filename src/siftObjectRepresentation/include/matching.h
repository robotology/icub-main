// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#ifndef __MATCHING_HPP__B
#define __MATCHING_HPP__B

//std libs
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

//Rob Hess SIFT libs
#include "kdtree.h"
#include "imgfeatures.h"
#include "siftRH.h"

//Bruno Damas libs
#include "sift.hpp"
#include "hashclasses.hpp"
using namespace FBB;

#include <string>
#include <vector>
#include <queue>
using namespace std;

//opencv libs
#include <cv.h>
#include <highgui.h>

//yarp libs
#include <yarp/os/Bottle.h>
using namespace yarp::os;

//iCub libs
#include <iCub/head/iCubHeadKinematics.h>
using namespace iCub::contrib;

#include "SIFT_disparity.h"
#include "common.hpp"

#define BDD_MATCH_MESSAGE
#ifdef BDD_MATCH_MESSAGE
#define MESSAGE( A ) ( cout << "Matching: " << A << endl )
#else
#define MESSAGE( A )
#endif

#define HASH_MAP_SIZE 2000

class Matching {
private:
    //feature database
    struct feature* all_features;
    int number_of_all_features;
    struct kd_node* kd_image_root_all;
    //input image added of the extracted features plus detected objects
    IplImage * matched_image;
    //Structure with all the parameters needed for this class
    struct Param {
        int max_images;
        int max_matches;
        int max_models;

        float bin_theta_size;
        float bin_scale_size;
        float bin_pos_size;
        float scale_probability;
        float false_match;
        float correct_match;
        float significance;

        float distance_ratio;

        int min_models_per_bin;
    };
    static Param param;
    static bool initialized;

    struct Model;
    //A possible object detected in the SIFT_Image image_to_match;
    struct Model {
        int id;

        float M[2][2];
        float T[2][1];

        int image;  //index in the 'keypoint' database of the possible object detected
        int matches;

        float p_value;
        float sensitivity;
        float sensitivity2;
        float sensitivity4;
        float sensitivity10;
    };
    //A 'keypoint' match between the image_to_match and the database
    struct Match {
        int training_image; //index of the image in the datbase to which the matched 'keypoint' belongs to
        int training_keypoint; //index of the 'keypoint' in the datbase to which this 'keypoint' was matched to
        int keypoint;       //index of the keypoint of image_to_match
        //pose of recognized feature in image_to_match
        float s;
        float th;
        float x;
        float y;
        
        int model;//index of Model this Match belongs to

        Match() : model(-1) {}
    };

    ///this is the reason this code does not compile in Visual Studio, this asks for ext/hash_map, which is not GNU standard, and Visual Studio uses his own libs for hashes. This compiles in Dev C++ althogh..
    class HoughBin {
        public:
            int count;
            StringMultiHash<Match *>::iterator first;
            StringMultiHash<Match *>::iterator last;

            bool operator<(HoughBin const &other) const {
                return count < other.count;
            }
    };
    StringMultiHash<Match *> hough_table;
    priority_queue<HoughBin> hough_bins;
    //image that may contain objects already stored in the database
    SIFT_Image image_to_match; 
    //functions called by the constructors/destructors
    void start();
    void copy(Matching const &other);
    void destroy();
    void restart();

    void insert_database_match( Match const &m);
    void insert_model( Model const &model );
    //functions that do most of the work, matching and validating that objects were really detected in the image
    void match_to_database();
    void get_matches();
    void get_matches_BrunoDamas();
    void cluster_matches();
    void order_models();
    void refine_models();
    void validate_models();
    void remove_models();
    double cumulative_binomial( int n, double p, int k );
    int critical_region( int n, double p, double significance );
    void fill_matrices( int i, Match const &match, float matA[][6], float matB[][1] );
    bool agree( Match const &match, float const affine_tr[][1]  );
    int image_range_features( SIFT_Image &img, Model const &model );
    
    //rebuilds the feature data tree due to adding/removal of new image
    void rebuild_kd_tree(SIFT_Image image, int image_index);

public:
    ///--this should be private, but SIFT_disparity requires access to it--
    Database<SIFT_Image> training_samples;
    Database<Match> matches;
    Database<Model> models;
    Database<Model> &get_models();
    Database<SIFT_Image> &get_database_images();
    ///-- --
    
    //contructors and destructor
    Matching();
    Matching(Matching const &other);
    Matching &operator=(Matching const &other);
    ~Matching();
    //store new image do database
    bool add_training_image(SIFT_Image const &img);
    bool add_training_image(IplImage *img);
    bool add_training_image(IplImage *img, string label);
    bool add_training_image(string const &filename);
    bool add_training_image(string const &filename, string label);
    //load an image and look for known objects in it
    void match_to_database(SIFT_Image const &img);
    void match_to_database(IplImage *img);
    void match_to_database(string const &filename);
    //draw bounding boxes of detected objects in the image and return image.
    IplImage * update_matched_image();
    IplImage * get_matched_image();
    IplImage * get_original_image();
                
    //save/load to/from ./database/database.txt ./database/x.png ./database/x.txt
    void save_database();
    void load_database();
    //calls add_training_image to the images specified in filename
    void load_images(string const &filename);

    //display functions (in opencv windows)
    void display_matching();
    void display_test_image();
    void display_database_images();
    //load parameters from filename, or if filename=="" loads default parameters
    static bool init( string const &filename = "" );
    //debug function
    static void test_class();
    //draws in the image_to_match the matched features to the database (possible types: RECTANGLE, CROSS, POINT, PLUS)
    void draw_match_features(int draw_type);
    //makes an yarp bottle with the position and lables of detected objects in the image
    int make_bottle(yarp::os::Bottle * bawtle, double* encoders, double fx, double fy);
    //adds an object to the database and returns its index in the dataase
    int insert_database_image( SIFT_Image const &image);
    
};

#endif
