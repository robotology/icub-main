// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *          2007 Bruno Damas
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#ifndef __SIFT_HPP__B
#define __SIFT_HPP__B

#include "database.hpp"
#include "keypoint.hpp"
#include "histogram.hpp"

#include <string>
using namespace std;

#include <cv.h>

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200
/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

#define PLUS 3
#define POINT 2
#define CROSS 1
#define RECTANGLE 0
class SIFT_Image {
public:
    SIFT_Image( SIFT_Image const &other);
    SIFT_Image const &operator=(SIFT_Image const &other);
    SIFT_Image();
    SIFT_Image( string const &filename );
    SIFT_Image( IplImage const *img );
    ~SIFT_Image();
    void restart();

    bool load( string const &filename );
    bool load( IplImage const *img );
    //bool load( IplImage *img ); //added

    //bool SIFT_Image::load( string const &filename , int trim); //So not to change the originals, we trim only here
    //bool SIFT_Image::load( IplImage const *img , int trim); //So not to change the originals, we trim only here

    operator IplImage const &();

    IplImage const *get_image() const;
    IplImage const *get_image_color() const;
    void draw_features(int draw_type);
    void draw_keypoints(int draw_type);

    Database<Keypoint> &get_keypoints();

    void show( string const &window_name );
    void show_gaussians( string const &window_name );
    void show_diff_of_gaussians( string const &window_name );
    void show_gradient( string const &window_name );

    static bool init( string const &filename = "" );
    static void test_class();

    //void trim_keypoints();
    int add_to_database(Keypoint const &k);
    void add_image(IplImage const *img);

    Database<Keypoint> keypoints;
    struct feature* features;
    int number_of_features;
    struct kd_node* kd_image_root;
//     int * keypoint_indexes;

    string label;
    double frame_time;
    int sec; //seconds of the frame_time
    int usec;//microseconds of the frame_time
    //private: // PUZ Param param PUBLIC pra poder mecher em Matcing
    struct Param {
        int depth;

        int scales_per_octave;
        double prior_sigma;
        double initial_sigma;

        float m_contrast;
        float edge_ratio;

        int orientation_window_size;
        double orientation_sigma_factor;
        int orientation_nbins;
        float multiple_peaks_factor;

        int descriptor_bin_size;
        int descriptor_n_bins;
        int descriptor_n_orientations;
        bool descriptor_lin_interpolation;
        float descriptor_sigma_factor;
        float descriptor_mag_threshold;

        bool delete_images_after_processing;

        // 			int max_number_per_image;

        int max_keypoints;

    };

    static Param param;
    IplImage *image;
    IplImage *image_color;

private:
    static bool initialized;

    IplImage **gaussians;
    IplImage **DOG;
    IplImage **gradient_mag;
    IplImage **gradient_theta;

    // 		Database<Keypoint> keypoints; //PUZ PUBLIC pra poder mecher em Matching

    int n_keypoints;
    int n_high_contrast_keypoints;
    int n_stable_keypoints;
    int n_total_keypoints;

    // 		int max_number_per_image; //Trimming number

    int number_of_octaves;
    int total_scales;
    double sigma;
    double k;

    void copy(SIFT_Image const &other);
    void destroy();
    void start();

    IplImage *new_image( CvSize s );
    void create_aux_images();
    void delete_aux_images();
    void show_and_wait_key( string const &window_name, IplImage *img );
    //int add_to_database(Keypoint const &k); //PASSEI ISTO PRA PUBLIC
    void set_keypoint_th(Keypoint &k, float theta );
    void adjust_keypoint(Keypoint &k, float dx = 0.0, float dy = 0.0, float ds = 0.0 );

    void extract_features();
    // 		void extract_features(int trim); //So not to change the originals, we trim only here

    void build_gaussians();
    void build_next_gaussian(int octave, int scale);
    void build_DOG();				//Diference of Gaussian
    void build_gradients();
    void find_keypoints();
    bool iskeypoint(Keypoint const &kp);
    void remove_unstable_keypoints();
    void build_Hessian_and_Jacobian( CvMat *H, CvMat *J, Keypoint const &kp );
    bool low_contrast( CvMat const *H, CvMat const *J, CvMat *X, Keypoint &kp );//antes: Keypoint const &kp
    bool low_contrast( CvMat const *H, CvMat const *J, CvMat *X, Keypoint const &kp );
    bool is_edge( CvMat const *H, CvMat const *J, Keypoint &kp ); //antes: Keypoint const &kp mais rapido
    bool is_edge( CvMat const *H, CvMat const *J, Keypoint const &kp );
    void orientation_assignment();
    float fit_parabola( OrientationHistogram const &histogram, int left, int max, int right );
    void keypoint_descriptor();
    IplImage *build_gaussian_weight( int wl, CvMat const *mat );
    IplImage *build_interpolation_window();

    void show_contents(IplImage const * img);
    void display_image(IplImage const * img);
    void display_image(Keypoint const &kp);
    void test1(Keypoint const &kp);
    void test2(Keypoint const &kp);
    void test3( IplImage const *img );

    void print_all_keypoints(void);
};

#endif

