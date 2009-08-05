// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 *
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


#include "SIFT_disparity.h"
#include "siftRH.h"
#include "common_functions.h"
#ifndef ABS
#define ABS(x) ( ( x < 0 )? -x : x )
#endif

SIFT_disparity::SIFT_disparity() {
    there_is_a_close_object = false;
    average_disparity_of_object = 0;
    total_matches = 0;
    outliers = 0;
}

SIFT_disparity::~SIFT_disparity() {
}



void SIFT_disparity::insert_database_disparity_match( Disparity_Match const &m) {
    if( d_matches.insert_record( m ) == -1 ) {
        cout <<"Resizing disparity_matches database..."<<endl;
        d_matches.resize( 2*d_matches.get_capacity() );
        if( d_matches.insert_record( m ) == -1 ) {
            cerr << "Disparity Matching::insert_database_match: could not allocate memory. Quiting " << endl;
            exit(1);
        }
    }
}

void SIFT_disparity::go(IplImage const *limg, IplImage const *rimg) {

    left.load(limg);
    right.load(rimg);
    this->go();
}

void SIFT_disparity::go(SIFT_Image sift_limg, SIFT_Image sift_rimg) {

    left = sift_limg;
    right = sift_rimg;
    this->go();
}


void SIFT_disparity::go() {
    double t_start = (double)cvGetTickCount();

    d_matches.reset(1000);

    Database<Keypoint> &left_image_kps = left.get_keypoints();
    int n = left.number_of_features;
    struct feature* left_image_feats_root = left.features;
    struct feature* actual_feat;
    struct kd_node* right_kd_root;
    struct feature** nbrs;

    Disparity_Match nearest;
    double d_feat= DBL_MAX, nearest_dist, nearest2nd_dist;

    HorLim = left.image->width / 6.4;
    VertLim = left.image->height / 4.; ///WAS /16 , but to accomodate the "vesgo" chico...
    average_disparity_of_object = 0;
    
    //     cout<<"x>HORIZONTAL LIM = " <<Hor

    MaxHeight = -1;
    MinHeight = INT_MAX;
    MaxWidth = -1;
    MinWidth = INT_MAX;
    for(int i = 0; i < n; i++ ) { //for all features in image_to_match
        actual_feat = left_image_feats_root + i;
        nearest.left_keypoint = actual_feat->keypoint_index;

        nearest_dist = DBL_MAX;
        nearest2nd_dist = DBL_MAX;

        if (actual_feat == NULL) {
            cout << "image_feat "<< i <<" == NULL"<< endl;
            return;
        } else {
            right_kd_root = right.kd_image_root;
            if(right_kd_root == NULL) {
                cout << "kd_root of right image == NULL"<< endl;
                return;
            }
            int k = kdtree_bbf_knn( right_kd_root, actual_feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
            if(k == -1) {
                cout << "kdtree_bbf_knn() failed somehow, did not find 2 neighbors" << endl;
                return;
            } else if(k == 2 ) {
                //                 cout << "gm? ";
                /// Is it a good match?
                if( descr_dist_sq(actual_feat, nbrs[0]) < descr_dist_sq(actual_feat, nbrs[1]) * NN_SQ_DIST_RATIO_THR ) {
                    nearest.right_keypoint = nbrs[0]->keypoint_index;
                    nearest.disparity[0] = actual_feat->x - nbrs[0]->x;
                    nearest.disparity[1] = actual_feat->y - nbrs[0]->y;
                    //                     cout << "D={"<<nearest.disparity[0]<<","<<nearest.disparity[1]<<"} ";
                    if(ABS(nearest.disparity[0]) > HorLim) { ///It's close "enough"
                        if(ABS(nearest.disparity[1]) < VertLim) { ///Also need to test for outliers of horizontal disparity of diferent sign than the average
                            nearest.left_feat = i;
                            insert_database_disparity_match( nearest );
                            average_disparity_of_object += nearest.disparity[0];
                            //fprintf(fpFile, "%g %g\n", nearest.disparity[0], nearest.disparity[1]);//DEBUG
                            /// Determine cropping parameters
                            if(actual_feat->x > MaxWidth)
                                MaxWidth = (int) actual_feat->x;
                            if(actual_feat->x < MinWidth)
                                MinWidth = (int) actual_feat->x;
                            if(actual_feat->y > MaxHeight)
                                MaxHeight = (int) actual_feat->y;
                            if(actual_feat->y < MinHeight)
                                MinHeight = (int) actual_feat->y;
                            //cout << "Disparity: {"<<nearest.disparity[0]<<","<<nearest.disparity[1]<<"} ";
                            //cout << "Width:{"<<MinWidth<<","<<MaxWidth<<"} Height:{"<<MinHeight<<","<<MaxHeight<<"}"<<endl;
                        }
                    }
                    total_matches++;
                    ///for the present images, that have negative horizontal disparity, this is an outlier BUT SHOULD BE "SIGN DIFFERENT THAN THE AVERAGE SIGN"
                    if(nearest.disparity[0] > 0 || ABS(nearest.disparity[1]) > VertLim) { 
                        outliers++;
//                         cout << "D={"<<nearest.disparity[0]<<","<<nearest.disparity[1]<<"} ";
                    }

                }
            }
            free(nbrs);
        }
    }
    ///Increasing the cropping parameters to include the border in which SIFT features are ignored
    if(MaxHeight != -1 && MinHeight != INT_MAX && MaxWidth != -1 && MinWidth != INT_MAX) {
        MaxWidth = (MaxWidth+(SIFT_IMG_BORDER+1) > left.image->width)?  left.image->width : MaxWidth+(SIFT_IMG_BORDER+1);
        MinWidth = (MinWidth-(SIFT_IMG_BORDER+1) < 0)?  0 : MinWidth-SIFT_IMG_BORDER;
        MaxHeight = (MaxHeight+(SIFT_IMG_BORDER+1) > left.image->height)?  left.image->height : MaxHeight+(SIFT_IMG_BORDER+1);
        MinHeight = (MinHeight-(SIFT_IMG_BORDER+1) < 0)?  0 : MinHeight-(SIFT_IMG_BORDER+1);
    }

    ///To save a new object how many features do we need? Well, it only needs three for recognition, but we don't want to be affected by bad matches, so, say 10 is a good number
    if(d_matches.get_size() >= 10){ 
        create_close_object_SIFT_Image();
        there_is_a_close_object = true;
        average_disparity_of_object = ABS(average_disparity_of_object/d_matches.get_size());
        //cout << "Average closeness: "<< average_disparity_of_object<<" [Horizontal Limit:"<< HorLim<<"] ";
    }else{
        there_is_a_close_object = false;
    }


//     cout <<"total matches = "<<total_matches<<"; outliers = "<<outliers<<";"<<endl;
    double t_end = (double)cvGetTickCount() - t_start;
    cout << "Disparity:: Found "<<d_matches.get_size()<<" good close object matches in "<< t_end/(cvGetTickFrequency()*1000.)<<"ms. Average depth: "<<average_disparity_of_object<<" [Hor Limit:"<< HorLim<<" Vert Lim:"<< VertLim<< "] "<<endl;
}

bool SIFT_disparity::go( string const &left , string const &right) {
    IplImage * LeftImage;
    IplImage * RightImage;

    if( (LeftImage = cvLoadImage( left.c_str() )) == 0 ) {
        cerr << "Could not load image file: " << left << endl;
        return false;
    }
    if( (RightImage = cvLoadImage( right.c_str() )) == 0 ) {
        cerr << "Could not load image file: " << right << endl;
        return false;
    }

    //     string aux = left;
    //     aux.append("_disparty.txt");//DEBUG
    //     fpFile = fopen (aux.c_str(),"w"); //DEBUG

    this->go(LeftImage, RightImage);

    cvReleaseImage( &LeftImage );
    cvReleaseImage( &RightImage );

    return true;
}


void SIFT_disparity::create_close_object_SIFT_Image(){
    //SIFT_Image empty;
    close_obj.restart(); ///clean the SIFT_Image close_obj before starting to tinker with it's pointers
    
    close_obj.number_of_features = d_matches.get_size();
    close_obj.features = (feature *) calloc( close_obj.number_of_features, sizeof(struct feature) );
    if(close_obj.features == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}
//     close_obj.keypoint_indexes = (int *) calloc( close_obj.number_of_features, sizeof(int) );
//     if(close_obj.keypoint_indexes == NULL) {cout<<"Unable to allocate memory"<<endl; exit(1);}

    int dur = 0;
    for(int index = d_matches.go_to_first(); dur < close_obj.number_of_features, index != -1; dur++, index = d_matches.go_to_next()) {
        clone_feature((close_obj.features + dur), (left.features + d_matches[index].left_feat));
//         close_obj.keypoint_indexes[dur] = left.keypoint_indexes[d_matches[index].left_feat];
//         cout<<"1:"<<(left.features + d_matches[index].left_feat)->keypoint_index;
//         cout<<"\t 2:" << left.keypoint_indexes[d_matches[index].left_feat] <<endl;
    }
    close_obj.kd_image_root = kdtree_build( close_obj.features, close_obj.number_of_features );
    // 	cout << "out SIFT_Image::copy(SIFT_Image const &other)"<< endl; //DEBUG

    IplImage* near_object = crop(left.image_color, MaxWidth, MinWidth, MaxHeight, MinHeight);
    close_obj.image_color = near_object;

    IplImage* image = cvCreateImage( cvGetSize(near_object) , close_obj.param.depth, 1 );
    if( near_object->nChannels > 1 ) {
        IplImage *tmp = cvCreateImage( cvGetSize(image), close_obj.param.depth, near_object->nChannels );
        cvConvertScale( near_object, tmp, 1.0/255 );
        cvCvtColor( tmp, image, CV_BGR2GRAY );
        cvReleaseImage( &tmp );
    }else
        cvConvertScale( near_object, image, 1.0/255 );
    close_obj.image = image;

    struct feature* feat, *aux;
    Keypoint kp;
    feat = close_obj.features;
    int index;
    for(int i = 0; i < close_obj.number_of_features; i++ ) {
        aux = feat + i;

        aux->x = aux->x - MinWidth;
        aux->y = aux->y - MinHeight;

        kp = feature2keypoint(aux);
        index = close_obj.add_to_database( kp );
        aux->keypoint_index = index;
//         close_obj.keypoint_indexes[i] = index;

        //db_index = get_current();
        // 		MESSAGE("Out feature2keypoint: kp.s = " << kp.s << "DB index: " << db_index); //DEBUG
    }

}

void SIFT_disparity::draw_and_show_d_matches(int draw_type){
    IplImage * left_i = 0;
    IplImage * right_i = 0;
    
    left_i = cvCloneImage(left.image_color);
    right_i = cvCloneImage(right.image_color);

    for(int index = d_matches.go_to_first(); index != -1; index = d_matches.go_to_next()) {
        int x = (int) left.keypoints[d_matches[index].left_keypoint].x;
        int y = (int) left.keypoints[d_matches[index].left_keypoint].y;
        switch(draw_type){
            case RECTANGLE:
                cvRectangle(left_i, cvPoint(x-1, y-1), cvPoint(x+1, y+1), CV_RGB( 255, 255, 0 ));
                break;
            case CROSS:
                cvLine(left_i, cvPoint(x-2, y-2), cvPoint(x+2, y+2), CV_RGB( 0, 255, 255 ));
                cvLine(left_i, cvPoint(x-2, y+2), cvPoint(x+2, y-2), CV_RGB( 0, 255, 255 ));
                break;
            case POINT:
                cvRectangle(left_i, cvPoint(x, y), cvPoint(x, y), CV_RGB( 255, 0, 255 ));
                break;
            case PLUS:
                cvLine(left_i, cvPoint(x-2, y), cvPoint(x+2, y), CV_RGB( 255, 0, 255 ));
                cvLine(left_i, cvPoint(x, y+2), cvPoint(x, y-2), CV_RGB( 255, 0, 255 ));
        }

        x = (int) right.keypoints[d_matches[index].right_keypoint].x;
        y = (int) right.keypoints[d_matches[index].right_keypoint].y;
        switch(draw_type){
            case RECTANGLE:
                cvRectangle(right_i, cvPoint(x-1, y-1), cvPoint(x+1, y+1), CV_RGB( 255, 255, 0 ));
                break;
            case CROSS:
                cvLine(right_i, cvPoint(x-2, y-2), cvPoint(x+2, y+2), CV_RGB( 0, 255, 255 ));
                cvLine(right_i, cvPoint(x-2, y+2), cvPoint(x+2, y-2), CV_RGB( 0, 255, 255 ));
                break;
            case POINT:
                cvRectangle(right_i, cvPoint(x, y), cvPoint(x, y), CV_RGB( 255, 0, 255 ));
                break;
            case PLUS:
                cvLine(right_i, cvPoint(x-2, y), cvPoint(x+2, y), CV_RGB( 255, 0, 255 ));
                cvLine(right_i, cvPoint(x, y+2), cvPoint(x, y-2), CV_RGB( 255, 0, 255 ));
        }
    }

    cvNamedWindow( "left_i" );
    cvNamedWindow( "right_i" );
    cvShowImage("left_i", left_i);
    cvShowImage("right_i", right_i);
    cvWaitKey(0);
    cvDestroyWindow( "left_i" );
    cvDestroyWindow( "right_i" );


    cvReleaseImage(&left_i);
    cvReleaseImage(&right_i);
}

