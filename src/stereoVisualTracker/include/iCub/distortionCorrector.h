#ifndef __DISTORTION_CORRECTOR_H__
#define __DISTORTION_CORRECTOR_H__

#include "public.h"
#ifndef NEW_VERSION
#include "frameGrabber.h"
#endif

/**
 * @brief corrects for the lens distortion
 */
class DistortionCorrector :  public Filter
{
 protected:

  //  IplImage *distortionMap;

  IplImage *tmp_image;
  IplImage *distMap;
  CvCalibFilter *calib;
  double chessboard[3];
  char paramfile[80];
  FrameGrabber *grabber;
  int undistortion_ready;


 public:
  /**
   * constructor
   */ 
  DistortionCorrector();

  /**
   * constructor
   * @param filename file where to load/save the distortion parameters 
   */ 
  DistortionCorrector(char *filename);
  
  /**
   * Destructor
   */
  virtual ~DistortionCorrector();
  
  /**
   * @brief specifies where to grab frames from
   * @param grab a pointer to that framegrabber
   */
  void SetGrabber(FrameGrabber *grab);
  /**
   * @brief interactive camera calibration (using a chessboard)
   * @param nb_frame the number of views of the chessboard used for calibration
   */
  int CalibrateCamera(int nb_frames);
 
  /**
   * @brief indistorts the image
   * @param image a pointer to the image to be undistorted, The resulting
   * (undistorted) image replaces the distorted one.
   */
  virtual void Apply(IplImage *image);  
 
  /**
   * @return 1 if the camera is calibrated, 0 if it is not.
   */
  int IsCalibrated();
 
  /**
   * @brief saves the camera parameters
   * @param filename the file where to save the paramters
   * @return 1 upon success, 0 upon failure
   */
  int SaveCameraParams( const char* filename );
 
  /**
   * @brief loads the camera parameters
   * @param filename the file where to read the paramters
   * @return 1 upon success, 0 upon failure
   */
  int LoadCameraParams( const char* filename );

  /**
   * @brief empty function, not used
   * @return NULL
   */

  virtual IplImage *Process(IplImage *image);
 /**
   * @brief empty function, not used
   * @return NULL
   */
  virtual void Config(IplImage *image, CvRect selection);

  /**
   * @brief initializes the undistortion map 
   * @return 1 upon success, 0 upon failure
   */
  int InitUndistortion(IplImage *img);

  /**
   *  @return the camera parameters, NULL upon failre
   */
  const CvCamera *GetCameraParams()const;

  void SetChessboardParams(int width_small,int height_big, float size);
  void SetChessboardParams(double p[3])
  {for(int i=0;i<3;i++)chessboard[i]=p[i];};

  double* GetChessboardParams(){return chessboard;}; 
 protected:
 /**
   * @brief class initialization (called by the constructor) 
   */
  void Init();
};




#endif

