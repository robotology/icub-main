#ifndef __LOCATOR3D_H__ 
#define __LOCATOR3D_H__ 




#define MAX_OBJ 100

/**
 * @brief performs the 3D localization from 2d position in two images
 */
class Locator3D
{
 protected:

  char paramfile[80];
  int isCalibrated;
  int nb_points;
  Cv3dTrackerCameraInfo camInfos[2];
  Cv3dTrackerTrackedObject pos3d[MAX_OBJ];
  Cv3dTracker2dTrackedObject pos2d[2][MAX_OBJ];
  Cv3dTrackerCameraIntrinsics camera_intrinsics[2];
 public:
  
  Locator3D();
  Locator3D(char *params);
  //  ~Locator3D();
  
  /**
   * @brief calibrates the extrinsic parameters
   * @param img a set of one view of the checkerboard for each camera
   * @param cam the set of camera parameters
   * @param checkerboard_size the size of the checkerboard (number of inner corners)
   * @param sq_size the size of each square of the checkerboard
   * @return 1 if the calibration was successful, 0 otherwise.
  */
  int Calibrate(IplImage **img, const CvCamera *cam[], CvSize checkerbord_size,float sq_size);
  /**
   * @brief performs the 3D localization
   */
  int Locate(int nbpoints, const Vec2 *p1, const Vec2 *p2, Vec3 *out, int *found);
  int LocateAllPoints();
  void AddPoint(Vec2 p1, Vec2 p2);
  int Get3dPoint(int i,Vec3 *point);
  
  int GetNbPoints()const{return nb_points;}
  int IsCalibrated()const;
  void Decalibrate();
  int SaveParams(const char* filename);
  int SaveParams(){return SaveParams(paramfile);};
  void PrintParams(int k=1);
  void SetDefaultParams(float angle0, float angle1, float dist);
  void SetDefaultTranslation(float dist);
  void CenterOrigin(bool apply = true);
  void SetIntrinsicsParams(const CvCamera *cam[]);
  void SetIntrinsicsParams(int i, const CvCamera *cam);

  int LoadParams(const char* filename);
 
  void ResetPoints(){nb_points=0;}
  const Cv3dTrackerCameraInfo *GetCameraInfo()const
  {if(IsCalibrated())return camInfos;return NULL;};
};





#endif
