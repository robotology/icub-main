#ifndef __OBJSUPPORT__
#define __OBJSUPPORT__

class BlobInfo {
public:
  double roi_x;
  double roi_y;
  double roi_width;
  double roi_height;
  double angle;

  double hist[16];

  double area;
  double convexity;
  double eccentricity;
  double compactness;
  double circleness;
  double squareness;

  //  int readObjInfo(const Bottle &msg);

  void printBlob() {
    cout << "center: " << roi_x << ", " << roi_y << endl;
    cout << "size: " << roi_width << ", " << roi_height << endl;
    cout << "angle" << angle << endl;
    cout << "area" << area << endl;
    cout << "convexity" << convexity << " eccentricity "<< eccentricity << " compactness " << compactness << " cricleness "<< circleness << " squareness " << squareness << endl;
  }
};


class TrackInfo {
public:
  int roi_x;
  int roi_y;
  int roi_width;
  int roi_height;
  
  int hist[16];
  int v_min;
  int v_max;
  int s_min;

  //int readTrackInfo(const Bottle &msg);

  void printTrack() {
    cout << "center: " << roi_x << ", " << roi_y << endl;
    cout << "size: " << roi_width << ", " << roi_height << endl;
    cout << "v_min v_max and s_min" <<  v_min << " "<< v_max << " " << s_min << endl;
  }

};

#endif
