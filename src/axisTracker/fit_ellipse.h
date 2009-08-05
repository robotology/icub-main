#ifndef FIT_ELLIPSE_INC
#define FIT_ELLIPSE_INC

#include <yarp/sig/Image.h>

#include "ConicPlotter.h"
/*
void fit_ellipse(float *x_in, float *y_in, int num_points,
		 double& xc, double& yc,
		 double& xa, double& ya,
		 double& la, double& lb);
*/

class FitEllipse : public ConicPlotter {
private:
  double pvec[7];
  yarp::sig::ImageOf<yarp::sig::PixelFloat> xs, ys;
  int len;
  double xc, yc, xa, ya, la, lb;
  double xx[4], yy[4];
  double Ao, Ax, Ay, Axx, Ayy, Axy;
  int maxlen;
public:
  FitEllipse() {
    maxlen = 1000;
    xs.resize(maxlen,1);
    ys.resize(maxlen,1);
    len = 0;
  }

  void apply(float *x_in, float *y_in, int num_points);

  int generate();

  float getX(int index) {
    //exit(1);
    return xs(index,0);
  }

  float getY(int index) {
    return ys(index,0);
  }

  void getParams(double& xc, double& yc,
		 double& xa, double& ya,
		 double& la, double& lb) {
    xc = this->xc;
    yc = this->yc;
    xa = this->xa;
    ya = this->ya;
    la = this->la;
    lb = this->lb;
  }

  // callback
  virtual int plot(int x, int y) {
    //printf(">>> %d %d\n", x, y);
    if (len>=maxlen) {
      return 0;
      //exit(1);
    }
    xs(len,0) = x;
    ys(len,0) = y;
    len++;
    return 1;
  }
};

#endif
