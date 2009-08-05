
#ifndef CONICPLOT
#define CONICPLOT


//
// CONIC  2D Bresenham-like conic drawer.
//       CONIC(Sx,Sy, Ex,Ey, A,B,C,D,E,F) draws the conic specified
//       by A x^2 + B x y + C y^2 + D x + E y + F = 0, between the
//       start point (Sx, Sy) and endpoint (Ex,Ey).

// Author: Andrew W. Fitzgibbon (andrewfg@ed.ac.uk),
//         Machine Vision Unit,
//         Dept. of Artificial Intelligence,
//         Edinburgh University,
// 
// Date: 31-Mar-94
// Version 2: 6-Oct-95
//      Bugfixes from Arne Steinarson <arst@ludd.luth.se>

// Available on author's website.  No license mentioned...
// http://research.microsoft.com/%7Eawf/graphics/bres-ellipse.html

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

class ConicPlotter {
public:
  // A xx + B xy + C yy + D x + E y + F = 0
 
  int A;
  int B;
  int C;
  int D;
  int E;
  int F;
 
  ConicPlotter();
  ConicPlotter(int A, int B, int C, int D, int E, int F);
  void assign(int A, int B, int C, int D, int E, int F);
  void assignf(double SCALE, double A, double B, double C, double D, double E, double F);
  
  // This is the routine which draws the conic, calling the plot member at each pixel.
  int draw(int xs, int ys, int xe, int ye);
 
  // User supplies this plot routine, which does the work.
  virtual int plot(int x, int y);
};
 
#endif
