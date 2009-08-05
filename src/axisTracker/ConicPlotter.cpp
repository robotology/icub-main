#include "ConicPlotter.h"

#define printf if (0) printf

static int DIAGx[] = {999, 1,  1, -1, -1, -1, -1,  1,  1};
static int DIAGy[] = {999, 1,  1,  1,  1, -1, -1, -1, -1};
static int SIDEx[] = {999, 1,  0,  0, -1, -1,  0,  0,  1};
static int SIDEy[] = {999, 0,  1,  1,  0,  0, -1, -1,  0};

#define DEBUG 0

inline int odd(int n)
{
  return n&1;
}


int getoctant(int gx, int gy)
{
  // Use gradient to identify octant.
  int upper = abs(gx)>abs(gy);
  if (gx>=0)                            // Right-pointing
    if (gy>=0)                          //    Up
      return 4 - upper;
    else                                //    Down
      return 1 + upper;
  else                                  // Left
    if (gy>0)                           //    Up
      return 5 + upper;
    else                                //    Down
      return 8 - upper;
}

int ConicPlotter::draw(int xs, int ys, int xe, int ye)
{
  int plots = 0;
  A *= 4;
  B *= 4;
  C *= 4;
  D *= 4;
  E *= 4;
  F *= 4;

  if (DEBUG) fprintf(stderr,"ConicPlotter::draw -- %d %d %d %d %d %d\n", A,B,C,D,E,F);

  // Translate start point to origin...
  F = A*xs*xs + B*xs*ys + C*ys*ys + D*xs + E*ys + F;
  D = D + 2 * A * xs + B * ys;
  E = E + B * xs + 2 * C * ys;

  // Work out starting octant
  int octant = getoctant(D,E);
  
  int dxS = SIDEx[octant]; 
  int dyS = SIDEy[octant]; 
  int dxD = DIAGx[octant];
  int dyD = DIAGy[octant];

  int d,u,v;
  switch (octant) {
  case 1:
    d = A + B/2 + C/4 + D + E/2 + F;
    u = A + B/2 + D;
    v = u + E;
    break;
  case 2:
    d = A/4 + B/2 + C + D/2 + E + F;
    u = B/2 + C + E;
    v = u + D;
    break;
  case 3:
    d = A/4 - B/2 + C - D/2 + E + F;
    u = -B/2 + C + E;
    v = u - D;
    break;
  case 4:
    d = A - B/2 + C/4 - D + E/2 + F;
    u = A - B/2 - D;
    v = u + E;
    break;
  case 5:
    d = A + B/2 + C/4 - D - E/2 + F;
    u = A + B/2 - D;
    v = u - E;
    break;
  case 6:
    d = A/4 + B/2 + C - D/2 - E + F;
    u = B/2 + C - E;
    v = u - D;
    break;
  case 7:
    d = A/4 - B/2 + C + D/2 - E + F;
    u =  -B/2 + C - E;
    v = u + D;
    break;
  case 8:
    d = A - B/2 + C/4 + D - E/2 + F;
    u = A - B/2 + D;
    v = u - E;
    break;
  default:
    fprintf(stderr,"FUNNY OCTANT\n");
    abort();
  }
  
  int k1sign = dyS*dyD;
  int k1 = 2 * (A + k1sign * (C - A));
  int Bsign = dxD*dyD;
  int k2 = k1 + Bsign * B;
  int k3 = 2 * (A + C + Bsign * B);

  // Work out gradient at endpoint
  int gxe = xe - xs;
  int gye = ye - ys;
  int gx = 2*A*gxe +   B*gye + D;
  int gy =   B*gxe + 2*C*gye + E;
  
  int octantcount = getoctant(gx,gy) - octant;
  if (octantcount < 0)
    octantcount = octantcount + 8;
  else if (octantcount==0)
    if((xs>xe && dxD>0) || (ys>ye && dyD>0) ||
       (xs<xe && dxD<0) || (ys<ye && dyD<0))
      octantcount +=8;

  if (DEBUG)
    fprintf(stderr,"octantcount = %d\n", octantcount);
  
  int x = xs;
  int y = ys;

  while (octantcount > 0) {

    if (DEBUG)
      fprintf(stderr,"-- %d -------------------------\n", octant); 
    
    if (odd(octant)) {
      while (2*v <= k2) {

        // Plot this point
        int ok = plot(x,y);
	plots++;
	if (!ok) {
	  printf("aborting line\n");
	  return -1;
	}
        
        // Are we inside or outside?
        if (DEBUG) fprintf(stderr,"x = %3d y = %3d d = %4d\n", x,y,d);
        if (d < 0) {                    // Inside
          x = x + dxS;
          y = y + dyS;
          u = u + k1;
          v = v + k2;
          d = d + u;
        }
        else {                          // outside
          x = x + dxD;
          y = y + dyD;
          u = u + k2;
          v = v + k3;
          d = d + v;
        }
      }
    
      d = d - u + v/2 - k2/2 + 3*k3/8; 
      // error (^) in Foley and van Dam p 959, "2nd ed, revised 5th printing"
      u = -u + v - k2/2 + k3/2;
      v = v - k2 + k3/2;
      k1 = k1 - 2*k2 + k3;
      k2 = k3 - k2;
      int tmp = dxS; dxS = -dyS; dyS = tmp;
    }
    else {                              // Octant is even
      while (2*u < k2) {

        // Plot this point
        int ok = plot(x,y);
	plots++;
	if (!ok) {
	  printf("aborting line\n");
	  return -1;
	}
        if (DEBUG) fprintf(stderr,"x = %3d y = %3d d = %4d\n", x,y,d);
        
        // Are we inside or outside?
        if (d > 0) {                    // Outside
          x = x + dxS;
          y = y + dyS;
          u = u + k1;
          v = v + k2;
          d = d + u;
        }
        else {                          // Inside
          x = x + dxD;
          y = y + dyD;
          u = u + k2;
          v = v + k3;
          d = d + v;
        }
      }
      int tmpdk = k1 - k2;
      d = d + u - v + tmpdk;
      v = 2*u - v + tmpdk;
      u = u + tmpdk;
      k3 = k3 + 4*tmpdk;
      k2 = k1 + tmpdk;
      
      int tmp = dxD; dxD = -dyD; dyD = tmp;
    }
    
    octant = (octant&7)+1;
    octantcount--;
  }

  // Draw final octant until we reach the endpoint
  if (DEBUG)
    fprintf(stderr,"-- %d (final) -----------------\n", octant); 
    
  if (odd(octant)) {
    while (2*v <= k2) {

      // Plot this point
      int ok = plot(x,y);
	plots++;
      if (!ok) {
	printf("Aborted line\n");
	return -1;
      }
      if (x == xe && y == ye)
        break;
      if (DEBUG) fprintf(stderr,"x = %3d y = %3d d = %4d\n", x,y,d);
      
      // Are we inside or outside?
      if (d < 0) {                      // Inside
        x = x + dxS;
        y = y + dyS;
        u = u + k1;
        v = v + k2;
        d = d + u;
      }
      else {                            // outside
        x = x + dxD;
        y = y + dyD;
        u = u + k2;
        v = v + k3;
        d = d + v;
      }
    }
  }
  else {                                // Octant is even
    while ((2*u < k2)) {

      // Plot this point
      int ok = plot(x,y);
	plots++;
      if (!ok) { 
	  printf("aborting line\n");
	  return -1; 
      }
      if (x == xe && y == ye)
        break;
      if (DEBUG) fprintf(stderr,"x = %3d y = %3d d = %4d\n", x,y,d);
      
      // Are we inside or outside?
      if (d > 0) {                      // Outside
        x = x + dxS;
        y = y + dyS;
        u = u + k1;
        v = v + k2;
        d = d + u;
      }
      else {                            // Inside
        x = x + dxD;
        y = y + dyD;
        u = u + k2;
        v = v + k3;
        d = d + v;
      }
    }
  }

  return 1;
}


int ConicPlotter::plot(int x, int y)
{
  printf("ConicPlotter::plot(%d, %d)\n", x, y);
  return 1;
}

ConicPlotter::ConicPlotter() { assign(0,0,0,0,0,0); }
ConicPlotter::ConicPlotter(int A, int B, int C, int D, int E, int F) { assign(A,B,C,D,E,F); }

void ConicPlotter::assign(int A_, int B_, int C_, int D_, int E_, int F_)
{
  A = A_;
  B = B_;
  C = C_;
  D = D_;
  E = E_;
  F = F_;
}

inline int rnd(double x) { return (x>=0.0)?(int)(x + 0.5):(int)(x - 0.5); }

void ConicPlotter::assignf(double scale,
                           double A_, double B_, double C_, double D_, double E_, double F_)
{
  A = rnd(A_ * scale);
  B = rnd(B_ * scale);
  C = rnd(C_ * scale);
  D = rnd(D_ * scale);
  E = rnd(E_ * scale);
  F = rnd(F_ * scale);
}

