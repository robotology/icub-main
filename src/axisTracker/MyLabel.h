// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


#ifndef MYLABEL_INC
#define MYLABEL_INC

#include <yarp/sig/LabelImage.h>

using namespace yarp::sig;


class MyLabel : public yarp::sig::LabelImage {
public:
    double dist(double x1, double y1, double x2, double y2) {
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }

    hash_ii counts;
    hash_ip patches;
    int at_id;

    ImageOf<PixelFloat> *src_x;
    ImageOf<PixelFloat> *src_y;
    ImageOf<PixelFloat> *src_m;

    int interest;

    MyLabel() {
        src_x = src_y = src_m = NULL;
    }

    void connect(ImageOf<PixelFloat>& nsrc_x,
                 ImageOf<PixelFloat>& nsrc_y,
                 ImageOf<PixelFloat>& nsrc_m) {
        src_x = &nsrc_x;
        src_y = &nsrc_y;
        src_m = &nsrc_m;
        interest = 0;
        fixed = 0;
        at_id = -1;
        counts.clear();
        patches.clear();
    }

    /*
      void Clone(MyLabel& alt)
      {
      counts = alt.counts;
      patches = alt.patches;
      at_id = alt.at_id;
      }
    */

    double tot_x, tot_y, tot_x2, tot_y2, tot_xy;
    double tot_dx, tot_dy;
    int lo_x, lo_y;
    int hi_x, hi_y;
    float lo, hi;
    int xlike;

    int count2;
    int fixed;

    void Add(OrientedPatch& p1, int ct = 100)
    {
        at_id++;
        counts[at_id] = ct;
        patches[at_id] = p1;
    }

    void Reset()
    {
        at_id = 0;
    }

    void Note(int x, int y, float dx, float dy)
    {
        tot_x += x;
        tot_y += y;
        tot_x2 += x*x;
        tot_y2 += y*y;
        tot_xy += x*y;
        float v = tot_dx*dx+tot_dy*dy;
        if (v>0)
            {
                tot_dx += dx;
                tot_dy += dy;
            }
        else
            {
                tot_dx -= dx;
                tot_dy -= dy;
            }
        count2++;
    }

    float fx, fy, fdx, fdy;

    virtual void Notify(int x, int y)
    {
        float dx = (*src_x)(x,y);
        float dy = (*src_y)(x,y);
        Note(x,y,dx,dy);
        float d = fdx*(x-fx)+fdy*(y-fy);
        if (d>0)
            {
                if (d>hi)
                    {
                        hi_x = x;
                        hi_y = y;
                        hi = d;
                    }
            }
        else
            {
                d = -d;
                if (d>lo)
                    {
                        lo_x = x;
                        lo_y = y;
                        lo = d;
                    }
            }

        /*
          if (xlike)
          {
          if (x<lo_x)
	      {
          lo_x = x;
          lo_y = y;
	      }
          if (x>hi_x)
	      {
          hi_x = x;
          hi_y = y;
	      }
          }
          else
          {
          if (y<lo_y)
	      {
          lo_x = x;
          lo_y = y;
	      }
          if (y>hi_y)
	      {
          hi_x = x;
          hi_y = y;
	      }
          }
        */
    }

    virtual void Notify(int id, int count, int finished)
    {
        if (finished)
            {
                //if (count>20)
#ifndef FINE_DETAIL
                if (count>10)
#else
                    if (count>(DETAIL_THETA))
#endif
                        {
                            if (id>at_id)
                                {
                                    at_id = id;
                                }
                            interest++;
                            counts[id] = count;
                            double mean_x = tot_x/count;
                            double mean_y = tot_y/count;
                            double mean_dx = tot_dx/count;
                            double mean_dy = tot_dy/count;
                            // for now this is all we care about - many other
                            // measures available though
                            OrientedPatch patch;
                            patch.x = mean_x;
                            patch.y = mean_y;
                            patch.dx = mean_dx;
                            patch.dy = mean_dy;
                            patch.area = count;

                            //PFHIT -- refine endpoints
                            double len = (dist(lo_x,lo_y,hi_x,hi_y)/2);
                            lo_x = (int)(mean_x+len*mean_dx);
                            lo_y = (int)(mean_y+len*mean_dy);
                            hi_x = (int)(mean_x-len*mean_dx);
                            hi_y = (int)(mean_y-len*mean_dy);

                            patch.lo_x = lo_x;
                            patch.lo_y = lo_y;
                            patch.hi_x = hi_x;
                            patch.hi_y = hi_y;
                            patches[id] = patch;
                            //printf("Got (%g,%g) vector (%g,%g) count (%d,%d)\n",
                            //mean_x, mean_y, mean_dx, mean_dy, count, count2);
                        }
            }
        else
            {
                tot_x = tot_y = tot_x2 = tot_y2 = tot_xy = 0;
                tot_dx = tot_dy = 0;
                count2 = 0;
                fixed = 0;
                lo_x = lo_y = 10000;
                hi_x = hi_y = -1;
                lo = -1;
                hi = -1;
                xlike = 0;
            }
    }

    virtual int IsCompatible(int x1, int y1, int x2, int y2)
    {
        float m1 = (*src_m)(x1,y1);
        float m2 = (*src_m)(x2,y2);
        if (m1<0.5&&m2<0.5) {
            return 1;
        }
        if (m1>0.5&&m2<0.5) {
            return 0;
        }
        if (m2>0.5&&m1<0.5) {
            return 0;
        }

        int result = 0;
        float dx1 = (*src_x)(x1,y1);
        float dy1 = (*src_y)(x1,y1);
        if (!fixed)
            {
                fx = x1;
                fy = y1;
                lo_x = x1;  lo_y = y1;
                hi_x = x1;  hi_y = y1;
                fdx = dx1;
                fdy = dy1;
                if (fabs(fdx)>fabs(fdy))
                    {
                        xlike = 1;
                    }
                else
                    {
                        xlike = 0;
                    }
                fixed = 1;
            }
        if (x1!=x2 || y1!=y2)
            {
                float xx = fabs((x2-fx)*fdx+(y2-fy)*fdy);
                float yy = fabs(-(x2-fx)*fdy+(y2-fy)*fdx);
                int doomed = 0;
                if (xx<0.001f) xx = 0.001f;
                doomed = (yy>(0.1*xx+3));
                doomed = 0;

                if (!doomed)
                    {
                        float dx2 = (*src_x)(x2,y2);
                        float dy2 = (*src_y)(x2,y2);
                        //if (src_m(x1,y1)>=4 && src_m(x2,y2)>=4)
                        //if (src_m(x1,y1)>=5 && src_m(x2,y2)>=5)
                        //if (src_m(x1,y1)>=6 && src_m(x2,y2)>=6)
                        if ((*src_m)(x1,y1)>=0.5 && (*src_m)(x2,y2)>=0.5)
                            {
                                //if (src_m(x1,y1)<2 || src_m(x2,y2)<2)
                                //{
                                //  result = 0;
                                //}
                                if (fabs(dx1*dx2+dy1*dy2)>0.95)
                                    {
                                        if (fabs(dx1)+fabs(dy1)>0.1)
                                            {
                                                if (fabs(dx1)+fabs(dy1)>0.1)
                                                    {
                                                        result = 1;
                                                    }
                                            }
                                    }
                            }
                    }
                if (x2<3 || y2<3 || 
                    x2>=(*src_x).width()-3 || y2>=(*src_y).height()-3)
                    {
                        result = 0;
                    }
            }
        else
            {
                result = 1;
            }

          if (count2>40)
          {
              result = 0;
          }

        //printf("%d %d --> %d %d = %d\n", x1,y1, x2,y2, result);
        return result;
    }
};


#endif
