// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Thread.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>

#include <stdio.h>

#define PAN_MODE

extern double _status_eye_tilt;
extern double _status_eye_pan;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;

#include "NoseFinder.h"
#include "hough_transform.h"

#define MAX_IMG 100


class GlobalTrack {
public:
    ImageOf<PixelFloat> prev;
    ImageOf<PixelRgb> imgs[MAX_IMG];
    int xx, yy;
    int index;
    int writing;
    bool thinking;
    double eyeTilt;
    double thth;

    GlobalTrack() {
        index = 0;
        writing = -1;
        thinking = false;
        eyeTilt = 0;
    }

    void reset() {
        if (prev.width()!=0) {
            xx = prev.width()/2;
            yy = prev.height()/2;
        }
        thth = 0;
    }

    void apply(ImageOf<PixelFloat>& src, ImageOf<PixelRgb>& srcColor, ImageOf<PixelRgb>& dest) {
        if (prev.width()==0) {
            prev = src;
            xx = prev.width()/2;
            yy = prev.height()/2;
            return;
        }
        int gdx = 0, gdy = 0;
        double gtotal = 1e32;
        int ww = src.width();
        int hh = src.height();
        int step = 8;
        int mm = 8;
        for (int dx=-mm; dx<=mm; dx++) {
            for (int dy=-mm; dy<=mm; dy++) {
                double total = 0;
                for (int x=0.25*ww; x<ww*0.75; x+=step) {
                    for (int y=0; y<hh; y+=step) {
                        total += fabs(src.safePixel(x+dx,y+dy)-prev(x,y));
                    }
                }
                if (total<gtotal) {
                    gtotal = total;
                    gdx = dx;
                    gdy = dy;
                }
            }
        }
        bool update = false;
        static bool catchUpTime = false;
        static double lastBigMove = 0;
        double now = Time::now();
        if (gdx*gdx+gdy*gdy>=4*4) {
            lastBigMove = now;
            if (fabs(gdx)>2) {
                if (gdx>0) {
                    printf("moving left\n");
                    static int pruner = 0;
                    if (writing==-1) {
                        if (pruner%3==0) {
                            if (index<MAX_IMG) {
                                printf("added image %d\n", index);
                                if (index==0) {
#ifndef PAN_MODE
                                    eyeTilt = _status_eye_tilt;
#else
                                    eyeTilt = _status_eye_pan;
#endif
                                }
                                imgs[index] = srcColor;
                                index++;
                            }
                        }
                    }
                    pruner++;
                } else {
                    printf("moving right\n");
                }
            }
            xx += gdx;
            yy += gdy;
            update = true;
            if (catchUpTime) {
                printf("a good time to catch up on work stops\n");
                catchUpTime = false;
            }
        } else if (now-lastBigMove>1) {
            if (!catchUpTime) {
                if (index>0) {
                    printf("a good time to catch up on work starts\n");
                    catchUpTime = true;
                    if (writing==-1) {
                        writing = 0;
                    }
                }
            }
        }

        if (catchUpTime) {
            static int series = 0;
            if (writing!=-1 ) {
                if (writing>=index) {
                    writing = -1;
                    printf("Done writing\n");
                    series++;
                    index = 0;
                } else if (writing<index) {
                    printf("Writing image %d\n", writing);
                    char buf[256];
                    sprintf(buf,"/tmp/shake_%06d_%+06d_%06d.ppm", series, 
                            (int)(eyeTilt*10),
                            writing);
                    write(imgs[writing],buf);
                    writing++;
                }
            }
        }

        addCircle(dest,PixelRgb(255,0,0),xx,yy,10);
        if (update) {
            prev = src;
        }
    }
};

class Bimode {
public:
    int x0, y0;
    int x1, y1;
    bool active;

    Bimode() {
        x0 = y0 = x1 = y1 = 0;
        active = false;
    }


    void apply(ImageOf<PixelFloat>& src, ImageOf<PixelRgb>& dest) {
        if (!active) {
            // top right ish
            x0 = src.width()-1;
            y0 = 0;
            // bottom left ish
            x1 = src.width()-50;
            y1 = src.height()-1;
            active = 1;
        }

        float th = 1;
        // compare with top-right ish (x0,y0)
        ImageOf<PixelFloat> vals;
        vals.resize(src.width(),1);
        vals.zero();
        IMGFOR(src,x,y) {
            if (y>y0 && x<x0) {
                // compute x intersection
                float m = (y-y0)/((float)x-x0);
                int xi = x0+(y1-y0)/m;
                // so blame goes to (xi,y1)
                // compute value
                float val = 0;
                if (src(x,y)>th) {
                    val = 2;
                } else {
                    val = -1;
                }
                // update
                if (xi>0 && xi<x0) {
                    for (int xx=x0/2; xx<=xi; xx++) {
                        vals.safePixel(xx,0) += val;
                    }
                }
            }
        }
        int xpref = 0;
        float vpref = 0;
        for (int i=0; i<vals.width(); i++) {
            if (vals(i,0)>vpref) {
                xpref = i;
                vpref = vals(i,0);
            }
        }
        //printf("xpref is %d with val of %g\n", xpref, vpref);
        x1 = xpref;

        // other direction now

        int top = src.width(); //20;
        vals.resize(src.height()+top,1);
        vals.zero();
        IMGFOR(src,x,y) {
            if (x>x1 && y<y1) {
                // compute x intersection
                float m = (y-y1)/((float)x-x1);
                int yi = y1+m*(x0-x1);
                // so blame goes to (x0,yi)
                // compute value
                float val = 0;
                if (src(x,y)>th) {
                    val = 2;
                } else {
                    val = -1;
                }
                // update
                if (yi>-top && yi<y1) {
                    for (int yy=-top; yy<=yi; yy++) {
                        vals.safePixel(yy+top,0) += val;
                    }
                }
            }
        }


        int ypref = 0;
        vpref = 0;
        for (int i=0; i<vals.width(); i++) {
            if (vals(i,0)>vpref) {
                ypref = i-top;
                vpref = vals(i,0);
            }
        }
        //printf("ypref is %d with val of %g\n", ypref, vpref);
        y0 = ypref;

        float good = 0;
        float bad = 0;
        IMGFOR(src,x,y) {
            int side = ((x0-x1)*(y-y1)-(y0-y1)*(x-x1))>0;
            int v = src(x,y)>th;
            good += side==v;
            bad += (side!=v);
        }
        //printf("good %g, bad %g\n", good, bad);

        addCircle(dest,PixelRgb(255,0,0),x1,y1,10);
        addCircle(dest,PixelRgb(0,255,0),x0,y0,10);

        if (1) {
            int thick = 3;
            PixelRgb pix(0,0,255);
            if (good*0.01>bad) {
                thick = 5;
                pix.r = 255;
            }
            if (good*0.1>bad) {
                pix.g = 255;
                pix.b = 0;
            }
            for (float f=0; f<1; f+=0.01) {
                float ix = x0+f*(x1-x0);
                float iy = y0+f*(y1-y0);
                addCircle(dest,pix,(int)ix,(int)iy,thick);
            }
            if (thick>4) {
                for (float f=0; f<1; f+=0.01) {
                    float ix = x0+f*(x1-x0);
                    float iy = y0+f*(y1-y0);
                    addCircle(dest,PixelRgb(0,0,0),(int)ix,(int)iy,2);
                }
            }
        }


    }
};


void NoseFinder::reset() {
    mutex.wait();
    ImageOf<PixelRgb> blank;
    prev.copy(blank);
    mask.copy(blank);
    mutex.post();
}

void NoseFinder::apply(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src,
                       yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest) {
    mutex.wait();
    if (prev.width()==0) {
        prev = src;
        dest = src;
        mask.copy(src);
        mask.zero();
        mutex.post();
        return;
    }
    dest = src;
    float total = 0;
    int area = 0;
    IMGFOR(dest,x,y) {
        PixelRgb pix0 = src(x,y);
        PixelRgb pix1 = prev(x,y);
        bool out = false;
        if (mask(x,y)==255) {
            out = true;
        }
        if (!out) {
            float t0 = pix0.r+pix0.g+pix0.b;
            float t1 = pix1.r+pix1.g+pix1.b;
            int th = 50;
            float dr = fabs(pix0.r/t0 - pix1.r/t1) * (pix0.r>th);
            float dg = fabs(pix0.g/t0 - pix1.g/t1) * (pix0.g>th);
            float db = fabs(pix0.b/t0 - pix1.b/t1) * (pix0.b>th);
            if (dr+dg+db>0.05) {
                out = true;
            }
        }
        if (out) {
            dest(x,y) = PixelRgb(0,0,0);
            mask(x,y) = 255;
            total++;
        }
        area++;
    }
    total/=area;
    static int ct = 0;
    ct++;
    if (ct%2==0) {
        prev = src;
    }
    mutex.post();
}



///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

class NoseMaster : public TypedReaderCallback<ImageOf<PixelRgb> >, 
   public NoseControl {
public:
    bool dead;
    Semaphore working;
    BufferedPort<ImageOf<PixelRgb> > port;
    NoseFinder finder;
    Bimode bimode;
    GlobalTrack track;

    NoseMaster() : working(1) {
        dead = 0;
    }

    ~NoseMaster() {
        working.wait();
        dead = 1;
        working.post();
    }

    void init() {
        port.useCallback(*this);
        port.open("/nose");
        Network::connect("/bozo", "/nose");
        //Network::connect("/james/cam/left", "/nose");
        Network::connect("/nose", "/james/nose");
    }

    void reset() {
        finder.reset();
        track.reset();
    }
    
    void onRead(ImageOf<PixelRgb>& img) {
        if (dead) return;
        working.wait();
        if (dead) return;
        ImageOf<PixelRgb>& output = port.prepare();
        /*
        IMGFOR(img,x,y) {
            double v = x/320.0 + 0.25*y/240.0;
            if (v<0.5) {
                img(x,y) = PixelRgb(0,0,0);
            }
        }
        */

#if 0
        finder.apply(img,output);
        ImageOf<PixelFloat> dy, hgh;
        HoughState hs;
        dy.copy(output);
        IMGFOR(dy,x,y) {
            if (output(x,y).r>0) {
                dy(x,y) = 255;
            } else {
                dy(x,y) = 0;
            }
        }
        bimode.apply(dy,output);
#else
        output = img;
        ImageOf<PixelFloat> dy;
        dy.copy(img);
        for (int i=0; i<output.width(); i++) {
            output(i,output.height()/2).r = 255;
        }
        track.apply(dy,img,output);
#endif

        /*
        float total = 0;
        int area = 0;
        IMGFOR(dy,x,y) {
            if (output(x,y).r<10) {
                total++;
            }
            int mm = 5;
            float v = output(x,y).r - output.safePixel(x-mm,y).r;
            float v2 = output(x,y).r - output.safePixel(x-mm,y-mm).r;
            if (v2<v) v = v2;
            if (output.safePixel(x+2,y).r<10) {
                v = 0;
            }
            if (v<10) {
                v = 0;
            } else {
                v = 255;
            }
            // mask
            if (x<=mm+1||y<=mm+1||x>=dy.width()-mm-1||y>=dy.height()-mm-1) {
                v = 0;
            }
            dy(x,y) = v;
            area++;
        }
        total/=area;
        //printf("total is %g\n", total);
        IMGFOR(dy,x,y) {
            if (dy(x,y)>10) {
                output(x,y).r = 0;
                output(x,y).g = 0;
                output(x,y).b = 255;
            }
        }
        hough_transform(dy,hgh,
                        10,
                        M_PI/2,0.5,
                        0,0,
                        hs);

        if (hs.best_angle>M_PI/2) hs.best_angle-=M_PI;
        if (hs.best_angle<-M_PI/2) hs.best_angle+=M_PI;
        
        int door_sgn = 1;

        if (total>0.55) {
            for (int xx=-5; xx<=5; xx++) {
                for (int d=-200; d<200; d++) {
                    int x0 = (int)(0.5+hs.best_x+cos(hs.best_angle)*d+
                                   xx*door_sgn*sin(hs.best_angle));
                    int y0 = (int)(0.5+hs.best_y-sin(hs.best_angle)*d+
                                   xx*door_sgn*cos(hs.best_angle));
                    PixelRgb& pix = output.safePixel(x0,y0);
                    if (xx>0) {
                        pix.g = 255;
                    } else {
                        pix.r = 255;
                    }
                }
            }
        }
        */

        port.write();
        working.post();
    }
};

static NoseMaster nose;

NoseControl& NoseFinder::run(int argc, char *argv[]) {
    nose.init();
    //return nose.finder;
    return nose;
}

