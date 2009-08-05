// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Property.h>
#include <yarp/os/NetInt32.h>
#include <yarp/os/all.h>
#include <yarp/sig/GlutOrientation.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>

#include "stuff.h"
#include "Label2Image.h"
#include "ConicPlotter.h"

#include "fit_ellipse.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::file;
using namespace yarp::sig::draw;


static double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}


class Sector {
public:
    double xx, yy;

    void apply(OrientedPatch& p, OrientedPatch& q) {
        double num = -(q.x-p.x)*q.dx - (q.y-p.y)*q.dy;
        double denom = - p.dy*q.dx + p.dx*q.dy;
        if (fabs(denom)<0.0001) {
            xx = 0;
            yy = 0;
            return;
        }
        double k = num/denom;
        xx = p.x + k*p.dy;
        yy = p.y - k*p.dx;
    }

    double getX() { return xx; }
    double getY() { return yy; }
};


int use_test = 1;
int capacity = 50;




/**
 * This test should return a number >= 0.
 * Any value greater than 1 means the test passed.
 */
float applyTest(ImageOf<PixelRgb>& img, int xx, int yy) {
    // weak pink bias
    int likes = 0;
    int tots = 0;
    int dd = 5;
    for (int i=-dd; i<=dd; i++) {
        for (int j=-dd; j<=dd; j++) {
            PixelRgb& cent = img.safePixel(xx+i,yy+j);
            if (cent.r > cent.g*1.05 && 
                //cent.r > cent.b*1.05 && 
                //cent.g > cent.r*0.3 &&
                //cent.b > cent.r*0.3 &&
                //cent.g > cent.b*0.6 &&
                //cent.b > cent.g*0.6 &&
                //cent.b > cent.g*1.05 && 
                cent.r > 50) {
                likes++;
            }
            tots++;
        }
    }
    
    if (!use_test) {
        return 1;
    }

    return (likes)/25.0;
}




int process(Searchable& config, 
            ImageOf<PixelRgb>& img, 
            ImageOf<PixelRgb>& dest,
            int& xout, int& yout, double& conf) {
    conf = 0;
    xout = 0;
    yout = 0;

    int saveMode = config.check("save_intermediate", "write to file");
    bool dropRes = false;
    //config.check("fast", "should we prefer speed to accuracy");

    if (dropRes)
        {
            fprintf(stderr, "Working with low resolution\n");
        }

    if (dropRes) {
        // reduce resolution
        ImageOf<PixelRgb> img2;
        img2.copy(img,img.width()/2,img.height()/2);
        img = img2;
    }
  

    GlutOrientation orient("data/orient.txt");
    ImageOf<PixelRgb> save;
    ImageOf<PixelFloat> dx, dy, mag;

    
    int root_combinations = 0;
    static int luminance = 5;
    luminance -= 2;


    ImageOf<PixelRgb> gen;
    ImageOf<PixelFloat> sum, center, xdir, ydir;
    ImageOf<PixelRgbFloat> fsum;
    int ct = 0;
    MyLabel label;
    ImageOf<PixelInt> group;

    int rep = 0;
    do {
        printf("Luminance at %d\n", luminance);
        if (luminance<3) {
            luminance = 3;
        }
        orient.SetLuminanceFilter(luminance);
        orient.SetDemocracy(1);
        orient.Apply(img,dest,dx,dy,mag);

        if (saveMode) {
            write(dest,"foo.ppm");
        }

        save = dest;

        printf("building label\n");
        label.connect(dx,dy,mag);
        printf("applying label\n");
        group.resize(dx);
        group.zero();
        label.Reset();
        label.Apply(dx.width(),dx.height(),group);
        printf("applied label\n");
        label2image(group,dest);

        if (saveMode) {
            write(dest,"label.ppm");
        }

        ImageOf<PixelFloat> mat;
        mat.copy(group);

        if (saveMode) {
            write(mat,"label.txt");
        }

        gen.resize(dest);
        gen.zero();
        ct = 0;
        root_combinations = 0;
        for (hash_ip::iterator it=label.patches.begin(); 
             it!=label.patches.end();
             it++) {
            OrientedPatch& p = (*it).second;
            if (p.area<400) {
                int id = group(p.x,p.y);
                PixelRgb pix;
                pix.r = (id*341)%200+50;
                pix.g = (id*9707)%256;
                pix.b = (id*914)%256;
                if (dist(p.lo_x,p.lo_y,p.hi_x,p.hi_y)<50) {
                    if (applyTest(img,p.x,p.y)>=0.99) {
                        //printf("patch %g\n", p.area);
                        root_combinations++;
                        addSegment(gen,pix,p.lo_x,p.lo_y,p.hi_x,p.hi_y);
                        //addSegment(gen,pix,p.x,p.y,p.x+20*p.dx,p.y+20*p.dy);
                        ct++;
                    } else {
                        p.area = 0;
                    }
                } else {
                    p.area = 0;
                }
            } else {
                p.area = 0;
            }
        }

        if (saveMode) {
            write(gen,"clean_label.ppm");
        }
        //dest = gen;
        //return 0;


        printf("root combinations %d (cubes to %ld)\n",
               root_combinations,
               root_combinations*root_combinations*root_combinations);

        rep = root_combinations>capacity;

        if (rep) {
            luminance += 2;
        }
    } while (rep);

    
    ImageOf<PixelRgb> edge;
    edge = gen;

    gen = save;
    IMGFOR(gen,x,y) {
        PixelRgb& pix = gen(x,y);
        pix.r /= 2;
        pix.g /= 2;
        pix.b /= 2;
        pix.g = 0;
    }
    gen.zero();

    sum.resize(gen);
    sum.zero();
    center.resize(gen);
    center.zero();
    fsum.resize(gen);
    fsum.zero();
    xdir.resize(gen);
    xdir.zero();
    ydir.resize(gen);
    ydir.zero();
    dest.zero();
    int combinations = 0;
    for (hash_ip::iterator it=label.patches.begin(); 
         it!=label.patches.end();
         it++) {
        fflush(stdout);
        OrientedPatch& p = (*it).second;

        if (p.area<=0.5) {
            continue;
        }
        if (applyTest(img,p.x,p.y)<0.99) {
            continue;
        }

        printf(".");


        for (hash_ip::iterator it2=label.patches.begin(); 
             it2!=label.patches.end();
             it2++) {
            OrientedPatch& q = (*it2).second;

            if (q.area<=0.5) {
                continue;
            }

            if (applyTest(img,q.x,q.y)<0.99) {
                continue;
            }

            /*
            if (fabs(p.dx*q.dx+p.dy*q.dy)>0.9) {
                continue;
            }
            */

            if (applyTest(img,0.5*(p.x+q.x),0.5*p.y+q.y)<0.99) {
                continue;
            }

            for (hash_ip::iterator it3=label.patches.begin(); 
                 it3!=label.patches.end();
                 it3++) {
                OrientedPatch& r = (*it3).second;

                if (r.area<=0.5) {
                    continue;
                }

                if (applyTest(img,0.5*(p.x+r.x),0.5*p.y+r.y)<0.99) {
                    continue;
                }

                if (applyTest(img,0.5*(q.x+r.x),0.5*q.y+r.y)<0.99) {
                    continue;
                }


                /*
                if (fabs(p.dx*r.dx+p.dy*r.dy)>0.9) {
                    continue;
                }

                if (fabs(q.dx*r.dx+q.dy*r.dy)>0.9) {
                    continue;
                }
                */


                /*
                float s = 2;
                float s1 = s;
                float x_in[] = {
                    p.x-p.dx*s1,
                    p.x+p.dx*s1,
                    q.x-q.dx*s1,
                    q.x+q.dx*s1,
                    r.x-r.dx*s1,
                    r.x+r.dx*s1
                };

                float s2 = s;
                float y_in[] = {
                    p.y-p.dy*s2,
                    p.y+p.dy*s2,
                    q.y-q.dy*s2,
                    q.y+q.dy*s2,
                    r.y-r.dy*s2,
                    r.y+r.dy*s2
                };
                */

                float x_in[] = {
                    /*
                    p.x-p.dx,
                    p.x+p.dx,
                    q.x-q.dx,
                    q.x+q.dx,
                    r.x-r.dx,
                    r.x+r.dx,
                    */
                    p.lo_x,
                    p.hi_x,
                    q.lo_x,
                    q.hi_x,
                    r.lo_x,
                    r.hi_x
                };
                float y_in[] = {
                    /*
                    p.y-p.dy,
                    p.y+p.dy,
                    q.y-q.dy,
                    q.y+q.dy,
                    r.y-r.dy,
                    r.y+r.dy,
                    */
                    p.lo_y,
                    p.hi_y,
                    q.lo_y,
                    q.hi_y,
                    r.lo_y,
                    r.hi_y
                };

                double xx, yy, xa, ya, la, lb;
                if (it!=it2 && it!=it3 && it2!=it3) {

                    Sector sect;
                    sect.apply(p,q);
                    double xpq = sect.getX();
                    double ypq = sect.getY();

                    double dot1 = (xpq-p.x)*(q.x-p.x)+(ypq-p.y)*(q.y-p.y);
                    double dot2 = (xpq-q.x)*(p.x-q.x)+(ypq-q.y)*(p.y-q.y);

                    if (dot1>0 && dot2>0) {
                      
                        Sector sect2;
                        sect2.apply(p,r);
                        double xqr = sect2.getX();
                        double yqr = sect2.getY();
                        double dot3 = (xqr-q.x)*(r.x-q.x)+(yqr-q.y)*(r.y-q.y);
                        double dot4 = (xqr-r.x)*(q.x-r.x)+(yqr-r.y)*(q.y-r.y);

                        if (dot3>0 && dot4>0) {
                            xx = 99;
                            FitEllipse ellipse;
                            ellipse.apply(x_in,y_in,6);
                            ellipse.getParams(xx,yy,xa,ya,la,lb);

                            float goodness = applyTest(img, xx, yy);

                            if (goodness<0.99) {
                                xx = -1;
                            }

                            PixelRgb pix(255,255,255);
                      
                            double q = 0;

                            if (fabs(xx)<1000 && fabs(yy)<1000 && xx>=0 && yy>=0 &&
                                la>=3 && lb>=3 && (la>=10||lb>=10)) {

                                int NN = ellipse.generate();
                                //printf("generation done\n");

                                int ct = 0;
                                ct++;
                                double theta0 = atan2(ya,xa); 
                                // perpendicular
                                double c0 = cos(theta0);
                                double s0 = sin(theta0);

                                bool merited = false;
                                double merit = 0;
                                int merit_ct = 0;

                                mag.safePixel(-1,-1) = 0;
                                /*
                                for (double theta=0; theta<M_PI*2; theta+=M_PI/16) {
                                    double c = cos(theta);
                                    double s = sin(theta);
                                    double x0 = xx + la*c0*c + lb*s0*s;
                                    double y0 = yy - la*s0*c + lb*c0*s;
                                */

                                double px = ellipse.getX(NN-2);
                                double py = ellipse.getY(NN-2);
                                double ppx = ellipse.getX(NN-1);
                                double ppy = ellipse.getY(NN-1);
                                int side = 0;
                                for (int vv=0; vv<NN && NN>=4; vv++) {
                                    double x0 = ellipse.getX(vv);
                                    double y0 = ellipse.getY(vv);
                                    /*
                                    printf(">> %g %g // %g %g // %g %g\n",
                                           x0, y0,
                                           px, py,
                                           ppx, ppy);
                                    */

                                    PixelFloat& ft = mag.safePixel((int)x0,
                                                                   (int)y0);
                                    if (ft>0.5) {
                                        int key = vv/(NN/4);
                                        int lut[] = {1, 2, 4, 8};
                                        side |= lut[key];
                                        merit++;
                                    }
                                    merit_ct++;

                                    ppx = px;
                                    ppy = py;
                                    px = x0;
                                    py = y0;
                                }

                                merit /= (merit_ct+0.0001);
                                /*
                                if (side!=15) {
                                    merit = 0;
                                }
                                */
                                
                                if (merit>0.4) {
                                    merited = true;
                                    combinations++;

                                    int dd = 2;
                                    for (int ii=xx-dd; ii<=xx+dd; ii++) {
                                        for (int jj=yy-dd; jj<=yy+dd; jj++) {
                                            PixelFloat& ft = 
                                                center.safePixel(ii,jj);
                                            ft+=merit;
                                            double xvect = -xa;
                                            double yvect = ya;
                                            double xvect0 = 
                                                xdir.safePixel(ii,jj);
                                            double yvect0 = 
                                                ydir.safePixel(ii,jj);
                                            if (xvect*xvect0+yvect*yvect0<0) {
                                                xvect *= -1;
                                                yvect *= -1;
                                            }
                                            xdir.safePixel(ii,jj) += xvect;
                                            ydir.safePixel(ii,jj) += yvect;
                                        }
                                    }
                                }

                                if (merited) {
                                    for (int vv=0; vv<NN; vv++) {
                                        double x0 = ellipse.getX(vv);
                                        double y0 = ellipse.getY(vv);
                                        int dd = 2;
                                  
                                        for (int ii=x0-dd; ii<=x0+dd; ii++) {
                                            for (int jj=y0-dd; jj<=y0+dd; jj++) {
                                                PixelFloat& ft = 
                                                    sum.safePixel(ii,jj);
                                                PixelRgbFloat& ft2 = 
                                                    fsum.safePixel(ii,jj);
                                                //merit = 1;
                                                ft += merit;
                                                double ratio = la/lb;
                                                if (ratio>1) ratio = 1/ratio;
                                                ratio = (1-ratio);
                                                ft2.r += merit*0.25;
                                                ft2.g += merit*ratio;
                                                //dest.safePixel(ii,jj).r = 255;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (saveMode) {
        write(dest,"tmp.ppm");
    }

    printf("\n");

    printf("%d combinations evaluated\n", combinations);

    double pk = 0.0001;
    double pk2 = 0.0001;
    IMGFOR(sum,x,y) {
        if (sum(x,y)>pk) {
            pk = sum(x,y);
        }
        if (fsum(x,y).r>pk2) {
            pk2 = fsum(x,y).r;
        }
        if (fsum(x,y).g>pk2) {
            pk2 = fsum(x,y).g;
        }
    }

    int pcx = -100;
    int pcy = -100;
    double pc = 0;
    IMGFOR(center,x,y) {
        double c = center(x,y);
        if (c>pc) {
            pc = c;
            pcx = x;
            pcy = y;
        }
    }

    double xvect = xdir.safePixel(pcx,pcy);
    double yvect = ydir.safePixel(pcx,pcy);
    double norm = dist(0,0,xvect,yvect);
    if (norm>=0.0001) {
        xvect /= norm;
        yvect /= norm;
    }
    IMGFOR(gen,x,y) {
        PixelRgb& pix = gen(x,y);
        if (pix.r==0&&pix.g==0&&pix.b==0) {
            pix.r = (int)(255*(fsum(x,y).r/pk2));
            pix.g = (int)(255*(fsum(x,y).g/pk2));
        }
    }

    conf = 0;
    if (pcx>=0) {
        addCircle(gen,PixelRgb(0,255,255),pcx,pcy,10);
        xout = (int)pcx;
        yout = (int)pcy;
        conf = 1;

        /*
        for (int i=-50; i<=50; i++) {
            int kk = i*4;
            addCircle(gen,PixelRgb(255,255,255),pcx+xvect*kk,pcy+yvect*kk,5);
        }
        */
    } else {
        addCircle(gen,PixelRgb(255,0,0),gen.width()/2, gen.height()/2,
                  gen.height()/2);
    }


    printf("%d patches\n", ct);
    if (saveMode) {
        write(gen,"gen.ppm");
    }
    dest = gen;
    IMGFOR(dest,x,y) {
        PixelRgb& o = dest(x,y);
        PixelRgb& i = img(x,y);
        PixelRgb& e = edge(x,y);
        if (o.r<30&&o.g<30&&o.b<30) {
            o.r = i.r/4;
            o.g = i.g/4;
            o.b = i.b/4;
        }  /*else {
            o.r /= 2;
            o.g /= 2;
            o.b /= 2;
            }*/
        if (e.r>0||e.g>0||e.b>0) {
            o = e;
        }
    }

    return 0;
}




int main(int argc, char *argv[]) {
    Property config;
    config.fromCommand(argc,argv);
    int xout, yout;
    double conf;

    capacity = config.check("capacity",Value(50)).asInt();

    if (config.check("notest")) {
        use_test = 0;
    }

    if (config.check("src")) {
        // read and process image from file
        ConstString fname = 
            config.check("src",Value("default.ppm"),
                         "the image to load").toString().c_str();
        ImageOf<PixelRgb> src;
        bool ok = read(src,fname.c_str());
        if (!ok) return 1;
        config.put("save_intermediate",1);
        ImageOf<PixelRgb> dest;
        return process(config,src,dest,xout,yout,conf);
    }


    // read and process images from a port

    ConstString image_port = 
        config.check("image_port",Value("/ellipse/img")).asString();
    ConstString pos_port = 
        config.check("pos_port",Value("/ellipse/pos")).asString();
    ConstString cam_port = 
        config.check("cam_port",Value("/pf/cam")).asString();
    ConstString view_port = 
        config.check("view_port",Value("/pf")).asString();
    ConstString carrier = 
        config.check("carrier",Value("mcast")).asString();

    printf("Welcome to axisTracker, a terrible, temporary hack.\n");
    printf("   capacity is %d (the bigger, the slower - and more accurate)\n", 
           capacity);
    printf("   image_port is %s\n", image_port.c_str());
    printf("   pos_port is %s\n", pos_port.c_str());
    printf("   cam_port is %s\n", cam_port.c_str());
    printf("   view_port is %s\n", view_port.c_str());
    printf("   carrier is %s\n", carrier.c_str());
    printf("Command line operation is possible using --src filename.ppm\n");
    printf("On slow machines, startup is slow (need to read a big file).\n");
    printf("Use \"yarp read /read %s\" to check the output format.\n", 
           pos_port.c_str());
    printf("Make sure the data directory is in the same place as your executable.\n");
    Time::delay(2);

    BufferedPort<ImageOf<PixelRgb> > port;
    BufferedPort<Bottle> posPort;
    port.open(image_port);
    posPort.open(pos_port);
    //Network::connect("/james/cam/right","/ellipse","mcast");
    Network::connect(cam_port,port.getName(),carrier);
    Network::connect(port.getName(),view_port,carrier);
    while (true) {
        ImageOf<PixelRgb> *src = port.read();
        if (src==NULL) {
            continue;
        }
        ImageOf<PixelRgb>& dest = port.prepare();
        process(config,*src,dest,xout,yout,conf);
        Bottle& bot = posPort.prepare();
        bot.clear();
        bot.addDouble(xout);
        bot.addDouble(yout);
        bot.addDouble(src->width());
        bot.addDouble(src->height());
        bot.addDouble(conf);
        posPort.write();
        port.write();
    }

    return 0;
}

