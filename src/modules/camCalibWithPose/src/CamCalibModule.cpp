// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/CamCalibModule.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>


#define USE_INERTIAL 0


using namespace std;
using namespace yarp::os;
using namespace yarp::math;
using namespace yarp::sig;

CamCalibPort::CamCalibPort()
{
    portImgOut=NULL;
    calibTool=NULL;

    verbose=false;

    filter_enable = 0;
    cf1 = 10.514;
    cf2 = 0.809;
    r_xv[0]=r_xv[1]=0;
    p_xv[0]=p_xv[1]=0;
    y_xv[0]=y_xv[1]=0;
    r_yv[0]=r_yv[1]=0;
    p_yv[0]=p_yv[1]=0;
    y_yv[0]=y_yv[1]=0;

    t0=Time::now();
}

void CamCalibPort::setPointers(yarp::os::Port *_portImgOut, ICalibTool *_calibTool)
{
    portImgOut=_portImgOut;
    calibTool=_calibTool;
}

void CamCalibPort::setSaturation(double satVal)
{
    currSat = satVal;
}

void CamCalibPort::onRead(ImageOf<PixelRgb> &yrpImgIn)
{
    double t=Time::now();

    yarp::os::Stamp s;
    this->getEnvelope(s);
    double time = s.getTime();

    if (!updatePose(time)) {
        return;
    }


    unsigned char *pixel = yrpImgIn.getPixelAddress(0, 0);
    double *stamp = reinterpret_cast<double*>(pixel);
    double backdoorTime = stamp[0];
    double backdoorRoll = stamp[1];
    double backdoorPitch = stamp[2];
    double backdoorYaw = stamp[3];

    if (time != backdoorTime) {
        yWarning() << "Backdoor time:" << backdoorTime << "Imu time:" << time << "diff:" << (backdoorTime - time);
    }

    Bottle& b = rpyPort.prepare();
    b.clear();
    b.addDouble(roll);
    b.addDouble(pitch);
    b.addDouble(yaw);
    b.addDouble(backdoorRoll);
    b.addDouble(backdoorPitch);
    b.addDouble(backdoorYaw);
    b.addDouble(backdoorRoll - roll);
    b.addDouble(backdoorPitch - pitch);
    b.addDouble(backdoorYaw - yaw);
    rpyPort.write();

#if 0
    yarp::sig::Vector roll_vector(3);
    roll_vector(0) = roll/180.0*M_PI;
    roll_vector(1) = 0;
    roll_vector(2) = 0;
    yarp::sig::Matrix roll_dcm = yarp::math::rpy2dcm(roll_vector);

    yarp::sig::Vector pitch_vector(3);
    pitch_vector(0) = 0;
    pitch_vector(1) = pitch/180.0*M_PI;
    pitch_vector(2) = 0;
    yarp::sig::Matrix pitch_dcm = yarp::math::rpy2dcm(pitch_vector);

    yarp::sig::Vector yaw_vector(3);
    yaw_vector(0) = 0;
    yaw_vector(1) = 0;
    yaw_vector(2) = yaw/180.0*M_PI;
    yarp::sig::Matrix yaw_dcm = yarp::math::rpy2dcm(yaw_vector);

    yarp::sig::Matrix dcm = (yaw_dcm * pitch_dcm) * roll_dcm;

    yarp::sig::Vector r_roll_vector(3);
    r_roll_vector(0) = backdoorRoll/180.0*M_PI;
    r_roll_vector(1) = 0;
    r_roll_vector(2) = 0;
    yarp::sig::Matrix r_roll_dcm = yarp::math::rpy2dcm(r_roll_vector);

    yarp::sig::Vector r_pitch_vector(3);
    r_pitch_vector(0) = 0;
    r_pitch_vector(1) = backdoorPitch/180.0*M_PI;
    r_pitch_vector(2) = 0;
    yarp::sig::Matrix r_pitch_dcm = yarp::math::rpy2dcm(r_pitch_vector);

    yarp::sig::Vector r_yaw_vector(3);
    r_yaw_vector(0) = 0;
    r_yaw_vector(1) = 0;
    r_yaw_vector(2) = backdoorYaw/180.0*M_PI;

    yarp::sig::Matrix r_yaw_dcm = yarp::math::rpy2dcm(r_yaw_vector);

    yarp::sig::Matrix r_dcm = (r_yaw_dcm * r_pitch_dcm) * r_roll_dcm;


    printf("backdoor                  imu                       diff\n");
    for(int i = 0; i <= 2; ++i) {
        for(int j = 0; j <= 2; ++j) {
            printf("%+03.3f ", r_dcm(i,j));
        }
        printf("     ");
        for(int j = 0; j <= 2; ++j) {
            printf("%+03.3f ", dcm(i,j));
        }
        printf("     ");
        for(int j = 0; j <= 2; ++j) {
            double diff = dcm(i,j) - r_dcm(i,j);
            bool err = ((diff >= 0.2) || (diff <= -0.2));
            bool warn = ((diff >= 0.05) || (diff <= -0.05));
            printf("%s%+03.3f%s ", (err ? "\033[0;31m" : (warn ? "\033[0;33m" : "")), diff, ((err||warn) ? "\033[0m" : ""));
        }
        printf("\n");
    }
    printf("\n--------------------------------------\n");



    if (roll != backdoorRoll) {
        yWarning() << "backdoor roll:" << backdoorRoll << "Imu roll:" << roll << "diff:" << (backdoorRoll - roll);
    }

    if (pitch != backdoorPitch) {
        yWarning() << "backdoor pitch:" << backdoorPitch << "Imu pitch:" << pitch << "diff:" << (backdoorPitch - pitch);
    }

    if (yaw != backdoorYaw) {
        yWarning() << "backdoor yaw:" << backdoorYaw << "Imu yaw:" << yaw << "diff:" << (backdoorYaw - yaw);
    }
#endif



    // execute calibration
    if (portImgOut!=NULL)
    {        
        yarp::sig::ImageOf<PixelRgb> yrpImgOut;

        if (verbose)
            yDebug("received input image after %g [s] ... ",t-t0);

        double t1=Time::now();

        if (calibTool!=NULL)
        {
            calibTool->apply(yrpImgIn,yrpImgOut);

            for (int r =0; r <yrpImgOut.height(); r++)
            {
                for (int c=0; c<yrpImgOut.width(); c++)
                {
                    unsigned char *pixel = yrpImgOut.getPixelAddress(c,r);
                    double mean = (1.0/3.0)*(pixel[0]+pixel[1]+pixel[2]);

                    for(int i=0; i<3; i++)
                    {
                        double s=pixel[i]-mean;
                        double sn=currSat*s;
                        sn+=mean;

                        if(sn<0.0)
                            sn=0.0;
                        else if(sn>255.0)
                            sn=255.0;

                        pixel[i]=(unsigned char)sn;
                    }
                }
            }

            if (verbose)
                yDebug("calibrated in %g [s]\n",Time::now()-t1);
        }
        else
        {
            yrpImgOut=yrpImgIn;

            if (verbose)
                yDebug("just copied in %g [s]\n",Time::now()-t1);
        }

        m.lock();
        
        //timestamp propagation
        //yarp::os::Stamp stamp;
        //BufferedPort<ImageOf<PixelRgb> >::getEnvelope(stamp);
        //portImgOut->setEnvelope(stamp);

        yDebug() << "ROLL =" << roll;
        Bottle pose;
        pose.addDouble(roll);
        pose.addDouble(pitch);
        pose.addDouble(yaw);
        portImgOut->setEnvelope(pose);

        portImgOut->write(yrpImgOut);
        m.unlock();
    }

    t0=t;
}



bool CamCalibPort::selectBottleFromMap(double time,
                                       std::map<double, yarp::os::Bottle> *datamap,
                                       yarp::os::Bottle *bottle,
                                       bool verbose)
{
    if (datamap->size() == 0) {
        return false;
    }

    std::map<double, yarp::os::Bottle>::iterator it_prev, it_next;

    m.lock();
    it_next = datamap->lower_bound(time);

    // wait until we receive a sample with a time greater than image time
    while(it_next->first < time) {
        m.unlock();
        yarp::os::Time::delay(0.001);
        m.lock();
        it_next = datamap->lower_bound(time);
    }

    it_prev = it_next;
    if(it_prev != datamap->begin()) {
        --it_prev;
    }
    if(it_next == datamap->end() && it_next != datamap->begin()) {
        --it_next;
    }

    double diff_prev = time - it_prev->first;
    double diff_next = it_next->first - time;
    double diff = (diff_prev >= diff_next) ? diff_next : diff_prev;

    if (verbose) {
        std::map<double, yarp::os::Bottle>::iterator it_begin, it_end;

        it_end = datamap->end();
        if(it_end != datamap->begin()) {
            --it_end;
        }
        it_begin = datamap->begin();

        int count_less = 0;
        int count_more = 0;
        for(std::map<double, yarp::os::Bottle>::iterator it = datamap->begin(); it != datamap->end(); ++it) {
            if (it->first > time) {
                ++count_more;
            } else {
                ++count_less;
            }
        }

        double diff_end = it_end->first - time;
        double diff_begin = time - it_begin->first;
        bool err_prev = ((diff_prev >= 0.0025) || (diff_prev <= -0.0025));
        bool warn_prev = ((diff_prev >= 0.0015) || (diff_prev <= -0.0015));
        bool err_next = ((diff_next >= 0.0025) || (diff_next <= -0.0025));
        bool warn_next = ((diff_next >= 0.0015) || (diff_next <= -0.0015));
        bool err_end = ((diff_end >= 0.0025) || (diff_end <= -0.0025));
        bool warn_end = ((diff_end >= 0.0015) || (diff_end <= -0.0015));
        bool err_begin = ((diff_begin >= 0.0025) || (diff_begin <= -0.0025));
        bool warn_begin = ((diff_begin >= 0.0015) || (diff_begin <= -0.0015));
        bool err = ((diff >= 0.0025) || (diff <= -0.0025));
        bool warn = ((diff >= 0.0015) || (diff <= -0.0015));

        printf("%f, %f, %s%f%s, %d, %f, %s%f%s, %f, %s%f%s, %f, %s%f%s, %d, %s%f%s, %s%zd%s    %s\n",
               time,
               it_begin->first,
               (err_begin ? "\033[0;31m" : (warn_begin ? "\033[0;33m" : "")), diff_begin, ((err_begin||warn_begin) ? "\033[0m" : ""),
               count_less,
               it_prev->first,
               (err_prev ? "\033[0;31m" : (warn_prev ? "\033[0;33m" : "")), diff_prev, ((err_prev||warn_prev) ? "\033[0m" : ""),
               it_next->first,
               (err_next ? "\033[0;31m" : (warn_next ? "\033[0;33m" : "")), diff_next, ((err_next||warn_next) ? "\033[0m" : ""),
               it_end->first,
               (err_end ? "\033[0;31m" : (warn_end ? "\033[0;33m" : "")), diff_end, ((err_end||warn_end) ? "\033[0m" : ""),
               count_more,
               (err ? "\033[0;31m" : (warn ? "\033[0;33m" : "")), diff, ((err||warn) ? "\033[0m" : ""),
               ((datamap->size() <= 10) ? "\033[0;31m" : ((datamap->size() <= 15) ? "\033[0;33m" : "")), datamap->size(), ((datamap->size() <= 15) ? "\033[0m" : ""),
               ((diff > maxDelay) ? "\033[0;31mSKIPPED\033[0m" : "OK"));
    }

    if (diff > maxDelay) {
        m.unlock();
        return false;
    }

    if (0) {
        yDebug() << "RIGHT EYE! UNA MERDA";
        *bottle = (diff_prev >= diff_next) ? it_next->second : it_prev->second;
    } else {
        yDebug() << "LEFT EYE! TROPPO FIGO";


        // Select previous
//        *bottle = it_prev->second;

        // Select next
//        *bottle = it_next->second;

        // Select closest
//    *bottle = (diff_prev >= diff_next) ? it_next->second : it_prev->second;


        bottle->clear();
        for(int i = 0; i < it_prev->second.size(); ++i) {
            if(i < 3) {
                double x0 = it_prev->second.get(i).asDouble();
                double x1 = it_next->second.get(i).asDouble();
                double t0 = it_prev->first;
                double t1 = it_next->first;
                double tx = time;
                double xx = 0;

                // Linear interpolation
                xx = x0 + (tx - t0)*(x1 - x0)/(t1 - t0);

#if USE_INERTIAL
                double v0 = it_prev->second.get(i+6).asDouble();
                double v1 = it_next->second.get(i+6).asDouble();
                double a = (v1 - v0) / (t1 - t0);

                if (fabs(v1 - v0) > 30) {
                    m.unlock();
                    return false;
               //     if (leftEye) {
                     //   v0 = (fabs(v1) > fabs(v0)) ? xx = x0 + (tx - t0) * v0
                     //                              : xx = x1 - (t1 - tx) * v1;
                //    } else {
                  //      xx = x0 + (tx - t0) * v0;
                    //}
                } else {

                // x + vt
                xx = x0 + (tx - t0) * v0;

                // best
            //    xx = (diff_prev >= diff_next) ? (x1 - (t1 - tx) * v1)
            //                                  : (x0 + (tx - t0) * v0);

                // mean
            //    xx = ((x0 + (tx - t0) * v0) + (x1 - (t1 - tx) * v1)) / 2;



                // x + vt + 1/2at^2
            //    xx = x0 + (tx - t0) * v0 + a / 2 * pow((tx - t0), 2);
            //    xx = x1 - (t1 - tx) * v1 - a / 2 * pow((t1 - tx), 2);
            //    xx = (diff_prev >= diff_next) ? x1 - (t1 - tx) * v1 - a / 2 * pow((t1 - tx), 2)
            //                                  : x0 + (tx - t0) * v0 + a / 2 * pow((tx - t0), 2);
            }
#endif

                bottle->addDouble(xx);
            } else {
                bottle->add(it_prev->second.get(i));
            }
        }
    } // left eye


    if (it_prev != datamap->begin()) {
        datamap->erase(datamap->begin(), it_prev);
    }
    m.unlock();

    return true;
}


bool CamCalibPort::updatePose(double time)
{
    if (!selectBottleFromMap(time, &h_encs_map, &h_encs, true)) {
#if !USE_INERTIAL
        return false;
#endif
    }
    if (!selectBottleFromMap(time, &imu_map, &imu, false)) {
#if USE_INERTIAL
        return false;
#endif
    }

    double ix = -h_encs.get(1).asDouble()/180.0*M_PI; // neck roll
    double iy = h_encs.get(0).asDouble()/180.0*M_PI;  // neck pitch
    double iz = h_encs.get(2).asDouble()/180.0*M_PI;  // neck yaw

    double t =  h_encs.get(3).asDouble()/180.0*M_PI;  // eye tilt
    double vs = h_encs.get(4).asDouble()/180.0*M_PI;  // eye version
    double vg = h_encs.get(5).asDouble()/180.0*M_PI;  // eye vergence

    double imu_x = imu.get(0).asDouble()/180.0*M_PI; // imu roll
    double imu_y = imu.get(1).asDouble()/180.0*M_PI; // imu pitch
    double imu_z = imu.get(2).asDouble()/180.0*M_PI; // imu yaw

    yDebug() << ix << iy << iz;

    yarp::sig::Vector neck_roll_vector(3);
    neck_roll_vector(0) = ix;
    neck_roll_vector(1) = 0;
    neck_roll_vector(2) = 0;
    yarp::sig::Matrix neck_roll_dcm = yarp::math::rpy2dcm(neck_roll_vector);

    yarp::sig::Vector neck_pitch_vector(3);
    neck_pitch_vector(0) = 0;
    neck_pitch_vector(1) = iy;
    neck_pitch_vector(2) = 0;
    yarp::sig::Matrix neck_pitch_dcm = yarp::math::rpy2dcm(neck_pitch_vector);

    yarp::sig::Vector neck_yaw_vector(3);
    neck_yaw_vector(0) = 0;
    neck_yaw_vector(1) = 0;
    neck_yaw_vector(2) = iz;
    yarp::sig::Matrix neck_yaw_dcm = yarp::math::rpy2dcm(neck_yaw_vector);

    yarp::sig::Matrix neck_dcm = (neck_pitch_dcm * neck_roll_dcm) * neck_yaw_dcm;


    yarp::sig::Vector eye_pitch_vector(3);
    eye_pitch_vector(0) = 0;
    eye_pitch_vector(1) = t;
    eye_pitch_vector(2) = 0;
    yarp::sig::Matrix eye_pitch_dcm = yarp::math::rpy2dcm(eye_pitch_vector);

    yarp::sig::Vector eye_yaw_vector(3);
    eye_yaw_vector(0) = 0;
    eye_yaw_vector(1) = 0;
    eye_yaw_vector(2) = 0;
    if (leftEye) {
        eye_yaw_vector(2) = -(vs + vg/2);
    } else {
        eye_yaw_vector(2) = -(vs - vg / 2);
    }
    yarp::sig::Matrix eye_yaw_dcm = yarp::math::rpy2dcm(eye_yaw_vector);

    yarp::sig::Matrix eye_dcm = eye_pitch_dcm * eye_yaw_dcm;

    yarp::sig::Matrix T = neck_dcm * eye_dcm;

#if !USE_INERTIAL
    yarp::sig::Vector v = yarp::math::dcm2rpy(T);
#endif

/*
    yarp::sig::Vector imu_roll_vector(3);
    imu_roll_vector(0) = imu_x;
    imu_roll_vector(1) = 0;
    imu_roll_vector(2) = 0;
    yarp::sig::Matrix imu_roll_dcm = yarp::math::rpy2dcm(imu_roll_vector);

    yarp::sig::Vector imu_pitch_vector(3);
    imu_pitch_vector(0) = 0;
    imu_pitch_vector(1) = imu_y;
    imu_pitch_vector(2) = 0;
    yarp::sig::Matrix imu_pitch_dcm = yarp::math::rpy2dcm(imu_pitch_vector);

    yarp::sig::Vector imu_yaw_vector(3);
    imu_yaw_vector(0) = 0;
    imu_yaw_vector(1) = 0;
    imu_yaw_vector(2) = imu_z;
    yarp::sig::Matrix imu_yaw_dcm = yarp::math::rpy2dcm(imu_yaw_vector);
*/

//  Aeronautic convention (iCub)
//    yarp::sig::Matrix imu_dcm = (imu_yaw_dcm * imu_roll_dcm) * imu_pitch_dcm;

//  Robotic convention (gazebo)
//    yarp::sig::Matrix imu_dcm = (imu_yaw_dcm * imu_pitch_dcm) * imu_roll_dcm;

#if USE_INERTIAL
//    yarp::sig::Vector v = yarp::math::dcm2rpy(imu_dcm);
//    v = yarp::math::dcm2rpy(imu_dcm);
#endif

#if 0
    printf("\n--------------------------------------\n");
    printf ("%+03.3f %+03.3f %+03.3f", neck_roll_vector(0)*180.0/M_PI, neck_pitch_vector(1)*180.0/M_PI, neck_yaw_vector(2)*180.0/M_PI);
    printf ("  >>>>>>>> %+03.3f %+03.3f %+03.3f", v(0)*180.0/M_PI,v(1)*180.0/M_PI,v(2)*180.0/M_PI);
    printf ("  >>>>>>>> %+03.3f %+03.3f %+03.3f", imu_roll_vector(0)*180.0/M_PI, imu_pitch_vector(1)*180.0/M_PI, imu_yaw_vector(2)*180.0/M_PI);

    printf("\n--------------------------------------\n");
    printf("encoders                  imu                       diff\n");
    for(int i = 0; i <= 2; ++i) {
        for(int j = 0; j <= 2; ++j) {
            printf("%+03.3f ", neck_dcm(i,j));
        }
        printf("     ");
        for(int j = 0; j <= 2; ++j) {
            printf("%+03.3f ", imu_dcm(i,j));
        }
        printf("     ");
        for(int j = 0; j <= 2; ++j) {
            double diff = neck_dcm(i,j) - imu_dcm(i,j);
            bool err = ((diff >= 0.2) || (diff <= -0.2));
            bool warn = ((diff >= 0.05) || (diff <= -0.05));
            printf("%s%+03.3f%s ", (err ? "\033[0;31m" : (warn ? "\033[0;33m" : "")), diff, ((err||warn) ? "\033[0m" : ""));
        }
        printf("\n");
    }
    printf("\n--------------------------------------\n");
#endif

    yDebug() << v(0);

    roll = v(0)*180.0/M_PI;
    pitch = v(1)*180.0/M_PI;
    yaw = v(2)*180.0/M_PI;

//    yDebug() << "IMU_X =" << imu_x;
//    roll = imu_x*180.0/M_PI;
//    pitch = imu_y*180.0/M_PI;
//    yaw = imu_z*180.0/M_PI;


    if (filter_enable) {
//        if (yaw<0) yaw=yaw+360;


         r_xv[0] = r_xv[1];
//         printf ("%f %f ",   r_xv[0] , r_xv[1]);
         r_xv[1] = roll / cf1;
//         printf ("%f %f %f ",   r_xv[1] , r_off, cf1 );
         r_yv[0] = r_yv[1];
//         printf ("%f %f %f ",   r_yv[0] , r_yv[1], cf2 );
         r_yv[1] =   (r_xv[0] + r_xv[1]) + ( cf2 * r_yv[0]);
         roll = r_yv[1];
//         printf ("%f\n", r_off);

         p_xv[0] = p_xv[1];
         p_xv[1] = pitch / cf1;
         p_yv[0] = p_yv[1];
         p_yv[1] =   (p_xv[0] + p_xv[1]) + ( cf2 * p_yv[0]);
         pitch = p_yv[1];

         y_xv[0] = y_xv[1];
         y_xv[1] = yaw / cf1;
         y_yv[0] = y_yv[1];
         y_yv[1] =   (y_xv[0] + y_xv[1]) + ( cf2 * y_yv[0]);
         yaw = y_yv[1];
    } else {
        r_xv[0]=r_xv[1]=0;
        p_xv[0]=p_xv[1]=0;
        y_xv[0]=y_xv[1]=0;
        r_yv[0]=r_yv[1]=0;
        p_yv[0]=p_yv[1]=0;
        y_yv[0]=y_yv[1]=0;
 }

    return true;
}


CamCalibModule::CamCalibModule(){

    _calibTool = NULL;	
}

CamCalibModule::~CamCalibModule(){

}

bool CamCalibModule::configure(yarp::os::ResourceFinder &rf){

    ConstString str = rf.check("name", Value("/camCalib"), "module name (string)").asString();
    setName(str.c_str()); // modulePortName

    double maxDelay = rf.check("maxDelay", Value(0.010), "Max delay between image and encoders").asDouble();

    // pass configuration over to bottle
    Bottle botConfig(rf.toString().c_str());
    botConfig.setMonitor(rf.getMonitor());		
    // Load from configuration group ([<group_name>]), if group option present
    Value *valGroup; // check assigns pointer to reference
    if(botConfig.check("group", valGroup, "Configuration group to load module options from (string)."))
    {
        strGroup = valGroup->asString().c_str();        
        // is group a valid bottle?
        if (botConfig.check(strGroup.c_str())){            
            Bottle &group=botConfig.findGroup(strGroup.c_str(),string("Loading configuration from group " + strGroup).c_str());
            botConfig.fromString(group.toString());
        }
        else{
            yError() << "Group " << strGroup << " not found.";
            return false;
        }
    }
    else
    {
        yError ("There seem to be an error loading parameters (group section missing), stopping module");
        return false;
    }

    string calibToolName = botConfig.check("projection",
                                         Value("pinhole"),
                                         "Projection/mapping applied to calibrated image [projection|spherical] (string).").asString().c_str();

    _calibTool = CalibToolFactories::getPool().get(calibToolName.c_str());
    if (_calibTool!=NULL) {
        bool ok = _calibTool->open(botConfig);
        if (!ok) {
            delete _calibTool;
            _calibTool = NULL;
            return false;
        }
    }

    if (yarp::os::Network::exists(getName("/in")))
    {
        yWarning() << "port " << getName("/in") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/out")))
    {
        yWarning() << "port " << getName("/out") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/conf")))
    {
        yWarning() << "port " << getName("/conf") << " already in use";
    }
    _prtImgIn.setSaturation(rf.check("saturation",Value(1.0)).asDouble());
    _prtImgIn.open(getName("/in"));
    _prtImgIn.setPointers(&_prtImgOut,_calibTool);
    _prtImgIn.setVerbose(rf.check("verbose"));
    _prtImgIn.setLeftEye((strGroup == "CAMERA_CALIBRATION_LEFT") ? true : false);
    _prtImgIn.setMaxDelay(maxDelay);
    _prtImgIn.useCallback();
    _prtImgOut.open(getName("/out"));
    _configPort.open(getName("/conf"));
    _prtHEncsIn.open(getName("/head_encs/in"));
    _prtHEncsIn._prtImgIn = &_prtImgIn;
//    _prtHEncsIn.setStrict();
    _prtHEncsIn.useCallback();
    _prtTEncsIn.open(getName("/torso_encs/in"));
//    _prtTEncsIn.setStrict();
    _prtImuIn.open(getName("/imu/in"));
    _prtImuIn._prtImgIn = &_prtImgIn;
//    _prtImuIn.setStrict();
    _prtImuIn.useCallback();
    attach(_configPort);
    fflush(stdout);

    _prtImgIn.rpyPort.open(getName("/rpy"));

    return true;
}

bool CamCalibModule::close(){
    _prtImgIn.close();
	_prtImgOut.close();
    _prtHEncsIn.close();
    _prtTEncsIn.close();
    _prtImuIn.close();
    _configPort.close();
    if (_calibTool != NULL){
        _calibTool->close();
        delete _calibTool;
        _calibTool = NULL;
    }
    return true;
}

bool CamCalibModule::interruptModule(){
    _prtImgIn.interrupt();
    _prtImgOut.interrupt();
    _configPort.interrupt();
    _prtHEncsIn.interrupt();
    _prtImuIn.interrupt();
    return true;
}

void HeadEncoderPort::onRead(yarp::os::Bottle &h_encs) {
    yarp::os::Stamp s;
    this->getEnvelope(s);
    double time = s.getTime();
    if(time !=0) {
        _prtImgIn->setHeadEncoders(time, h_encs);
    }
}

void ImuPort::onRead(yarp::os::Bottle &imu) {
    yarp::os::Stamp s;
    this->getEnvelope(s);
    double time = s.getTime();
    if(time !=0) {
        _prtImgIn->setImuData(time, imu);
    }
}

bool CamCalibModule::updateModule()
{
    Bottle* t_encs = _prtTEncsIn.read(false); //torso encoders

    if (t_encs!=NULL) {
        yarp::os::Stamp s;
        _prtTEncsIn.getEnvelope(s);
        double time = s.getTime();
        if(time !=0) {
            _prtImgIn.setTorsoEncoders(time, *t_encs);
        }
    }

    return true;
}

double CamCalibModule::getPeriod() {
  return 0.001;
}

bool CamCalibModule::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).asString()=="quit") 
    {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="sat" || command.get(0).asString()=="saturation")
    {
        double satVal = command.get(1).asDouble();
        _prtImgIn.setSaturation(satVal);
        
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="filt")
    {
        _prtImgIn.filter_enable = command.get(1).asInt();
    }
    else if (command.get(0).asString()=="cf1")
    {
         _prtImgIn.cf1 = command.get(1).asDouble();
    }
    else if (command.get(0).asString()=="cf2")
    {
        _prtImgIn.cf2 = command.get(1).asDouble();
    }

    else
    {
        yError() << "command not known - type help for more info";
    }
    return true;
}

