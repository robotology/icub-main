// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#include "stereovision.h"

/*
// mouse callback functions
#ifdef OPENCV_9_5
void on_mouse0( int event, int x, int y, int flags);//, void* param );
void on_mouse1( int event, int x, int y, int flags);
#else
void on_mouse0( int event, int x, int y, int flags, void* param );
void on_mouse1( int event, int x, int y, int flags, void* param );
#endif

void on_mouse( int event, int x, int y, int flags, int index);


// trackbar callback functions
void update_sigma0(int slider);
void update_sigma1(int slider);
void update_sigma(int slider,int window);

*/


Stereovision::Stereovision(int argc, char *argv[]){
    diff =0;
    finish=0;
    track=0;
    locate=0;
    comm = 0;
    kalman=0;
    nb_obj=1;
    skip = MAX_OBJECTS;
    save3d=0;
    loc=NULL;
    for(int i=0;i<MAX_CAM;i++){
        dc[i] = NULL;
        grabber[i]=NULL;
        blob[i] = NULL;
        gmm[i] = NULL;
        for(int j=0;j<MAX_OBJECTS;j++){
            colorfinder[i][j] = NULL; 
        }
    }
    char tmp[30];
    for(int j=0;j<MAX_OBJECTS;j++){
        sprintf(tmp,"/vision%d:o",j);
        port[j].SetPortName(tmp);
    }
}


Stereovision::~Stereovision()
{
    for(int i=0;i<MAX_CAM;i++){
        for(int j=0;j<MAX_OBJECTS;j++){
            if(colorfinder[i][j])delete colorfinder[i][j];
        }
        if(grabber[i])delete grabber[i];
        if(gmm[i])    delete gmm[i];
        if(blob[i])   delete blob[i];
        if(dc[i])     delete dc[i];
    }
    if(kalman){
        for(j=0;j<nb_obj;j++){
            if(kal[j])    delete kal[j];
        }
    }
    if(loc)           delete loc;
    for(int j=0;j<MAX_OBJECTS;j++){
        port[j].Stop();
    }
}

void Stereovision::InitVision(const char *paramdir){
    InitCameras();
    if(paramdir){
        sprintf(stereoparamfile,"params/%s/params3d",paramdir);
    }else{
        sprintf(stereoparamfile,"params/params3d");
    }
  
    loc = new Locator3D(stereoparamfile);

    for(int i=0;i<MAX_CAM;i++){
        if(!InitGrabber(i))cout<<"cannot initialize frame grabber "<<i<<endl;
        if(paramdir){
            sprintf(paramfile[i],"params/%s/params%d",paramdir,i+1);
        }else{
            sprintf(paramfile[i],"params/params%d",i+1);
        }

        dc[i]= new DistortionCorrector(paramfile[i]);

        dc[i]->SetGrabber(grabber[i]); 
        dc[i]->SetChessboardParams(5,7,20);
        //    dc[i]->CalibrateCamera(5);
        grabber[i]->GrabFrame(&(img[i]),0);
        tmp1_img[i] = cvCloneImage(img[i]);
        tmp2_img[i] = cvCloneImage(img[i]);
   
        sprintf(wname[i],"window %d",i);
        cvNamedWindow(wname[i],CV_WINDOW_AUTOSIZE);   
        //  cvMoveWindow(wname[i],10+(img[i]->width+13)*i,300); not supported

        colorfinder[i][0] = new ColorTracker();
        colorfinder[i][0]->SetGrabber(grabber[i]);
        colorfinder[i][0]->SetSigma(3);
        trackbar[i]=6;
        sprintf(tbname[i],"color sigma %d",i);
        i?cvCreateTrackbar((const char*)tbname[i],wname[i],&(trackbar[i]),10,update_sigma1):
            cvCreateTrackbar((const char*)tbname[i],wname[i],&(trackbar[i]),10,update_sigma0) ;


        mparam[i].finder =  colorfinder[i][0];
        mparam[i].grabber = grabber[i];
     
        i?cvSetMouseCallback(wname[i],on_mouse1):cvSetMouseCallback(wname[i],on_mouse0);
     
        blob[i] = new BlobTracker(MAX_OBJECTS);
        blob[i]->AddBlob(cvRect(0,0,img[i]->width,img[i]->height));
        gmm[i] = new GmmTracker(cvGetSize(img[i]),2);
        gmm[i]->Randomize();
    }
}




void Stereovision::InitCameras(){
#ifndef ICUB
#ifdef LINUX
    CameraV4L::createCameras((CameraV4L***) &cams,&nbcams);//only for linux, 
    for(int i=0;i<nbcams;i++){
        cams[i]->open();
    }
#endif
#endif
}

bool Stereovision::InitGrabber(int i){
#ifdef LINUX
    grabber[i]= new CameraImage(cams[i],640,480); //only for linux, use cvcam with windows
#else
    if(i<= CvcamGrabber.GetCamerasCount();){
        grabber[i]= new CvcamGrabber();
    }
#endif
    return grabber[i]!=NULL;
}

void Stereovision::Run()
{ 
    while(!finish){
        ProcessNextFrame();
#ifndef NO_WAIT_KEY
        int key =cvWaitKey(10);
        ExecuteCommand(key);
#endif
    }  
} 

void Stereovision::ProcessNextFrame(){
    int i,j;
    for(i=0;i<MAX_CAM;i++){
        grabber[i]->GrabFrame(&(img[i]),0);    
    }
    for(i=0;i<MAX_CAM;i++){
        if(diff){
            cvCopy(img[i],tmp1_img[i]);
        }
        dc[i]->Apply(img[i]);	
        if(diff){
            cvSub(img[i],tmp1_img[i],tmp2_img[i]);
            cvShowImage(diff_name[i],tmp2_img[i]);  
        }
        if(track){
            for(j=0;j<nb_obj;j++){
                if(j!=skip){
                    colorfinder[i][j]->Apply(img[i]);
                    if(j==ColorTracker::displayed_color){
                        colorfinder[i][j]->DrawMask(img[i]);
                    }
                    blob[i]->Apply(colorfinder[i][j]->GetMask(),j);
                    if(!blob[i]->Found(j)){
                        gmm[i]->Randomize();
                        gmm[i]->Apply(colorfinder[i][j]->GetMask());
                        CvPoint co = gmm[i]->GetCoords(0);
                        blob[i]->SetWindowPosition(j,vec2(trim(co.x,0,img[i]->width-5),
                                                          trim(co.y,0,img[i]->height-5)),vec2(5,5));
                    }
                }
            }
            blob[i]->DrawBlobs(img[i]);	 
        }
        cvShowImage(wname[i],img[i]);
    }
    if(locate){
        if(loc->IsCalibrated()){
            found = 0;
            for(j=0;j<nb_obj;j++){
                found3d[j]=0;
                if(j!=skip){
                    if(blob[0]->Found(j) && blob[1]->Found(j)){
                        for(i=0;i<2;i++){
                            point2d[i] = blob[i]->GetCoords(j);
                        }
                        loc->Locate(1,&(point2d[0]),&(point2d[1]),&(point3d[j]),
                                    &(found3d[j]));
                        found = found || found3d[j];
                        if(kalman){
                            if(found3d[j]){
                                point3df[j] = kal[j]->PredictAndCorrect(point3d[j]);
                            }
                        }
                    }
                }
            }
            if(found){
                if(comm){
                    //    port.SendPosition(point3d.x,point3d.y,point3d.z);
                    for(int k=0;k<nb_obj;k++){
                        position[k*3]= kalman? point3df[k].x:point3d[k].x;
                        position[k*3+1]= kalman? point3df[k].y:point3d[k].y;
                        position[k*3+2]= kalman? point3df[k].z:point3d[k].z;
                        if(found3d[k]){
                            //  port[k].SendPosition(position[3*k],position[3*k+1],position[3*k+2]);
                            SendOutput(&(port[k]),&(position[3*k]));
                        }
                    }
                    //                     port.SendPosition(position,found3d,nb_obj);
                }
            }
        }
        else{
            cout<<"calibrating stereovision..."<<endl;
            const CvCamera *cam[2];
            const double *chess_p = dc[0]->GetChessboardParams();
            cam[0] =  dc[0]->GetCameraParams();
            cam[1] =  dc[1]->GetCameraParams();
            while(!loc->IsCalibrated()){
                for(i=0;i<2;i++){
                    cvShowImage(wname[i],img[i]);
                    grabber[i]->GrabFrame(&(img[i]),0);
                }//maybe it should not be -1
                loc->Calibrate(img,cam,cvSize((int)chess_p[0]-1,(int)chess_p[1]-1),chess_p[2]); 
                if('q' == cvWaitKey(5)){
                    cout<<"exiting calibration"<<endl;
                    break;
                }
            }
        }
        if(save3d==1){
            if(loc->SaveParams(stereoparamfile)){
                cout<<"stereo vision parameter saved in "<<stereoparamfile<<endl;
            }
            save3d=0;
        }
    }//if(locate)
}

void Stereovision::SendOutput(VisionServer *port, double *position){
    port->SendPosition(position[0],position[1],position[2]);
}

void Stereovision::ExecuteCommand(int key)
{
    char fname[80];
    char tmpstr[80];
    int i,j;
    switch(key){

    case 'k': // adds kalman filtering on 3d position
        if((kalman = 1-kalman)){
            cout<< "kalman filtering"<<endl;
            for(i=0;i<nb_obj;i++){
                kal[i] = new CKalman(true,5.0f);
            }
        }
        else{
            cout<< "stopping kalman filtering"<<endl;
            for(i=0;i<nb_obj;i++){
                delete kal[i];
            }
        }
        break;
    case 'M':
        if(kalman){
            cout<<"measurement noise "<<kal[ColorTracker::displayed_color]->IncrementNoise()<<endl;
        }
        break;
    case 'm':
        if(kalman){
            cout<<"measurement noise "<<kal[ColorTracker::displayed_color]->DecrementNoise()<<endl;
        }
        break;
    case 'P':
        if(kalman){
            cout<<"process noise "<<kal[ColorTracker::displayed_color]->IncrementProcessNoise()<<endl;
        }
        break;
    case 'p':
        if(kalman){
            cout<<"process noise "<<kal[ColorTracker::displayed_color]->DecrementProcessNoise()<<endl;
        }
        break;

    case 'r':// remove one color (stop tracking one color)
        if(skip == MAX_OBJECTS){
            skip = ColorTracker::displayed_color;      
            cout <<"stop tracking object "<< ColorTracker::displayed_color<<endl;
        }
        else{
            cout <<"resuming tracking object "<< skip<<endl;
            skip = MAX_OBJECTS;
        }
        break;
    case 'a': //adds a color tracker
        if(nb_obj<MAX_OBJECTS){
            for(i=0;i<MAX_CAM;i++){
                colorfinder[i][nb_obj] = new ColorTracker();
                colorfinder[i][nb_obj]->SetGrabber(grabber[i]);
                blob[i]->AddBlob(cvRect(0,0,img[i]->width,img[i]->height));
	 
            }
            ColorTracker::displayed_color = nb_obj;
            if(kalman){
                kal[nb_obj] = new CKalman(true,5.0f);
            }
            if(comm){
                port[nb_obj].Start();
            }
            nb_obj++;
            cout << "up to "<<nb_obj<<" objects can be tracked"<<endl;
        }
        break;
    case 's':
        cout<<"enter color file name:"<<endl;
        cin>>fname;
        for(i=0;i<MAX_CAM;i++){
            for(j=0;j<nb_obj;j++){
                sprintf(tmpstr,"%s%d%d",fname,i,j);
                colorfinder[i][j]->Save(tmpstr);
            }
        }
        cout<<"color parameters saved"<<endl;
        break;
    case 'g':
        cout<<"enter color file name:"<<endl;
        cin>>fname; 
        for(i=0;i<MAX_CAM;i++){
            for(j=0;j<nb_obj;j++){
                sprintf(tmpstr,"%s%d%d",fname,i,j);
                if(colorfinder[i][j]->Load(tmpstr)){
                    cout<<"color file "<<tmpstr<<" loaded"<<endl;
                }
                else{
                    cout<<"cannot load file "<<tmpstr<<endl;
                }
            }
        }

        break;
    case 'n': // displays the next color
        ColorTracker::displayed_color++;
        ColorTracker::displayed_color %= nb_obj;
       
        //       for(i=0;i<MAX_CAM;i++){
        // 	trackbar[i]=
        // 	  (int)(2*colorfinder[i][ColorTracker::displayed_color]->GetSigma());
        //       }
        cout<< ColorTracker::displayed_color<<endl;
        break;
    case '+': // increments color variance (lower selectivity)
        for(i=0;i<MAX_CAM;i++){
            colorfinder[i][ColorTracker::displayed_color]->IncrementSigma();
        }
        break;
    case '-': // decrements color variance (higher selectivity)
        for(i=0;i<MAX_CAM;i++){
            colorfinder[i][ColorTracker::displayed_color]->DecrementSigma();
        }
        break;
    case 'c': //calibrates the cameras intrinsics params
        for(i=0;i<MAX_CAM;i++){
            cout<<"do you want to calibrate camera "<<i<<"? (y/n)"<<endl;
            if (cvWaitKey(0)=='y'){
                if(dc[i]->CalibrateCamera(10)){
                    cout<<"saving parameters"<<endl;
                    dc[i]->SaveCameraParams(paramfile[i]);
                    loc->SetIntrinsicsParams(i,dc[i]->GetCameraParams());
                }      
                else{
                    cout<<"cannot save parameters"<<endl;
                }
            }
        }
        break;
    case 'w': // change color mode (YCbCr or Normalized)
        for(i=0;i<MAX_CAM;i++){
            colorfinder[i][ColorTracker::displayed_color]->Mode();
        }
        break;
    case 'd': // show the camera distortion
        if((diff = 1-diff)){
            for(i=0;i<MAX_CAM;i++){
                sprintf(diff_name[i],"diff %d",i);
                cvNamedWindow(diff_name[i],CV_WINDOW_AUTOSIZE);
            }
        }
        else{
            for(i=0;i<MAX_CAM;i++){
                cvDestroyWindow(diff_name[i]);
            }
        }
        break;
        //     case 'o':
        //       CvRect sel = 
        //       colorfinder[i]->Config(img[i], );
        //    default:
    case 't': //track colors
        if((track=1-track)){
            cout<< "tracking ..."<<endl;
        }
        else{
            cout<< "stop tracking ..."<<endl;
        }
        break;
    case 'l': // locate the tracked blobs in 3d
        if((locate = 1-locate)){
            cout<< "locating ..."<<endl;
            for(i=0;i<2;i++){
                loc->SetIntrinsicsParams(i,dc[i]->GetCameraParams());
            }
        }
        else{
            cout<< "stop locating ..."<<endl;
        }
        break;
    case '3': // calibrate cameras extrinsics params
        cout << "3 pressed"<<endl;
        loc->Decalibrate();
        // loc->SetDefaultParams(10,-10,150);
        //loc->PrintParams();
        save3d = 1;
        break;
    case 'h': 
        //loc->CenterOrigin();
        loc->SetDefaultTranslation(68);
        loc->PrintParams();
        break;
    case 'H':
        loc->SetDefaultParams(0,0,68);
        loc->PrintParams();
        break;
    case 'u': // sends/stops sending the 3d locations throug yarp 
        if((comm = 1-comm)){
            for(i=0;i<nb_obj;i++){
                port[i].Start();
            }
            cout<<"connection opened"<<endl;
        }
        else{
            for(i=0;i<nb_obj;i++){ 
                port[i].Stop();
            }
            cout<<"connection closed"<<endl;
        }
        break;
    case 'i':
        loc->PrintParams();
        break;
    case 'q': //exits application
        finish=1;
        break;
    }
}


#ifdef OPENCV_9_5
void on_mouse0( int event, int x, int y, int flags){
#else
    void on_mouse0( int event, int x, int y, int flags, void *param){
#endif
        on_mouse(event,x, y, flags,0); 
    }

#ifdef OPENCV_9_5
    void on_mouse1( int event, int x, int y, int flags){
#else
        void on_mouse1( int event, int x, int y, int flags, void *param){
#endif
            on_mouse(event,x, y, flags,1); 
        }

        void on_mouse( int event, int x, int y, int flags, int index)//, void* params )
        {


            IplImage *image;
            CvRect *selection;
            //mouseParam_t *mparam = (mouseParam_t *)params;
            mouseParam_t mparam = ColorTracker::GetSelectionParams(index);
 
            if(event == CV_EVENT_RBUTTONDOWN){
                cout<<"resetting histograms"<<endl;
                mparam.finder->ResetHist();
                return;
            }
    
            if(event != CV_EVENT_LBUTTONDOWN && event != CV_EVENT_LBUTTONUP){
                return;
            }

            mparam.finder->grabber->GrabFrame(&image);
            selection = &(mparam.finder->selection);
            if(! image )
                return;

            if( image->origin )
                y = image->height - y;

            if( mparam.finder->select_object )
                {
                    selection->x = MIN(x, mparam.finder->origin.x);
                    selection->y = MIN(y, mparam.finder->origin.y);
                    selection->width = selection->x + CV_IABS(x -  mparam.finder->origin.x);
                    selection->height = selection->y + CV_IABS(y - mparam.finder-> origin.y);
        
                    selection->x = MAX( selection->x, 0 );
                    selection->y = MAX( selection->y, 0 );
                    selection->width = MIN( selection->width, image->width );
                    selection->height = MIN( selection->height, image->height );
                    selection->width -= selection->x;
                    selection->height -= selection->y;
                }

            switch( event )
                {
                case CV_EVENT_LBUTTONDOWN:
                    mparam.finder->origin = cvPoint(x,y);
                    *selection = cvRect(x,y,0,0);
                    mparam.finder->select_object = 1;
                    break;
                case CV_EVENT_LBUTTONUP:
                    mparam.finder->select_object = 0;
                    if(  selection->width > 0 &&  selection->height > 0 )
                        mparam.finder->Config(image,*selection);
                    break;
                }
        }


        void update_sigma0(int slider){
            update_sigma(slider,0);
        }
        void update_sigma1(int slider){
            update_sigma(slider,1);
        }
  
        void update_sigma(int slider,int win){
            mouseParam_t mparam = ColorTracker::GetSelectionParams(win);
            mparam.finder->SetSigma(slider/2.0);
        }
#ifdef STEREOVISION_MAIN

        int main(int argc, char *argv[]){
            Stereovision vision(argc,argv);
            vision.InitVision(argc<2?NULL:argv[1]);
            vision.Run();
        }

#endif
