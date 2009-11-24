/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 *
 */

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <math.h>

#include "rectify.h"

#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) > (b) ? (b) : (a))

#define IPP_RADIAL_TANGENTIAL 1
#define IPP_RADIAL            0


iCub::contrib::primateVision::Rectify::Rectify(bool clear_, char* crf,IppiSize ss_,double t_o_, double sfx_, double sfy_,double mfx_, double mfy_)
{

  fc1 = sfx_; //for sim..
  fc2 = sfy_;

  clear = clear_;
  width=ss_.width;
  height=ss_.height;

  tilt_offset=t_o_;

  srcsize=ss_;
  srcroi.x=0;
  srcroi.y=0;
  srcroi.width=ss_.width;
  srcroi.height=ss_.height;

 
  //set up rectification stuff:
  uO = get_vec(2);
  uA = get_vec(2);
  uB = get_vec(2);
  uC = get_vec(2);
  uD = get_vec(2);

  T=get_mat(3,3);
  A=get_mat(3,3);
  Po=get_mat(3,4);
  Pn=get_mat(3,4);
  An=get_mat(3,3);
  Rt=get_mat(3,4);
  Rtn=get_mat(3,4);
  H=get_mat(3,3);
  
  mod = false;



  
  //load intrinsic parameters from matlab file:
  char str [80];
  FILE *f;
  int psb_buf;
  
  
  if (std::string("none") != crf){
    printf("Rectify: Loading A from intrinsic params file: %s\n",crf);
    f=fopen(crf, "r");
    
    //get to data:
    fscanf(f,"%s",str);
    while (strcmp(str,"length:")!=0){
      fscanf(f,"%s",str);
    } 
    for (int i=0;i<3;i++){fscanf(f,"%s",str);}
    fscanf(f,"%f",&fc1);
    fscanf(f,"%s",str);
    fscanf(f,"%f",&fc2);
    for (int i=0;i<7;i++){fscanf(f,"%s",str);}
    fscanf(f,"%f",&cc1);
    fscanf(f,"%s",str);
    fscanf(f,"%f",&cc2);
    for (int i=0;i<13;i++){fscanf(f,"%s",str);}
    fscanf(f,"%f",&kc1);
    fscanf(f,"%s",str);
    fscanf(f,"%f",&kc2);
    fscanf(f,"%s",str);
    fscanf(f,"%f",&pc1);
    fscanf(f,"%s",str);
    fscanf(f,"%f",&pc2);
    fclose(f);
    
    //BAREL RECT PREPARATION:
    //A few different ways implemented.. to compare speed and output quality.
    
#if IPP_RADIAL
    //prepare for direct UndistortRadial without look-up table:
    //fast, but perhaps not continuous:
    ippiUndistortGetSize (srcsize, &buflen);
    buffer = ippiMalloc_8u_C1(buflen,1,&psb_buf);
#endif
    
    //OR     
    
#if IPP_RADIAL_TANGENTIAL
    //prepare ippi-made look-up table including radial AND tangential rectification
    //for use with ippiRemap:
    //probably slower, but may give more continuous result!
    ippiUndistortGetSize (srcsize, &buflen);
    buffer = ippiMalloc_8u_C1(buflen,1,&psb_buf);
    xMap = ippiMalloc_32f_C1(srcroi.width, srcsize.height, &xStep);    
    yMap = ippiMalloc_32f_C1(srcroi.width, srcsize.height, &yStep);
    ippiCreateMapCameraUndistort_32f_C1R (xMap, xStep, yMap, yStep, srcsize,
					  fc1, fc2, cc1, cc2, kc1, kc2, pc1, pc2, buffer);
#endif

  }
  else{
    printf("Rectify: Not applying barrel rectification.\n");
	//calibrated default values for simulator    
    cc1 = width/2.0;
    cc2 = height/2.0;
  }

// imput ('real') cam from reality or sim cam:
  A->me[0][0] = fc1; A->me[0][1] = 0.0; A->me[0][2] = cc1;
  A->me[1][0] = 0.0; A->me[1][1] = fc2; A->me[1][2] = cc2; 
  A->me[2][0] = 0.0; A->me[2][1] = 0.0; A->me[2][2] = 1.0;


//virtual mosaic output cam we are constructing should have same properties as real ones:
//can scale things by scaling fc1 and fc2 in An:
  An->me[0][0] = mfx_; An->me[0][1] = 0.0; An->me[0][2] = width/2.0;
  An->me[1][0] = 0.0; An->me[1][1] = mfy_; An->me[1][2] = height/2.0; 
  An->me[2][0] = 0.0; An->me[2][1] = 0.0; An->me[2][2] = 1.0;

}


iCub::contrib::primateVision::Rectify::~Rectify()
{

}





void iCub::contrib::primateVision::Rectify::barrel_rect(Ipp8u *image,int psb_in_, Ipp8u *rect_image, int psb_){

#if IPP_RADIAL_TANGENTIAL || MY_RADIAL_TANGENTIAL
  ippiRemap_8u_C1R(image,srcsize,psb_in_,
		   srcroi,xMap,xStep,yMap,yStep,
		   rect_image,psb_,srcsize,IPPI_INTER_CUBIC);
#endif


#if IPP_RADIAL
  ippiUndistortRadial_8u_C1R(image, psb_in_, rect_image, psb_, srcsize,
        fc1, fc2, cc1, cc2, kc1, kc2, buffer);
#endif

}







void iCub::contrib::primateVision::Rectify::proc(double ang_t, double ang_v, double ang_roll, double tx, double ty, double tz){

  //construct [R|t]:
  r = IPP_PI*ang_roll/180.0;// (angle around z axis, roll.)
  p = IPP_PI*(ang_t+tilt_offset)/180.0; //tilt (angle around x axis, pitch) up is +
  y = IPP_PI*ang_v/180.0; //pan (angle around y axis, yaw)  L is + // SIM: horiz angle seems to be doubled.
 // tx = 0.0; //x-pos of camera, origin is mid of baseline. in pixels
 // ty = 0.0; //y-pos of camera, should probably use 0.0.
 // tz = 0.0; //z-pos of camera, should probably use 0.0.
  
  //construct [R|t] for o:
  Rt->me[0][0] = cos(y)*cos(r); 
  Rt->me[0][1] = sin(p)*sin(y)*cos(r)-cos(p)*sin(r); 
  Rt->me[0][2] = cos(p)*sin(y)*cos(r)+sin(p)*sin(r); 
  Rt->me[0][3] = tx*cos(y);
  Rt->me[1][0] = cos(y)*sin(r); 
  Rt->me[1][1] = sin(p)*sin(p)*sin(r)+cos(p)*cos(r); 
  Rt->me[1][2] = cos(p)*sin(p)*sin(r)-sin(p)*cos(r); 
  Rt->me[1][3] = ty*sin(y);
  Rt->me[2][0] = -sin(y); 
  Rt->me[2][1] = sin(p)*cos(y); 
  Rt->me[2][2] = cos(p)*cos(y); 
  Rt->me[2][3] = tz;
  
  //create Po = A[R|t]:
  Po = m_mlt(A,Rt,MNULL);
  

  //desired rigid transformations to
  //camera orientation that 
  //enforces parallel epipolar geometry.
  //want a virtual cam at origin pointing along z-axis.

  //make Pn:
  rn = 0.0;//(angle around z axis, roll)
  pn = 0.0;//tilt (angle around x axis, pitch)
  yn_ = 0.0;//pan (angle around y axis, yaw)
  txn = 0.0;
  tyn = 0.0;
  tzn = 0.0;
  
  //construct [R|t] for n:
  Rtn->me[0][0] = cos(yn_)*cos(rn); 
  Rtn->me[0][1] = sin(pn)*sin(yn_)*cos(rn)-cos(pn)*sin(rn); 
  Rtn->me[0][2] = cos(pn)*sin(yn_)*cos(rn)+sin(pn)*sin(rn); 
  Rtn->me[0][3] = txn*cos(yn_);
  Rtn->me[1][0] = cos(yn_)*sin(rn); 
  Rtn->me[1][1] = sin(pn)*sin(pn)*sin(rn)+cos(pn)*cos(rn); 
  Rtn->me[1][2] = cos(pn)*sin(pn)*sin(rn)-sin(pn)*cos(rn); 
  Rtn->me[1][3] = tyn*sin(yn_);
  Rtn->me[2][0] = -sin(yn_); 
  Rtn->me[2][1] = sin(pn)*cos(yn_); 
  Rtn->me[2][2] = cos(pn)*cos(yn_); 
  Rtn->me[2][3] = tzn;
  

  //create Pn = A[R|t]:
  Pn = m_mlt(An,Rtn,MNULL);  // Pn = A*[R|t];
  //(re-use A as we want a virtual cam with equivalent params)  

  //get T:
  id_mat(H);
  P2T(Po,Pn,H,T);


  transf(T,width/2.0,height/2.0,uO);         //image centre
  transf(T,0.0,0.0,uA);                      //top left
  transf(T,(double)width,0.0,uB);            //top right
  transf(T,(double)width,(double)height,uC); //bottom right
  transf(T,0.0,(double)height,uD);           //bottom left
  
  
  // check for reflections
  if(uA->ve[0]>uC->ve[0]) {
    H->me[0][0]=-1.0;
    mod = true;}
  if(uA->ve[1]>uC->ve[1]) {
    H->me[1][1]=-1.0;
    mod = true;}
  if(mod){
    // modify T if necessary 
    P2T(Po,Pn,H,T);
    mod = false;
  }


  //memory alignment:

  //in it's simplest form this is enough:
    ix = (uO->ve[0] - width/2);
    iy = (uO->ve[1] - height/2);

#if 0
  //this check needs updating:
 
  //however, we want to maximise the post-transformation image area:
  if      (uA->ve[0]>=uD->ve[0] && uA->ve[0]>=0){ ix = uA->ve[0];}
  else if (uD->ve[0]>=uA->ve[0] && uD->ve[0]>=0){ ix = uD->ve[0];}
  else if (uA->ve[0]<=uD->ve[0] && uD->ve[0]<=0){ ix = uD->ve[0];}
  else  //if  (uD->ve[0]<=uA->ve[0] && uA->ve[0]<=0)
          { ix = uA->ve[0];}

  if      (uA->ve[1]>=uB->ve[1] && uA->ve[1]>=0){ iy = uA->ve[1];}
  else if (uB->ve[1]>=uA->ve[1] && uB->ve[1]>=0){ iy = uB->ve[1];}
  else if (uA->ve[1]<=uB->ve[1] && uB->ve[1]<=0){ iy = uB->ve[1];}
  else  //if  (uB->ve[1]<=uA->ve[1] && uA->ve[1]<=0)
          { iy = uA->ve[1];}
#endif

  //apply shift (in pixels) to T that brings (centre -for simple methed- of) transformed image back to origin of output image.
  //this brings the transformed image back into the output memory space.
  shift_origin(T,-ix,-iy);  

  //ix and iy are then the locations the image should be put in the mosaic.
}




void  iCub::contrib::primateVision::Rectify::epipolar_rect(Ipp8u *imagein,int psb_in, Ipp8u *imageout,int psb)
{

  for (int a=0;a<3;a++){
    for(int b=0;b<3;b++){
	Tc[a][b]=T->me[a][b];
    }
  }

  //this removes previous border contents, but leaves
  //black borders that affect saliency. better not to reset.
 if (clear){ 
	ippiSet_8u_C1R(0,imageout,psb,srcsize);
 }

  ippiWarpPerspective_8u_C1R(imagein,srcsize,psb_in,srcroi,
			     imageout,psb,srcroi,Tc,
			     IPPI_INTER_CUBIC|IPPI_SMOOTH_EDGE);

}









void  iCub::contrib::primateVision::Rectify::P2T (MAT *Po_, MAT *Pn_, MAT* H_,MAT *T_)
 {

   
   MAT *Qo, *IQo, *Qn, *Pna;
   Qo= get_mat(3,3); 
   IQo= get_mat(3,3); 
   Qn= get_mat(3,3); 
   Pna= get_mat(3,3); 

   Pna = m_mlt(H_,Pn_,MNULL);
   Pn_ = cp_mat(Pna,Pn_);

   set_col(Qo,0,get_col(Po_,0,VNULL));
   set_col(Qo,1,get_col(Po_,1,VNULL));
   set_col(Qo,2,get_col(Po_,2,VNULL));
   
   IQo = m_inverse(Qo,MNULL);   
   
   set_col(Qn,0,get_col(Pn_,0,VNULL));
   set_col(Qn,1,get_col(Pn_,1,VNULL));
   set_col(Qn,2,get_col(Pn_,2,VNULL));
   
   T_= m_mlt(Qn,IQo,T_);

   freemat(Qo);
   freemat(IQo);
   freemat(Qn);
   freemat(Pna);

 } 


void  iCub::contrib::primateVision::Rectify::shift_origin(MAT *A_, int x_, int y_)
 {
   // shift origin by applying transaltion
   MAT *Taux, *K;
   K = get_mat(3,3);
   Taux = get_mat(3,3);

// create a translation matrix:
   id_mat(K);

   if (y_!=0){
     K->me[1][2]=(double)y_;} //shift along y.
   if (x_!=0){
     K->me[0][2]=(double)x_;} //shift along x.

//apply it to our transformation:
   Taux=m_mlt(K,A_,MNULL);
   cp_mat(Taux,A_);
   
   freemat(Taux);
   freemat(K);

  }


void iCub::contrib::primateVision::Rectify::transf(MAT *A_, double a_, double b_, VEC * ris)
 {
   
   VEC *u1, *u2;
   
   u1 = get_vec(3);
   u1->ve[0] = a_;
   u1->ve[1] = b_;
   u1->ve[2] = 1; 
   u2 = mv_mlt(A_,u1,VNULL);  

   ris->ve[0]= u2->ve[0]/u2->ve[2];
   ris->ve[1]= u2->ve[1]/u2->ve[2]; 


   freevec(u1);
   freevec(u2);

 }
