#include <stdio.h>
#include <string.h>
#include <iostream.h>

extern "C" {
#include "depthmap.h"
#include "timer.h"
	   }

#include "acvc_fg.h"
#include "image.h"

int depth (Image *left, Image *right, Image *result)
{
  register int i;
  
	
  Matrix *resultmatrix, *leftmatrix, *rightmatrix;

  
  
  leftmatrix = MatrixAlloc(MMT_U_8, left->height, left->width);
  rightmatrix = MatrixAlloc(MMT_U_8, right->height, right->width);
  
  resultmatrix = MatrixAlloc(MMT_U_8, result->height, result->width);
	
  image_to_matrix (left->cdata, leftmatrix, left->width, left->height, 0, 0, left->width, left->height);
	
  image_to_matrix (right->cdata, rightmatrix, right->width, right->height, 0, 0, right->width, right->height);

  //cout << "depth: Depth Map \n";
  //for(i=0;i<10;i++) {
  //  resetTimer();

  //MMXrank_trans(leftmatrix, leftrankmatrix);
  //MMXrank_trans(rightmatrix, rightrankmatrix);
  
  //Depthmap_SAD(leftmatrix, rightmatrix, resultmatrix);
  //Recursive_Depthmap_SAD(leftmatrix, rightmatrix, resultmatrix);
  
  //MMXRecursive_Depthmap_SAD(leftrankmatrix, rightrankmatrix, resultmatrix);
  //MMXRecursive_Depthmap_SAD2(leftrankmatrix, rightrankmatrix, resultmatrix);
  //Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
  //Recursive_Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
  MMXRecursive_Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
  //fprintf(stdout,"Timer: %f\n",getTimer());
  //}

  cerr << "depth: Convert result to image \n";
  
  matrix_to_image (resultmatrix, result->cdata, result->width, result->height, 0, 0, result->width, result->height);

}


int main(int argc,char **argv)
{
  int rc, count=10;
  Image *source, *work, *left, *right, *left_sub, *right_sub, *depth_sub;
  ACVCFrameGrabber *acvc;


  source = new Image(512,480,true,ULONG);
  work = new Image(512,480,true,UCHAR);
  left = new Image(512,240,true,UCHAR);
  right = new Image(512,240,true,UCHAR);
  left_sub = new Image(128,120,true,UCHAR);
  right_sub = new Image(128,120,true,UCHAR);
  depth_sub = new Image(128,120,false,UCHAR);

  acvc = new ACVCFrameGrabber();

  acvc->initVideo(512,480);
  acvc->getImage(source->ldata);

  while (count--)
  {
    cout << "Init Timer. \n";
    resetTimer();

    cout << "Get Image. " << acvc->getImage(source->ldata) << "\n";

    cout << "Conv to Char. " << work->conv2Char(source) << "\n";

    left_sub->conv2RGB();
    right_sub->conv2RGB();	

    cout << "Split Image. " << work->conv2Two(left,right) << "\n";

    cout << "Oversample Image. " << left_sub->conv2Sub(left) << "\n";
    cout << "Oversample Image. " << right_sub->conv2Sub(right) << "\n";

    left_sub->conv2Grey();
    right_sub->conv2Grey();	

    cout << "Comp Depthmap. \n";


    depth(left_sub,right_sub,depth_sub);


    cout << "Timer: " << getTimer() << "\n";

  }

  depth_sub->save("rtdepthD.ppm");
  left_sub->save("rtdepthL.ppm");
  right_sub->save("rtdepthR.ppm");

  work->save("rtdepth.ppm");
    
  delete depth_sub;
  delete right_sub;
  delete left_sub;
  delete right;
  delete left;
  delete work;
  delete source;
  delete acvc;
  return rc; 

}

