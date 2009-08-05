#include <stdio.h>
#include <string.h>
#include <iostream.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <stdlib.h>
#include <memory.h>


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
  int fd;
  int rc, count=10;
  Image *source, *work, *left, *right, *left_sub, *right_sub, *depth_sub;
  ACVCFrameGrabber *acvc;

  char *data = "Hello, World";
  char buff[4096];
  int buff_len;

  struct sockaddr_in sin;
  int s;

  sin.sin_family = AF_INET;

  sin.sin_port = htons(12345); // htons for network byte order

  sin.sin_addr.s_addr = inet_addr("150.203.126.139"); 
  s = socket(AF_INET, SOCK_DGRAM, 0);
  bind(s, (struct sockaddr *)&sin, sizeof(sin));


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

    depth_sub->save("/tmp/rtdepthD.ppm");
    system("cjpeg -gr /tmp/rtdepthD.ppm > /tmp/rtdepthD.jpg");
    fd = open("/tmp/rtdepthD.jpg",O_RDONLY);
    buff_len =  read(fd,buff,4000);
    close(fd);
    sendto(s, buff, buff_len+1, 0, (struct sockaddr *)&sin, sizeof(sin));

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

