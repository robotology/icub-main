#include <stdio.h>
#include <string.h>
#include <iostream.h>


#include "image.h"

extern "C" {
 #include "depthmap.h"
#include "timer.h"
     }

int depth (char *left_fname, char *right_fname, char *out_fname, int window_size)
{
  register int i;
  float avg,timer;
  
  MMXMatrix *resultmatrix, *leftmatrix, *rightmatrix;
  MMXMatrix *leftrankmatrix, *rightrankmatrix;

  Image *left, *right, *result;

  fprintf(stderr,"depth: Load %s.\n",left_fname);
  left = new Image(left_fname);

  fprintf(stderr,"depth: Load %s.\n",right_fname);
  right = new Image(right_fname);

  result = new Image(left->width(),left->height(),1);


  leftmatrix = MMXMatrixAlloc(MMT_U_8, left->width(),left->height());
  rightmatrix = MMXMatrixAlloc(MMT_U_8, right->width(),right->height());
  leftrankmatrix = MMXMatrixAlloc(MMT_U_8, left->width(),left->height());
  rightrankmatrix = MMXMatrixAlloc(MMT_U_8, right->width(), right->height());
  
  resultmatrix = MMXMatrixAlloc(MMT_U_8, left->width(),left->height());


  left->conv2Char();
  right->conv2Char();  

  left->conv2Grey();
  right->conv2Grey();  
  
  cout <<"depth: Convert to matrix " << left_fname<< "\n";
  MMX_img_MMU8(left->data(), leftmatrix);
  
  cout <<"depth: Convert to matrix " << right_fname<< "\n";
  MMX_img_MMU8(right->data(), rightmatrix);


  cout << "depth: Median \n";

  resetTimer();
  //median(leftmatrix, resultmatrix);
  //MMXmedian(leftmatrix, resultmatrix);
  timer = getTimer();
  fprintf(stdout,"Median: %f\n",timer);  

  MMX_MMU8_img(resultmatrix, result->data());
  
  //  result->expand();
  result->save("medout.ppm");



  cout << "depth: Rank \n";

  MMXrank_trans(leftmatrix, leftrankmatrix);
  MMXrank_trans(rightmatrix, rightrankmatrix);
  leftrankmatrix = leftmatrix;
  rightrankmatrix = rightmatrix;

  MMX_MMU8_img(leftrankmatrix, result->data());
  
  result->save("rankL.pgm");

  MMX_MMU8_img(rightrankmatrix, result->data());
  
  //  result->expand();
  result->save("rankR.pgm");


  //  for(i=0;i<10;i++) {

  resetTimer();

  //Depthmap_SAD(leftmatrix, rightmatrix, resultmatrix);
  //Recursive_Depthmap_SAD(leftmatrix, rightmatrix, resultmatrix);
  
  //MMXRecursive_Depthmap_SAD(leftrankmatrix, rightrankmatrix, resultmatrix);
  //MMXRecursive_Depthmap_SAD2(leftrankmatrix, rightrankmatrix, resultmatrix);
 
  //MMXRecursive_Depthmap_SAD3(leftrankmatrix, rightrankmatrix, resultmatrix);

  MMXRecursive_Depthmap_SAD4(leftrankmatrix, rightrankmatrix, resultmatrix);
  //MMXRecursive_Depthmap_SAD6(leftrankmatrix, rightrankmatrix, resultmatrix);
  //Depthmap_FLOW(leftrankmatrix, rightrankmatrix, resultmatrix);
  //Recursive_Depthmap_FLOW(leftrankmatrix, rightrankmatrix, resultmatrix);
  //MMXRecursive_Depthmap_FLOW(leftrankmatrix, rightrankmatrix, resultmatrix);
  //Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
  //Recursive_Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
  //MMXRecursive_Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
  timer = getTimer();
  avg += timer;
  fprintf(stdout,"Timer: %f\n",timer);  
  //  }

  avg /=10;
  fprintf(stdout,"Timer avg: %f\n",avg);

  cerr << "depth: Convert result to image \n";


  MMX_MMU8_img(resultmatrix, result->data());
  
  //  result->expand();

//    result->conv2RGB();
//    for(i=0;i<result->getSize();i+=3) {
//      if(result->data()[i]==0xff) {
//        result->data()[i+1]=0x0;
//        result->data()[i+2]=0x0;
//      }
//    }
  result->save(out_fname);

  //  result->expand();
  //result->conv2RGB();
  for(i=0;i<result->getSize();i++) {
    result->data()[i]= ((result->data()[i]&0x0f)<<4) ;
    
  }
  result->save("dvalues.ppm");

}


int main(int argc,char **argv)
{
  int image_width = 512;
  int image_height = 240;
  int window_size = 16;
  char left_fname[255], right_fname[255], out_fname[255], *pext;
  int rc=0;

  
  if (argc == 4)
  {
    // command line mode: convert filename.lint
    // converts filename.lint to filename.ppm
    strcpy(left_fname,argv[1]);
    strcpy(right_fname,argv[2]);
    strcpy(out_fname,argv[3]);
    //pext = strstr(out_fname,".ppm");
    // if (pext) {
    //  strcpy(pext,"DEPTH.ppm\0");
    // }
    //else
    // {
    //  strcpy(out_fname+strlen(out_fname),".ppm\0");
      //printf("%s\n",fname);
    // }
    //window_size=atoi(argv[2]);
    rc = depth(left_fname, right_fname, out_fname,window_size);
    
  }
  /*
  else if (argc == 4)
  {
    // command line mode: convert filename.lint image.ppm
    // converts filename.lint to image.ppm
    strcpy(in_fname,argv[1]);
    strcpy(out_fname,argv[2]);
    window_size=atoi(argv[3]);
    rc = depth(in_fname, out_fname,window_size);
    
  }
  else if (argc ==2)
  {
    window_size=atoi(argv[1]);
    while (scanf("%s ",in_fname)!=EOF)
    {
      strcpy(out_fname,in_fname);
      pext = strstr(out_fname,".lint");
      if (pext) {
        strcpy(pext,".ppm\0");
      }
      else
      {
        strcpy(out_fname+strlen(out_fname),"DEPTH.ppm\0");
      }
      printf("%s\n",out_fname);      
    }
    rc = depth(in_fname, out_fname, window_size);
  }
  */
  else
  {
    cout << "usage: " << argv[0] << " [left_image.ppm] [right_image.ppm] [out_image.ppm] window_size";
  }

        return rc; 

}





