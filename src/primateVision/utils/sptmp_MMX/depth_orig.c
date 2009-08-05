#include <stdio.h>
#include <string.h>

#include "images.h"
#include "depthmap.h"
#include "timer.h"

int depth (char *left_fname, char *right_fname, char *out_fname, int window_size)
{
    int image_width = 512;
  int image_height = 240;
  unsigned long *imagebuf;
  unsigned char *charbuf, *greybuf, *resultbuf, *leftbuf, *rightbuf, *leftrankbuf, *rightrankbuf;
  Matrix *resultmatrix, *leftmatrix, *rightmatrix, *leftrankmatrix, *rightrankmatrix;
  int i;
   

  imagebuf = (unsigned long *) malloc(sizeof(unsigned long)*image_width*image_height);
  charbuf = (unsigned char *) malloc(sizeof(unsigned char)*3*image_width*image_height);

  leftbuf = (unsigned char *) malloc(sizeof(unsigned char)*image_width*image_height);
  rightbuf = (unsigned char *) malloc(sizeof(unsigned char)*image_width*image_height);
  resultbuf = (unsigned char *) malloc(sizeof(unsigned char)*image_width*image_height);
  
  leftmatrix = MatrixAlloc(MMT_U_8, image_height, image_width);
  rightmatrix = MatrixAlloc(MMT_U_8, image_height, image_width);
  leftrankmatrix = MatrixAlloc(MMT_U_8, image_height, image_width);
  rightrankmatrix = MatrixAlloc(MMT_U_8, image_height, image_width);
  resultmatrix = MatrixAlloc(MMT_U_8, image_height, image_width);



  fprintf(stderr,"depth: Load %s.\n",left_fname);
  load_image_ppm(left_fname, imagebuf,image_width, image_height);
  pack_rgb(imagebuf,charbuf,image_width,image_height);
  rgb_to_greyscale (charbuf, leftbuf, image_width, image_height);

  fprintf(stderr,"depth: Load %s.\n",right_fname);
  load_image_ppm(right_fname, imagebuf,image_width, image_height);
  pack_rgb(imagebuf,charbuf,image_width,image_height);
  rgb_to_greyscale (charbuf, rightbuf, image_width, image_height);
  
  
  fprintf(stderr,"depth: Convert to matrix %s.\n",left_fname);
        image_to_matrix (leftbuf, leftmatrix, image_width, image_height, 0, 0, image_width, image_height);

  fprintf(stderr,"depth: Convert to matrix %s.\n",right_fname);
        image_to_matrix (rightbuf, rightmatrix, image_width, image_height, 0, 0, image_width, image_height);  


  fprintf(stderr,"depth: Depth Map \n");
  for(i=0;i<10;i++) {
    resetTimer();

    //MMXrank_trans(leftmatrix, leftrankmatrix);
    //MMXrank_trans(rightmatrix, rightrankmatrix);
  
    //Depthmap_SAD(leftmatrix, rightmatrix, resultmatrix);
    //Recursive_Depthmap_SAD(leftmatrix, rightmatrix, resultmatrix);
    
    //MMXRecursive_Depthmap_SAD(leftrankmatrix, rightrankmatrix, resultmatrix);
    //MMXRecursive_Depthmap_SAD2(leftrankmatrix, rightrankmatrix, resultmatrix);
    //Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
    //Recursive_Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
    MMXRecursive_Depthmap_NCC(leftmatrix, rightmatrix, resultmatrix);
    fprintf(stdout,"Timer: %f\n",getTimer());
  }

  fprintf(stderr,"depth: Convert result to image \n");
        matrix_to_image (resultmatrix, resultbuf, image_width, image_height, 0, 0, image_width, image_height);

  fprintf(stderr,"depth: Make RGB \n");
  greyscale_to_rgb (resultbuf, charbuf, image_width, image_height);

  fprintf(stderr,"depth: Save %s\n",out_fname);
  save_image_ppm(out_fname, charbuf, image_width, image_height);
  

  free(imagebuf);
  free(leftbuf);
  free(rightbuf);
  free(resultbuf);
  free(charbuf);

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
    printf("usage: %s [left_image.ppm] [right_image.ppm] [out_image.ppm] window_size",argv[0]);
  }

        return rc; 

}



/* junk draw
  //resetTimer();
  //rank_trans(rightbuf, rightrankbuf, image_width, image_height, window_size);
  //fprintf(stdout,"Timer: %f\n",getTimer());

  //resetTimer();
  //rank_trans(leftbuf, leftrankbuf, image_width, image_height, window_size);
  //fprintf(stdout,"Timer: %f\n",getTimer());


  //leftrankbuf = (unsigned char *) malloc(sizeof(unsigned char)*image_width*image_height);
  //rightrankbuf = (unsigned char *) malloc(sizeof(unsigned char)*image_width*image_height);


  //free(greybuf);

 */
