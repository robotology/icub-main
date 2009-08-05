#include <stdio.h>
#include "images.h"
#include <string.h>

int census (char *in_fname, long *out_fname, int window_size)
{
    int image_width = 512;
  int image_height = 240;
  unsigned long *imagebuf, *resultbuf;
  unsigned char *charbuf, *greybuf, *resultbuf;
  
  imagebuf = (unsigned long *) malloc(sizeof(unsigned long)*image_width*image_height);
  charbuf = (unsigned char *) malloc(sizeof(unsigned char)*3*image_width*image_height);
  greybuf = (unsigned char *) malloc(sizeof(unsigned char)*image_width*image_height);


  resultbuf = (unsigned long *) malloc(sizeof(unsigned long)*image_width*image_height);
  
  
  load_image_ppm(in_fname, imagebuf,image_width, image_height);

  pack_rgb(imagebuf,charbuf,image_width,image_height);
  
  rgb_to_greyscale (charbuf, greybuf, image_width, image_height);

  census_trans(greybuf, resultbuf, image_width, image_height, window_size);

  
  save_image_32int(out_fname,resultbuf,image_width,image_height);
  

  free(imagebuf);
  free(greybuf);
  free(resultbuf);
  free(charbuf);
}


int main(int argc,char **argv)
{
  int image_width = 512;
  int image_height = 480;
  int window_size = 5;
  char in_fname[255], out_fname[255], *pext;
  int rc=0;

  
  if (argc == 3)
  {
    // command line mode: convert filename.lint
    // converts filename.lint to filename.ppm
    strcpy(in_fname,argv[1]);
    strcpy(out_fname,argv[1]);
    pext = strstr(out_fname,".ppm");
    if (pext) {
      strcpy(pext,"CENSUS.lint\0");
    }
    else
    {
      strcpy(out_fname+strlen(out_fname),".lint\0");
      //printf("%s\n",fname);
    }
    window_size=atoi(argv[2]);
    rc = census(in_fname, out_fname,window_size);
    
  }
  else if (argc == 4)
  {
    // command line mode: convert filename.lint image.ppm
    // converts filename.lint to image.ppm
    strcpy(in_fname,argv[1]);
    strcpy(out_fname,argv[2]);
    window_size=atoi(argv[3]);
    rc = census(in_fname, out_fname,window_size);
    
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
        strcpy(out_fname+strlen(out_fname),"CENSUS.ppm\0");
      }
      printf("%s\n",out_fname);      
    }
    rc = census(in_fname, out_fname, window_size);
  }
  else
  {
    printf("usage: %s [in_image.ppm] [out_image.lint] window_size",argv[0]);
  }

        return rc; 

}
