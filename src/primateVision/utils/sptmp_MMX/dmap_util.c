
#include "sptmpmmx.h"

int dmap_RemapImage(unsigned char *src, unsigned char *dest, int *src_idx, int length) {
   while (length--) {
           (*dest++) = src[*(src_idx++)];
   }
   return 0;
}

int dmap_blocker_getHeight(MMXMatrix * src, int dest_width, int D, int Doff, int E, int Eoff, int W)
{
  int left, right, top, bottom;
  int srcact_width, srcact_height;
  int active_width, active_height;
  int dest_height, req_cols;
  int overall_height, overall_width;

  // calc row width and dest image size
  // margins
  left = Doff+W/2;
  right = D-Doff+W/2;
  top =Eoff+W/2;
  bottom = E-Eoff+W/2;

  overall_width = dest_width;
  srcact_width = src->cols - left - right;
  srcact_height = src->rows - top - bottom;
  active_width = dest_width - left - right;
  active_height = srcact_height;
  dest_height = active_height + top + bottom;
  req_cols = (int) ceil((double) srcact_width / active_width);
  overall_height = req_cols * dest_height;
  return overall_height;
}

void dmap_blocker (MMXMatrix * src, MMXMatrix * mat, int dest_width, int D, int Doff, int E, int Eoff, int W)
{
  int y,c;
  int left, right, top, bottom;
  int srcact_width, srcact_height;
  int active_width, active_height;
  int dest_height, dest_y, req_cols;
  int overall_height, overall_width, cpy_width;

  unsigned char *psrc, *pdest;

  // calc row width and dest image size
  // margins
  left = Doff+W/2;
  right = D-Doff+W/2;
  top =Eoff+W/2;
  bottom = E-Eoff+W/2;

  overall_width = dest_width;
  srcact_width = src->cols - left - right;
  srcact_height = src->rows - top - bottom;
  active_width = dest_width - left - right;
  active_height = srcact_height;
  dest_height = active_height + top + bottom;
  req_cols = (int) ceil((double) srcact_width / active_width);
  overall_height = req_cols * dest_height;

  if ((mat->cols !=overall_width) || (mat->rows !=overall_height)) {
    fprintf(stderr,"dmap_blocker size error: mat(%d,%d) require(%d,%d)\n",mat->cols,mat->rows,overall_width,overall_height);
    return;
  }

  dest_y =0;
  for (c = 0; c < req_cols; c++) {
    if (dest_width > (src->cols -c*active_width)) {
      cpy_width = src->cols-c*active_width;
    }
    else {
      cpy_width = dest_width;
    }
    for (y = 0; y < src->rows; y++) {
      psrc = ((MM_U_8 *)(src->p[y].m)) + active_width*c;
      pdest = (MM_U_8 *)(mat->p[dest_y++].m);
      memcpy(pdest,psrc,cpy_width);
    }
  }
}

void dmap_deblocker (MMXMatrix * mat, MMXMatrix * src, int dest_width, int D, int Doff, int E, int Eoff, int W)
{
  int y,c;
  int left, right, top, bottom;
  int srcact_width, srcact_height;
  int active_width, active_height;
  int dest_height, dest_y, req_cols;
  int overall_height, overall_width, cpy_width;

  unsigned char *psrc, *pdest;

  // calculate margins
  left = Doff+W/2;
  right = D-Doff+W/2;
  top =Eoff+W/2;
  bottom = E-Eoff+W/2;


  // calc row width and dest image size
  overall_width = dest_width;
  srcact_width = src->cols - left - right;
  srcact_height = src->rows - top - bottom;
  active_width = dest_width - left - right;
  active_height = srcact_height;
  dest_height = active_height + top + bottom;
  req_cols = (int) ceil((double) srcact_width / active_width);
  overall_height = req_cols * dest_height;

  if ((mat->cols !=overall_width) || (mat->rows !=overall_height)) {
    fprintf(stderr,"dmap_blocker size error: mat(%d,%d) require(%d,%d)\n",mat->cols,mat->rows,overall_width,overall_height);
    return;
  }

  dest_y =0;
  for (c = 0; c < req_cols; c++) {
    if (active_width > (src->rows -c*active_width)) {
      cpy_width = src->rows-c*active_width;
    }
    else {
      cpy_width = active_width;
    }
    for (y = 0; y < src->rows; y++) {
      psrc = (MM_U_8 *)(mat->p[dest_y++].m) +left;
      pdest = ((MM_U_8 *)(src->p[y].m)) +left+active_width*c;

      memcpy(pdest,psrc,cpy_width);
    }
  }
}





void image_to_matrix (unsigned char * image, MMXMatrix * matrix, 
          int width, int height, int start_x, int start_y,
          int s_width, int s_height)
{
  int x,y;
  int image_index;
  // fprintf(stderr,"before loop in image to matrix\n");
  /* indeices put this way to be consistent with rest of matrix code */
  for (y = 0; y < s_height; y++) {
    for (x = 0; x < s_width; x++) {
      //   fprintf(stderr,"in itom xy = %d,%d\n",x,y);
      image_index = (start_y + y) * width + x + start_x; 
      ((MM_U_8 *)(matrix->p[y].m))[x] = image[image_index];
    
    }
  }
   
  //   debug = 1;
  //   fprintf(stderr,"before print matrix in image to matrix\n");
  //  PrintMMXMatrix(matrix,"search");
  //   c = getchar();
  // debug = 0;
}

void image_to_matrix_oversample (unsigned char * image, MMXMatrix * matrix, 
          int width, int height,
          int s_width, int s_height)
{
  int x,y,dx,dy;
  int image_index;
  // fprintf(stderr,"before loop in image to matrix\n");
  
  dx = width/s_width;
  dy = height/s_height;

  /* indeices put this way to be consistent with rest of matrix code */
  for (y = 0; y < s_height; y++) {
    image_index = 1+ y*dy*width;
    for (x = 0; x < s_width; x++) {
      //   fprintf(stderr,"in itom xy = %d,%d\n",x,y); 
      ((MM_U_8 *)(matrix->p[y].m))[x] = image[image_index];

      image_index += dx;     
    }
  }
   
  //   debug = 1;
  //   fprintf(stderr,"before print matrix in image to matrix\n");
  //  PrintMMXMatrix(matrix,"search");
  //   c = getchar();
  // debug = 0;
}

void matrix_to_image(MMXMatrix * matrix, unsigned char * image,  
          int width, int height, int start_x, int start_y,
          int s_width, int s_height)
{
  int x,y;
  int image_index;
  // fprintf(stderr,"before loop in image to matrix\n");
  /* indeices put this way to be consistent with rest of matrix code */
  for (y = 0; y < s_height; y++) {
    for (x = 0; x < s_width; x++) {
      //   fprintf(stderr,"in itom xy = %d,%d\n",x,y);
      image_index = (start_y + y) * width + x + start_x; 
      ((MM_U_8 *)image)[image_index] = ((MM_U_8 *)(matrix->p[y].m))[x];    
    }
  }
   
  //   debug = 1;
  //   fprintf(stderr,"before print matrix in image to matrix\n");
  //  PrintMMXMatrix(matrix,"search");
  //   c = getchar();
  // debug = 0;
}

void matrix_to_double (MMXMatrix * matrix, double * result, int width, int height)
{
  int x,y;
  
  /* indeices put this way to be consistent with rest of matrix code */
  for (x = 0; x < height; x++) {
    for (y = 0; y < width; y++) {
      result[x*width+y]= ((double *)(matrix->p[x].m))[y];
    }
  }

}




//-----------------------------------------------------------------------
//----------------------------------------------------------------
int MMXClip_1D(MMXMatrix *I, MMXMatrix  *O, unsigned char cmin, unsigned char cmax)
{
  int X,Y;
  register int x,y;

  mmx_t low,high;


  X = I->cols;
  Y = I->rows;

  //paddusw MM0, 0xffff - high ; in effect this clips to high 
  //psubusw MM0, (0xffff - high + low) ; in effect this clips to low 
  //paddw MM0, low ; undo the previous two offsets 

  for(x=0;x<8;x++) {
    low.ub[x] = cmin;
    high.ub[x] = cmax;
  }

  // mm2 = 0xffff - high
  //   fill min with 111111111111111s (max number)
  pcmpeqw_r2r(mm2,mm2);
  psubusb_m2r(high,mm2);


  // mm3 = (0xffff - high + low)
  pcmpeqw_r2r(mm3,mm3);
  psubusb_m2r(high,mm3);
  paddb_m2r(low,mm3);

  // mm4 = low 
  movq_m2r(low,mm4);
  


  for(y=0;y<Y;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);

    for(x=0;x<X;x+=8) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      movq_m2r (((MM_U_8 *)I->p[y].m)[x],mm0);
      paddusb_r2r (mm2,mm0);
      paddusb_r2r (mm3,mm0);
      paddb_r2r   (mm4,mm0);
      movq_r2m (mm0,((MM_U_8 *)O->p[y].m)[x]);
    }
  }

  return 0;
}


//-----------------------------------------------------------------------
//----------------------------------------------------------------
int MMXClip_2D(MMXMatrix *I, MMXMatrix  *O, 
         unsigned char rmin, unsigned char rmax,
         unsigned char gmin, unsigned char gmax)
{
  int X,Y;
  register int x,y;

  mmx_t low,high;


  X = I->cols;
  Y = I->rows;

  //paddusw MM0, 0xffff - high ; in effect this clips to high 
  //psubusw MM0, (0xffff - high + low) ; in effect this clips to low 
  //paddw MM0, low ; undo the previous two offsets 

  for(x=0;x<8;x+=2) {
    low.ub[x] = rmin;
    high.ub[x] = rmax;
    low.ub[x+1] = gmin;
    high.ub[x+1] = gmax;
  }

  // mm2 = 0xffff - high
  //   fill min with 111111111111111s (max number)
  pcmpeqw_r2r(mm2,mm2);
  psubusb_m2r(high,mm2);


  // mm3 = (0xffff - high + low)
  pcmpeqw_r2r(mm3,mm3);
  psubusb_m2r(high,mm3);
  paddb_m2r(low,mm3);

  // mm4 = low 
  movq_m2r(low,mm4);
  


  for(y=0;y<Y;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);

    for(x=0;x<X;x+=8) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      movq_m2r (((MM_U_8 *)I->p[y].m)[x],mm0);
      paddusb_r2r (mm2,mm0);
      paddusb_r2r (mm3,mm0);
      paddb_r2r   (mm4,mm0);
      movq_r2m (mm0,((MM_U_8 *)O->p[y].m)[x]);
    }
  }

  return 0;
}


//-----------------------------------------------------------------------
//----------------------------------------------------------------
int MMXClip_3D(MMXMatrix *I, MMXMatrix  *O, 
         unsigned char rmin, unsigned char rmax,
         unsigned char gmin, unsigned char gmax,
         unsigned char bmin, unsigned char bmax)
{
  int X,Y;
  register int x,y;

  mmx_t low1, low2, low3, high1, high2, high3;


  X = I->cols;
  Y = I->rows;

  //paddusw MM0, 0xffff - high ; in effect this clips to high 
  //psubusw MM0, (0xffff - high + low) ; in effect this clips to low 
  //paddw MM0, low ; undo the previous two offsets 

  low1.ub[0] = rmin;
  low1.ub[1] = gmin;
  low1.ub[2] = bmin;
  low1.ub[3] = rmin;
  low1.ub[4] = gmin;
  low1.ub[5] = bmin;
  low1.ub[6] = rmin;
  low1.ub[7] = gmin;

  high1.ub[0] = rmax;
  high1.ub[1] = gmax;
  high1.ub[2] = bmax;
  high1.ub[3] = rmax;
  high1.ub[4] = gmax;
  high1.ub[5] = bmax;
  high1.ub[6] = rmax;
  high1.ub[7] = gmax;

  low2.ub[0] = bmin;
  low2.ub[1] = rmin;
  low2.ub[2] = gmin;
  low2.ub[3] = bmin;
  low2.ub[4] = rmin;
  low2.ub[5] = gmin;
  low2.ub[6] = bmin;
  low2.ub[7] = rmin;

  high2.ub[0] = bmax;
  high2.ub[1] = rmax;
  high2.ub[2] = gmax;
  high2.ub[3] = bmax;
  high2.ub[4] = rmax;
  high2.ub[5] = gmax;
  high2.ub[6] = bmax;
  high2.ub[7] = rmax;

  low3.ub[0] = gmin;
  low3.ub[1] = bmin;
  low3.ub[2] = rmin;
  low3.ub[3] = gmin;
  low3.ub[4] = bmin;
  low3.ub[5] = rmin;
  low3.ub[6] = gmin;
  low3.ub[7] = bmin;

  high3.ub[0] = gmax;
  high3.ub[1] = bmax;
  high3.ub[2] = rmax;
  high3.ub[3] = gmax;
  high3.ub[4] = bmax;
  high3.ub[5] = rmax;
  high3.ub[6] = gmax;
  high3.ub[7] = bmax;

  emms();
  // mm2 = 0xffff - high
  //   fill min with 111111111111111s (max number)
  pcmpeqw_r2r(mm2,mm2);
  psubusb_m2r(high1,mm2);


  // mm3 = (0xffff - high + low)
  pcmpeqw_r2r(mm3,mm3);
  psubusb_m2r(high1,mm3);
  paddb_m2r(low1,mm3);


  // mm4 = 0xffff - high
  //   fill min with 111111111111111s (max number)
  pcmpeqw_r2r(mm4,mm4);
  psubusb_m2r(high2,mm4);


  // mm5 = (0xffff - high + low)
  pcmpeqw_r2r(mm5,mm5);
  psubusb_m2r(high2,mm5);
  paddb_m2r(low2,mm5);  

  // mm6 = 0xffff - high
  //   fill min with 111111111111111s (max number)
  pcmpeqw_r2r(mm6,mm6);
  psubusb_m2r(high3,mm6);


  // mm7 = (0xffff - high + low)
  pcmpeqw_r2r(mm7,mm7);
  psubusb_m2r(high3,mm7);
  paddb_m2r(low3,mm7);  


  for(y=0;y<Y;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);

    for(x=0;x<X;x+=24) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      movq_m2r (((MM_U_8 *)I->p[y].m)[x],mm0);
      paddusb_r2r (mm2,mm0);
      paddusb_r2r (mm3,mm0);
      paddb_m2r   (low1,mm0);
      movq_r2m (mm0,((MM_U_8 *)O->p[y].m)[x]);

      movq_m2r (((MM_U_8 *)I->p[y].m)[x+8],mm0);
      paddusb_r2r (mm4,mm0);
      paddusb_r2r (mm5,mm0);
      paddb_m2r   (low2,mm0);
      movq_r2m (mm0,((MM_U_8 *)O->p[y].m)[x+8]);


      movq_m2r (((MM_U_8 *)I->p[y].m)[x+16],mm0);
      paddusb_r2r (mm6,mm0);
      paddusb_r2r (mm7,mm0);
      paddb_m2r   (low3,mm0);
      movq_r2m (mm0,((MM_U_8 *)O->p[y].m)[x+16]);

    }
  }
  emms();

  return 0;
}










