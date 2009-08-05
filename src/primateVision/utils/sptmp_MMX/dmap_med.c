
#include "depthmap.h"



/*
inline void MMXpix_sort(mmx_t *a, mmx_t *b)
{
  movq_m2r(*a,mm5);
  //PRINTREG8(mm5,"in a");
  movq_m2r(*b,mm6);
  //PRINTREG8(mm6,"in b");
  movq_r2r(mm5,mm0);
  psubb_r2r(mm7,mm0);

  movq_r2r(mm6,mm1);
  psubb_r2r(mm7,mm1);

  pcmpgtb_r2r(mm1,mm0);
  movq_r2r(mm0,mm1);
  movq_r2r(mm0,mm2);
  pand_r2r(mm5,mm1);
  pand_r2r(mm6,mm2);
  movq_r2r(mm0,mm3);
  movq_r2r(mm0,mm4);
  pandn_r2r(mm5,mm3);
  pandn_r2r(mm6,mm4);
  por_r2r(mm4,mm1);
  por_r2r(mm3,mm2);
  movq_r2m(mm1,*b);
  movq_r2m(mm2,*a);
  //PRINTREG8(mm2,"out a");
  //PRINTREG8(mm1,"out b");

} 

void MMXopt_med9(mmx_t * p)
{
    MMXpix_sort(&p[1], &p[2]) ;
    MMXpix_sort(&p[4], &p[5]) ; 
    MMXpix_sort(&p[7], &p[8]) ;
    MMXpix_sort(&p[0], &p[1]) ; 
    MMXpix_sort(&p[3], &p[4]) ; 
    MMXpix_sort(&p[6], &p[7]) ;
    MMXpix_sort(&p[1], &p[2]) ; 
    MMXpix_sort(&p[4], &p[5]) ; 
    MMXpix_sort(&p[7], &p[8]) ;
    MMXpix_sort(&p[0], &p[3]) ; 
    MMXpix_sort(&p[5], &p[8]) ; 
    MMXpix_sort(&p[4], &p[7]) ;
    MMXpix_sort(&p[3], &p[6]) ; 
    MMXpix_sort(&p[1], &p[4]) ; 
    MMXpix_sort(&p[2], &p[5]) ;
    MMXpix_sort(&p[4], &p[7]) ; 
    MMXpix_sort(&p[4], &p[2]) ; 
    MMXpix_sort(&p[6], &p[4]) ;
    MMXpix_sort(&p[4], &p[2]) ; 
}
*/

#define MMXpix_sort(a, b)\
{\
  movq_m2r(a,mm5);\
  movq_m2r(b,mm6);\
  movq_r2r(mm5,mm0);\
  psubb_r2r(mm7,mm0);\
  movq_r2r(mm6,mm1);\
  psubb_r2r(mm7,mm1);\
  pcmpgtb_r2r(mm1,mm0);\
  movq_r2r(mm0,mm1);\
  movq_r2r(mm0,mm2);\
  movq_r2r(mm0,mm3);\
  pand_r2r(mm5,mm1);\
  pand_r2r(mm6,mm2);\
  pandn_r2r(mm5,mm0);\
  pandn_r2r(mm6,mm3);\
  por_r2r(mm3,mm1);\
  por_r2r(mm0,mm2);\
  movq_r2m(mm1,b);\
  movq_r2m(mm2,a);\
}


void MMXopt_med9(mmx_t * p)
{
    MMXpix_sort(p[1], p[2]) ;
    MMXpix_sort(p[4], p[5]) ; 
    MMXpix_sort(p[7], p[8]) ;
    MMXpix_sort(p[0], p[1]) ; 
    MMXpix_sort(p[3], p[4]) ; 
    MMXpix_sort(p[6], p[7]) ;
    MMXpix_sort(p[1], p[2]) ; 
    MMXpix_sort(p[4], p[5]) ; 
    MMXpix_sort(p[7], p[8]) ;
    MMXpix_sort(p[0], p[3]) ; 
    MMXpix_sort(p[5], p[8]) ; 
    MMXpix_sort(p[4], p[7]) ;
    MMXpix_sort(p[3], p[6]) ; 
    MMXpix_sort(p[1], p[4]) ; 
    MMXpix_sort(p[2], p[5]) ;
    MMXpix_sort(p[4], p[7]) ; 
    MMXpix_sort(p[4], p[2]) ; 
    MMXpix_sort(p[6], p[4]) ;
    MMXpix_sort(p[4], p[2]) ; 
    // median value is in p[4]
}


void MMXopt_med25(mmx_t * p)
{
    MMXpix_sort(p[0], p[1]) ;   MMXpix_sort(p[3], p[4]) ;   MMXpix_sort(p[2], p[4]) ;
    MMXpix_sort(p[2], p[3]) ;   MMXpix_sort(p[6], p[7]) ;   MMXpix_sort(p[5], p[7]) ;
    MMXpix_sort(p[5], p[6]) ;   MMXpix_sort(p[9], p[10]) ;  MMXpix_sort(p[8], p[10]) ;
    MMXpix_sort(p[8], p[9]) ;   MMXpix_sort(p[12], p[13]) ; MMXpix_sort(p[11], p[13]) ;
    MMXpix_sort(p[11], p[12]) ; MMXpix_sort(p[15], p[16]) ; MMXpix_sort(p[14], p[16]) ;
    MMXpix_sort(p[14], p[15]) ; MMXpix_sort(p[18], p[19]) ; MMXpix_sort(p[17], p[19]) ;
    MMXpix_sort(p[17], p[18]) ; MMXpix_sort(p[21], p[22]) ; MMXpix_sort(p[20], p[22]) ;
    MMXpix_sort(p[20], p[21]) ; MMXpix_sort(p[23], p[24]) ; MMXpix_sort(p[2], p[5]) ;
    MMXpix_sort(p[3], p[6]) ;   MMXpix_sort(p[0], p[6]) ;   MMXpix_sort(p[0], p[3]) ;
    MMXpix_sort(p[4], p[7]) ;   MMXpix_sort(p[1], p[7]) ;   MMXpix_sort(p[1], p[4]) ;
    MMXpix_sort(p[11], p[14]) ; MMXpix_sort(p[8], p[14]) ;  MMXpix_sort(p[8], p[11]) ;
    MMXpix_sort(p[12], p[15]) ; MMXpix_sort(p[9], p[15]) ;  MMXpix_sort(p[9], p[12]) ;
    MMXpix_sort(p[13], p[16]) ; MMXpix_sort(p[10], p[16]) ; MMXpix_sort(p[10], p[13]) ;
    MMXpix_sort(p[20], p[23]) ; MMXpix_sort(p[17], p[23]) ; MMXpix_sort(p[17], p[20]) ;
    MMXpix_sort(p[21], p[24]) ; MMXpix_sort(p[18], p[24]) ; MMXpix_sort(p[18], p[21]) ;
    MMXpix_sort(p[19], p[22]) ; MMXpix_sort(p[8], p[17]) ;  MMXpix_sort(p[9], p[18]) ;
    MMXpix_sort(p[0], p[18]) ;  MMXpix_sort(p[0], p[9]) ;   MMXpix_sort(p[10], p[19]) ;
    MMXpix_sort(p[1], p[19]) ;  MMXpix_sort(p[1], p[10]) ;  MMXpix_sort(p[11], p[20]) ;
    MMXpix_sort(p[2], p[20]) ;  MMXpix_sort(p[2], p[11]) ;  MMXpix_sort(p[12], p[21]) ;
    MMXpix_sort(p[3], p[21]) ;  MMXpix_sort(p[3], p[12]) ;  MMXpix_sort(p[13], p[22]) ;
    MMXpix_sort(p[4], p[22]) ;  MMXpix_sort(p[4], p[13]) ;  MMXpix_sort(p[14], p[23]) ;
    MMXpix_sort(p[5], p[23]) ;  MMXpix_sort(p[5], p[14]) ;  MMXpix_sort(p[15], p[24]) ;
    MMXpix_sort(p[6], p[24]) ;  MMXpix_sort(p[6], p[15]) ;  MMXpix_sort(p[7], p[16]) ;
    MMXpix_sort(p[7], p[19]) ;  MMXpix_sort(p[13], p[21]) ; MMXpix_sort(p[15], p[23]) ;
    MMXpix_sort(p[7], p[13]) ;  MMXpix_sort(p[7], p[15]) ;  MMXpix_sort(p[1], p[9]) ;
    MMXpix_sort(p[3], p[11]) ;  MMXpix_sort(p[5], p[17]) ;  MMXpix_sort(p[11], p[17]) ;
    MMXpix_sort(p[9], p[17]) ;  MMXpix_sort(p[4], p[10]) ;  MMXpix_sort(p[6], p[12]) ;
    MMXpix_sort(p[7], p[14]) ;  MMXpix_sort(p[4], p[6]) ;   MMXpix_sort(p[4], p[7]) ;
    MMXpix_sort(p[12], p[14]) ; MMXpix_sort(p[10], p[14]) ; MMXpix_sort(p[6], p[7]) ;
    MMXpix_sort(p[10], p[12]) ; MMXpix_sort(p[6], p[10]) ;  MMXpix_sort(p[6], p[17]) ;
    MMXpix_sort(p[12], p[17]) ; MMXpix_sort(p[7], p[17]) ;  MMXpix_sort(p[7], p[10]) ;
    MMXpix_sort(p[12], p[18]) ; MMXpix_sort(p[7], p[12]) ;  MMXpix_sort(p[10], p[18]) ;
    MMXpix_sort(p[12], p[20]) ; MMXpix_sort(p[10], p[20]) ; MMXpix_sort(p[10], p[12]) ;

    // median is in p[12]
}


// Median kernel of 3x3 or 5x5
//#define MMX_MEDIAN_25

/********************************************************************/
/* median transform                                                 */
/********************************************************************/
void MMXmedian (MMXMatrix *I, MMXMatrix  *O)
{
  int Y,X;

  register  int x,y;

#ifdef MMX_MEDIAN_25
  mmx_t pix_arr[25];

  Y = I->rows;
  X = I->cols;
   
  
  memcpy(((MM_U_8 *)O->p[0].m),((MM_U_8 *)I->p[0].m),MMT_U_8*X);
  memcpy(((MM_U_8 *)O->p[Y-1].m),((MM_U_8 *)I->p[Y-1].m),MMT_U_8*X);
  memcpy(((MM_U_8 *)O->p[1].m),((MM_U_8 *)I->p[1].m),MMT_U_8*X);
  memcpy(((MM_U_8 *)O->p[Y-2].m),((MM_U_8 *)I->p[Y-2].m),MMT_U_8*X);

  for (y = 1; y < Y; y++) {
      ((MM_U_8 *)O->p[y].m)[0] = ((MM_U_8 *)I->p[y].m)[0];
      ((MM_U_8 *)O->p[y].m)[X-1] = ((MM_U_8 *)I->p[y].m)[X-1];
      ((MM_U_8 *)O->p[y].m)[1] = ((MM_U_8 *)I->p[y].m)[1];
      ((MM_U_8 *)O->p[y].m)[X-2] = ((MM_U_8 *)I->p[y].m)[X-2];
  }

  emms();

  // make 0x88888888
  // for sign removal
  pxor_r2r(mm7,mm7);
  pcmpeqw_r2r(mm1,mm1);
  psubb_r2r(mm1,mm7);
  psllw_i2r(7,mm7);
  //PRINTREG8(mm7,"eights");


  for (y = 2; y < (Y-2); y++) {
    for (x = 2; x < (X-2); x+=8) {      
      movq (((MM_U_8 *)I->p[y-2].m)[x-2],pix_arr[0]);
      movq (((MM_U_8 *)I->p[y-2].m)[x-1],pix_arr[1]);
      movq (((MM_U_8 *)I->p[y-2].m)[x],pix_arr[2]);
      movq (((MM_U_8 *)I->p[y-2].m)[x+1],pix_arr[3]);
      movq (((MM_U_8 *)I->p[y-2].m)[x+2],pix_arr[4]);

      movq (((MM_U_8 *)I->p[y-1].m)[x-2],pix_arr[5]);
      movq (((MM_U_8 *)I->p[y-1].m)[x-1],pix_arr[6]);
      movq (((MM_U_8 *)I->p[y-1].m)[x],pix_arr[7]);
      movq (((MM_U_8 *)I->p[y-1].m)[x+1],pix_arr[8]);
      movq (((MM_U_8 *)I->p[y-1].m)[x+2],pix_arr[9]);

      movq (((MM_U_8 *)I->p[y].m)[x-2],pix_arr[10]);
      movq (((MM_U_8 *)I->p[y].m)[x-1],pix_arr[11]);
      movq (((MM_U_8 *)I->p[y].m)[x],pix_arr[12]);
      movq (((MM_U_8 *)I->p[y].m)[x+1],pix_arr[13]);
      movq (((MM_U_8 *)I->p[y].m)[x+2],pix_arr[14]);

      movq (((MM_U_8 *)I->p[y+1].m)[x-2],pix_arr[15]);
      movq (((MM_U_8 *)I->p[y+1].m)[x-1],pix_arr[16]);
      movq (((MM_U_8 *)I->p[y+1].m)[x],pix_arr[17]);
      movq (((MM_U_8 *)I->p[y+1].m)[x+1],pix_arr[18]);
      movq (((MM_U_8 *)I->p[y+1].m)[x+2],pix_arr[19]);

      movq (((MM_U_8 *)I->p[y+2].m)[x-2],pix_arr[20]);
      movq (((MM_U_8 *)I->p[y+2].m)[x-1],pix_arr[21]);
      movq (((MM_U_8 *)I->p[y+2].m)[x],pix_arr[22]);
      movq (((MM_U_8 *)I->p[y+2].m)[x+1],pix_arr[23]);
      movq (((MM_U_8 *)I->p[y+2].m)[x+2],pix_arr[24]);
      
      MMXopt_med25(pix_arr);

      movq (pix_arr[12],((MM_U_8 *)O->p[y].m)[x]);

    }
  }

#else


  mmx_t pix_arr[9];

  Y = I->rows;
  X = I->cols;
   
  
  memcpy(((MM_U_8 *)O->p[0].m),((MM_U_8 *)I->p[0].m),MMT_U_8*X);
  memcpy(((MM_U_8 *)O->p[Y-1].m),((MM_U_8 *)I->p[Y-1].m),MMT_U_8*X);

  for (y = 1; y < Y; y++) {
      ((MM_U_8 *)O->p[y].m)[0] = ((MM_U_8 *)I->p[y].m)[0];
      ((MM_U_8 *)O->p[y].m)[X-1] = ((MM_U_8 *)I->p[y].m)[X-1];

  }

  emms();

  // make 0x88888888
  // for sign removal
  pxor_r2r(mm7,mm7);
  pcmpeqw_r2r(mm1,mm1);
  psubb_r2r(mm1,mm7);
  psllw_i2r(7,mm7);
  //PRINTREG8(mm7,"eights");


  for (y = 1; y < (Y-1); y++) {
    for (x = 1; x < (X-1); x+=8) {      
      movq (((MM_U_8 *)I->p[y-1].m)[x-1],pix_arr[0]);
      movq (((MM_U_8 *)I->p[y-1].m)[x],pix_arr[1]);
      movq (((MM_U_8 *)I->p[y-1].m)[x+1],pix_arr[2]);
      movq (((MM_U_8 *)I->p[y].m)[x-1],pix_arr[3]);
      movq (((MM_U_8 *)I->p[y].m)[x],pix_arr[4]);
      movq (((MM_U_8 *)I->p[y].m)[x+1],pix_arr[5]);
      movq (((MM_U_8 *)I->p[y+1].m)[x-1],pix_arr[6]);
      movq (((MM_U_8 *)I->p[y+1].m)[x],pix_arr[7]);
      movq (((MM_U_8 *)I->p[y+1].m)[x+1],pix_arr[8]);
            
      MMXopt_med9(pix_arr);

      movq (pix_arr[4],((MM_U_8 *)O->p[y].m)[x]);

    }
  }

#endif
  
  emms();
}


typedef unsigned char pixelvalue;

#define PIX_SORT(a,b) { if ((a)>(b)) PIX_SWAP((a),(b)); }
#define PIX_SWAP(a,b) { pixelvalue temp=(a);(a)=(b);(b)=temp; }

pixelvalue opt_med5(pixelvalue * p)
{
    PIX_SORT(p[0],p[1]) ; PIX_SORT(p[3],p[4]) ; PIX_SORT(p[0],p[3]) ;
    PIX_SORT(p[1],p[4]) ; PIX_SORT(p[1],p[2]) ; PIX_SORT(p[2],p[3]) ;
    PIX_SORT(p[1],p[2]) ; return(p[2]) ;
}


pixelvalue opt_med9(pixelvalue * p)
{
    PIX_SORT(p[1], p[2]) ;
    PIX_SORT(p[4], p[5]) ; 
    PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[1]) ; 
    PIX_SORT(p[3], p[4]) ; 
    PIX_SORT(p[6], p[7]) ;
    PIX_SORT(p[1], p[2]) ; 
    PIX_SORT(p[4], p[5]) ; 
    PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[3]) ; 
    PIX_SORT(p[5], p[8]) ; 
    PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[3], p[6]) ; 
    PIX_SORT(p[1], p[4]) ; 
    PIX_SORT(p[2], p[5]) ;
    PIX_SORT(p[4], p[7]) ; 
    PIX_SORT(p[4], p[2]) ; 
    PIX_SORT(p[6], p[4]) ;
    PIX_SORT(p[4], p[2]) ; 
    return(p[4]) ;
}

pixelvalue opt_med25(pixelvalue * p)
{
    PIX_SORT(p[0], p[1]) ;   PIX_SORT(p[3], p[4]) ;   PIX_SORT(p[2], p[4]) ;
    PIX_SORT(p[2], p[3]) ;   PIX_SORT(p[6], p[7]) ;   PIX_SORT(p[5], p[7]) ;
    PIX_SORT(p[5], p[6]) ;   PIX_SORT(p[9], p[10]) ;  PIX_SORT(p[8], p[10]) ;
    PIX_SORT(p[8], p[9]) ;   PIX_SORT(p[12], p[13]) ; PIX_SORT(p[11], p[13]) ;
    PIX_SORT(p[11], p[12]) ; PIX_SORT(p[15], p[16]) ; PIX_SORT(p[14], p[16]) ;
    PIX_SORT(p[14], p[15]) ; PIX_SORT(p[18], p[19]) ; PIX_SORT(p[17], p[19]) ;
    PIX_SORT(p[17], p[18]) ; PIX_SORT(p[21], p[22]) ; PIX_SORT(p[20], p[22]) ;
    PIX_SORT(p[20], p[21]) ; PIX_SORT(p[23], p[24]) ; PIX_SORT(p[2], p[5]) ;
    PIX_SORT(p[3], p[6]) ;   PIX_SORT(p[0], p[6]) ;   PIX_SORT(p[0], p[3]) ;
    PIX_SORT(p[4], p[7]) ;   PIX_SORT(p[1], p[7]) ;   PIX_SORT(p[1], p[4]) ;
    PIX_SORT(p[11], p[14]) ; PIX_SORT(p[8], p[14]) ;  PIX_SORT(p[8], p[11]) ;
    PIX_SORT(p[12], p[15]) ; PIX_SORT(p[9], p[15]) ;  PIX_SORT(p[9], p[12]) ;
    PIX_SORT(p[13], p[16]) ; PIX_SORT(p[10], p[16]) ; PIX_SORT(p[10], p[13]) ;
    PIX_SORT(p[20], p[23]) ; PIX_SORT(p[17], p[23]) ; PIX_SORT(p[17], p[20]) ;
    PIX_SORT(p[21], p[24]) ; PIX_SORT(p[18], p[24]) ; PIX_SORT(p[18], p[21]) ;
    PIX_SORT(p[19], p[22]) ; PIX_SORT(p[8], p[17]) ;  PIX_SORT(p[9], p[18]) ;
    PIX_SORT(p[0], p[18]) ;  PIX_SORT(p[0], p[9]) ;   PIX_SORT(p[10], p[19]) ;
    PIX_SORT(p[1], p[19]) ;  PIX_SORT(p[1], p[10]) ;  PIX_SORT(p[11], p[20]) ;
    PIX_SORT(p[2], p[20]) ;  PIX_SORT(p[2], p[11]) ;  PIX_SORT(p[12], p[21]) ;
    PIX_SORT(p[3], p[21]) ;  PIX_SORT(p[3], p[12]) ;  PIX_SORT(p[13], p[22]) ;
    PIX_SORT(p[4], p[22]) ;  PIX_SORT(p[4], p[13]) ;  PIX_SORT(p[14], p[23]) ;
    PIX_SORT(p[5], p[23]) ;  PIX_SORT(p[5], p[14]) ;  PIX_SORT(p[15], p[24]) ;
    PIX_SORT(p[6], p[24]) ;  PIX_SORT(p[6], p[15]) ;  PIX_SORT(p[7], p[16]) ;
    PIX_SORT(p[7], p[19]) ;  PIX_SORT(p[13], p[21]) ; PIX_SORT(p[15], p[23]) ;
    PIX_SORT(p[7], p[13]) ;  PIX_SORT(p[7], p[15]) ;  PIX_SORT(p[1], p[9]) ;
    PIX_SORT(p[3], p[11]) ;  PIX_SORT(p[5], p[17]) ;  PIX_SORT(p[11], p[17]) ;
    PIX_SORT(p[9], p[17]) ;  PIX_SORT(p[4], p[10]) ;  PIX_SORT(p[6], p[12]) ;
    PIX_SORT(p[7], p[14]) ;  PIX_SORT(p[4], p[6]) ;   PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[12], p[14]) ; PIX_SORT(p[10], p[14]) ; PIX_SORT(p[6], p[7]) ;
    PIX_SORT(p[10], p[12]) ; PIX_SORT(p[6], p[10]) ;  PIX_SORT(p[6], p[17]) ;
    PIX_SORT(p[12], p[17]) ; PIX_SORT(p[7], p[17]) ;  PIX_SORT(p[7], p[10]) ;
    PIX_SORT(p[12], p[18]) ; PIX_SORT(p[7], p[12]) ;  PIX_SORT(p[10], p[18]) ;
    PIX_SORT(p[12], p[20]) ; PIX_SORT(p[10], p[20]) ; PIX_SORT(p[10], p[12]) ;

    return (p[12]);
}


/********************************************************************/
/* median transform                                                 */
/********************************************************************/
void median (MMXMatrix *I, MMXMatrix  *O)
{
  int Y,X;

  register  int x,y;

  MM_U_8 pix_arr[9];

  Y = I->rows;
  X = I->cols;
   
  
  memcpy(((MM_U_8 *)O->p[0].m),((MM_U_8 *)I->p[0].m),MMT_U_8*X);
  memcpy(((MM_U_8 *)O->p[Y-1].m),((MM_U_8 *)I->p[Y-1].m),MMT_U_8*X);
  for (y = 1; y < Y; y++) {
      ((MM_U_8 *)O->p[y].m)[0] = ((MM_U_8 *)I->p[y].m)[0];
      ((MM_U_8 *)O->p[y].m)[X-1] = ((MM_U_8 *)I->p[y].m)[X-1];

  }

  for (y = 1; y < (Y-1); y+=1) {
    for (x = 1; x < (X-1); x++) {
      pix_arr[0] = ((MM_U_8 *)I->p[y-1].m)[x-1];
      pix_arr[1] = ((MM_U_8 *)I->p[y-1].m)[x];
      pix_arr[2] = ((MM_U_8 *)I->p[y-1].m)[x+1];
      pix_arr[3] = ((MM_U_8 *)I->p[y].m)[x-1];
      pix_arr[4] = ((MM_U_8 *)I->p[y].m)[x];
      pix_arr[5] = ((MM_U_8 *)I->p[y].m)[x+1];
      pix_arr[6] = ((MM_U_8 *)I->p[y-1].m)[x-1];
      pix_arr[7] = ((MM_U_8 *)I->p[y].m)[x];
      pix_arr[8] = ((MM_U_8 *)I->p[y+1].m)[x+1];

      ((MM_U_8 *)O->p[y].m)[x] = opt_med9(pix_arr);
    }
  }
  
 
}


