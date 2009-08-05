
#include "sptmpmmx.h"



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


/********************************************************************/
/* rank transform                                                   */
/********************************************************************/
void MMXrank_trans (MMXMatrix *I, MMXMatrix  *O)
{
  int Y,X;
  const int W = 8;

  mmx_t ones,result;

  register  int x,y,j,i, w_2, count,curr;

  w_2 = W/2;
  Y = I->rows;
  X = I->cols;
  

  
  // take care of 0->W/2
  for (y = 0; y < w_2; y++) {
    for (x = 0; x < w_2; x++) {
      count = 0;
      curr = ((MM_U_8 *)I->p[y].m)[x];
      //printf("rank: source: %d\n",curr);
      for (j = 0; j < W; j++) {
  for (i = 0; i < W; i++) {
    if (((y+j-w_2)>=0)&&((x+i-w_2)>=0)){
      if (((MM_U_8 *)I->p[y+j-w_2].m)[x+i-w_2]<curr) { 
        count++;
      }
    }
      
  }
      }
      ((MM_U_8 *)O->p[y].m)[x]=count; 
    }
  }
  // take care of Y-W/2 -> Y
  for (y = (Y-w_2); y < Y; y++) {
    for (x = (X-w_2); x < X; x++) {
      count = 0;
      curr = ((MM_U_8 *)I->p[y].m)[x];
      for (j = 0; j < W; j++) {
  for (i = 0; i < W; i++) {
    if (((y+j-w_2)<Y)&&((x+i-w_2)<X)) {
      if (((MM_U_8 *)I->p[y+j-w_2].m)[x+i-w_2]<curr) { 
        count++;
      }
    }
      
  }
      }
      ((MM_U_8 *)O->p[y].m)[x]=count;
    }
  }
 
  /*
  // the rest
  for (y = 0; y < Y; y++) {
    for (x = 0; x < X; x++) {
      count = 0;
     
      curr = ((MM_U_8 *)I->p[y].m)[x];
      //printf("rank: source: %d\n",curr);
      for (j = 0; j < W; j++) {
  for (i = 0; i < W; i++) {
    if ( ((y+j-w_2)>=0) && ((y+j-w_2)<Y) ) {
      if ( ((x+i-w_2)>=0) && ((x+i-w_2)<X) ) {
        if (((MM_U_8 *)I->p[y+j-w_2].m)[x+i-w_2]<curr) { 
    count++;
        }
      }
    }
      
  }
      }
      ((MM_U_8 *)O->p[y].m)[x]=count;
      //printf("rank: dest: %d\n",count);
    
    }
  }
  */


  emms();
  ones.ud[0]=0x01010101;
  ones.ud[1]=0x01010101;

  movq_m2r(ones,mm6);      

  pxor_r2r(mm7,mm7);

  for (y = 0; y < Y-W; y++) {
    for (x = 0; x < X-W; x++) {
      // I -> I1 I1 I1 I1 I1 I1 I1 I1
      movq_m2r(((MM_U_8 *)I->p[y+w_2].m)[x+w_2],mm0);      
      punpcklbw_r2r(mm7,mm0);
      punpcklbw_r2r(mm0,mm0);
      punpcklwd_r2r(mm0,mm0);
      punpckldq_r2r(mm0,mm0);

      pxor_r2r(mm5,mm5);
      for (j = 0; j < W; j++) {
        // I -> I1 I2 I3 I4 I5 I6 I7 I8
  movq_m2r(((MM_U_8 *)I->p[y+j].m)[x],mm2);
  // reg 2 becomes mask
        pcmpgtb_r2r(mm0,mm2);
  pand_r2r(mm6,mm2);
  paddb_r2r(mm2,mm5);
      }
      movq_r2r (mm5,mm4);
      punpcklbw_r2r (mm7,mm5);
      punpckhbw_r2r (mm7,mm4);
      paddw_r2r (mm5,mm4);
      movq_r2r (mm4,mm3);
      punpcklwd_r2r (mm7,mm3);
      punpckhwd_r2r (mm7,mm4);
      paddd_r2r (mm4,mm3);  
      movq_r2m (mm3,result);

      ((MM_U_8 *)O->p[y+w_2].m)[x+w_2]= result.ud[0] + result.ud[1];          
    }
  }
  emms();
}


/********************************************************************/
/* laplace of gaussian filter                                       */
/* approximated with a difference of gaussian filter                */
/********************************************************************/
void LoG_filt(MMXMatrix *I, MMXMatrix  *O)
{
  int YY,XX,WW;

  register  int x,y,i, w_2, accum;

  MMXMatrix *K;

  WW = 7;
  YY = I->rows;
  XX = I->cols;
  w_2 = WW/2;


  K = MMXMatrixAlloc(MMT_U_16, 7, 7);

((MM_U_16 *)K->p[0].m)[0] = 299;
((MM_U_16 *)K->p[0].m)[1] = 494;
((MM_U_16 *)K->p[0].m)[2] = 598;
((MM_U_16 *)K->p[0].m)[3] = 611;
((MM_U_16 *)K->p[0].m)[4] = 598;
((MM_U_16 *)K->p[0].m)[5] = 494;
((MM_U_16 *)K->p[0].m)[6] = 299;

((MM_U_16 *)K->p[1].m)[0] = 494;
((MM_U_16 *)K->p[1].m)[1] = 601;
((MM_U_16 *)K->p[1].m)[2] = 277;
((MM_U_16 *)K->p[1].m)[3] = -15;
((MM_U_16 *)K->p[1].m)[4] = 277;
((MM_U_16 *)K->p[1].m)[5] = 601;
((MM_U_16 *)K->p[1].m)[6] = 494;

((MM_U_16 *)K->p[2].m)[0] = 598;
((MM_U_16 *)K->p[2].m)[1] = 277;
((MM_U_16 *)K->p[2].m)[2] = -1156;
((MM_U_16 *)K->p[2].m)[3] = -2169;
((MM_U_16 *)K->p[2].m)[4] = -1156;
((MM_U_16 *)K->p[2].m)[5] = 277;
((MM_U_16 *)K->p[2].m)[6] = 598;

((MM_U_16 *)K->p[0].m)[0] = 611;
((MM_U_16 *)K->p[0].m)[1] = -15;
((MM_U_16 *)K->p[0].m)[2] = -2169;
((MM_U_16 *)K->p[0].m)[3] = -3640;
((MM_U_16 *)K->p[0].m)[4] = -2169;
((MM_U_16 *)K->p[0].m)[5] = -15;
((MM_U_16 *)K->p[0].m)[6] = 611;

((MM_U_16 *)K->p[4].m)[0] = 598;
((MM_U_16 *)K->p[4].m)[1] = 277;
((MM_U_16 *)K->p[4].m)[2] = -1156;
((MM_U_16 *)K->p[4].m)[3] = -2169;
((MM_U_16 *)K->p[4].m)[4] = -1156;
((MM_U_16 *)K->p[4].m)[5] = 277;
((MM_U_16 *)K->p[4].m)[6] = 598;


((MM_U_16 *)K->p[5].m)[0] = 494;
((MM_U_16 *)K->p[5].m)[1] = 601;
((MM_U_16 *)K->p[5].m)[2] = 277;
((MM_U_16 *)K->p[5].m)[3] = -15;
((MM_U_16 *)K->p[5].m)[4] = 277;
((MM_U_16 *)K->p[5].m)[5] = 601;
((MM_U_16 *)K->p[5].m)[6] = 494;

((MM_U_16 *)K->p[6].m)[0] = 299;
((MM_U_16 *)K->p[6].m)[1] = 494;
((MM_U_16 *)K->p[6].m)[2] = 598;
((MM_U_16 *)K->p[6].m)[3] = 611;
((MM_U_16 *)K->p[6].m)[4] = 598;
((MM_U_16 *)K->p[6].m)[5] = 494;
((MM_U_16 *)K->p[6].m)[6] = 299;

  for (y = w_2; y < (YY-w_2); y++) {
    for (x = w_2; x < (XX-w_2); x++) {
      accum = 0;
      for (i = 0; i < WW; i++) {

  accum += ((MM_U_8 *)I->p[y-w_2].m)[x-w_2+i]* ((MM_U_16 *)K->p[0].m)[i];
  accum += ((MM_U_8 *)I->p[y-w_2+1].m)[x-w_2+i]* ((MM_U_16 *)K->p[1].m)[i];
  accum += ((MM_U_8 *)I->p[y-w_2+2].m)[x-w_2+i]* ((MM_U_16 *)K->p[2].m)[i];
  accum += ((MM_U_8 *)I->p[y-w_2+3].m)[x-w_2+i]* ((MM_U_16 *)K->p[3].m)[i];
  accum += ((MM_U_8 *)I->p[y-w_2+4].m)[x-w_2+i]* ((MM_U_16 *)K->p[4].m)[i];
  accum += ((MM_U_8 *)I->p[y-w_2+5].m)[x-w_2+i]* ((MM_U_16 *)K->p[5].m)[i];
  accum += ((MM_U_8 *)I->p[y-w_2+6].m)[x-w_2+i]* ((MM_U_16 *)K->p[6].m)[i];
      }
      ((MM_U_8 *)O->p[y].m)[x]=(0x0080+((accum>>16)&0x000000ff))>>1; 
    }
  } 


}


/********************************************************************/
/* laplace of gaussian filter                                       */
/* approximated with a difference of gaussian filter                */
/********************************************************************/
void MMXLoG_filt(MMXMatrix *I, MMXMatrix  *O)
{
  MMXMatrix *Kr, *Kc, *ga1, *ga2, *tmp;
  Kc = MMXMatrixAlloc(MMT_U_16, 1, 7);
  Kr = MMXMatrixAlloc(MMT_U_16, 7, 1);
  ga1 = MMXMatrixAlloc(MMT_U_8, I->rows, I->cols);
  ga2 = MMXMatrixAlloc(MMT_U_8, I->rows, I->cols);
  tmp = MMXMatrixAlloc(MMT_U_8, I->rows, I->cols);

  // do 1st 2D gaussian
  ((MM_U_16*)Kr->p[0].m)[0]=18;
  ((MM_U_16*)Kr->p[0].m)[1]=33;
  ((MM_U_16*)Kr->p[0].m)[2]=49;
  ((MM_U_16*)Kr->p[0].m)[3]=55;
  ((MM_U_16*)Kr->p[0].m)[4]=49;
  ((MM_U_16*)Kr->p[0].m)[5]=33;
  ((MM_U_16*)Kr->p[0].m)[6]=18;

  MMXrow_filt(I,Kr,tmp);

  ((MM_U_16*)Kc->p[0].m)[0]=18;
  ((MM_U_16*)Kc->p[1].m)[0]=33;
  ((MM_U_16*)Kc->p[2].m)[0]=49;
  ((MM_U_16*)Kc->p[3].m)[0]=55;
  ((MM_U_16*)Kc->p[4].m)[0]=49;
  ((MM_U_16*)Kc->p[5].m)[0]=33;
  ((MM_U_16*)Kc->p[6].m)[0]=18;
  
  MMXcol_filt(tmp,Kc,ga1);

  // do 2nd 2D gaussian
  ((MM_U_16*)Kr->p[0].m)[0]=5;
  ((MM_U_16*)Kr->p[0].m)[1]=23;
  ((MM_U_16*)Kr->p[0].m)[2]=59;
  ((MM_U_16*)Kr->p[0].m)[3]=82;
  ((MM_U_16*)Kr->p[0].m)[4]=59;
  ((MM_U_16*)Kr->p[0].m)[5]=23;
  ((MM_U_16*)Kr->p[0].m)[6]=5;

  MMXrow_filt(I,Kr,tmp);

  ((MM_U_16*)Kc->p[0].m)[0]=5;
  ((MM_U_16*)Kc->p[1].m)[0]=23;
  ((MM_U_16*)Kc->p[2].m)[0]=59;
  ((MM_U_16*)Kc->p[3].m)[0]=82;
  ((MM_U_16*)Kc->p[4].m)[0]=59;
  ((MM_U_16*)Kc->p[5].m)[0]=23;
  ((MM_U_16*)Kc->p[6].m)[0]=5;
  
  MMXcol_filt(tmp,Kc,ga2);
  
  MMXsubtract(ga1,ga2,O);


}

/********************************************************************/
/* subtract                                                         */
/********************************************************************/
void MMXsubtract(MMXMatrix *I1, MMXMatrix  *I2, MMXMatrix  *O)
{
  int YY,XX;

  register  int x,y, accum;

  YY = I1->rows;
  XX = I1->cols;
  
  for (y = 0; y < YY; y++) {
    for (x = 0; x < XX; x++) {
      accum = 0x0080+((MM_U_8 *)I1->p[y].m)[x] - ((MM_U_8 *)I2->p[y].m)[x];
      ((MM_U_8 *)O->p[y].m)[x]= accum>>1; 
      
    }
  } 
}


/********************************************************************/
/* add                                                              */
/********************************************************************/
void MMXadd(MMXMatrix *I1, MMXMatrix  *I2, MMXMatrix  *O)
{
  int YY,XX;

  register  int x,y, accum;

  YY = I1->rows;
  XX = I1->cols;
  
  for (y = 0; y < YY; y++) {
    for (x = 0; x < XX; x++) {
      accum = ((MM_U_8 *)I1->p[y].m)[x] + ((MM_U_8 *)I2->p[y].m)[x];
      ((MM_U_8 *)O->p[y].m)[x]= accum; 
      
    }
  } 
}


/********************************************************************/
/* row filter                                                       */
/********************************************************************/
void MMXrow_filt(MMXMatrix *I, MMXMatrix  *K, MMXMatrix  *O)
{
  int YY,XX, WW, NN;

  register  int x,y,i, w_2, accum, n;

  YY = I->rows;
  XX = I->cols;
  WW = K->cols;
  w_2 = WW/2;
  NN = WW/4;


  for (y = 0; y < YY; y++) {
    for (x = w_2; x < (XX-w_2); x++) {
      for (n=0;n<NN;n++) {
  
      }
      accum = 0;
      for (i = 0; i < WW; i++) {
  accum += ((MM_U_8 *)I->p[y].m)[x-w_2+i]* ((MM_U_16 *)K->p[0].m)[i];
      }
      ((MM_U_8 *)O->p[y].m)[x]=((accum>>8)&0x000000ff); 
    }
  } 
}


/********************************************************************/
/* col filter                                                       */
/********************************************************************/
void MMXcol_filt(MMXMatrix *I, MMXMatrix  *K, MMXMatrix  *O)
{
  int YY,XX, WW;

  register  int x,y,i, w_2, accum;

  YY = I->rows;
  XX = I->cols;
  WW = K->rows;
  w_2 = WW/2;
  
  for (y = w_2; y < (YY-w_2); y++) {
    for (x = 0; x < XX; x++) {
      accum = 0;
      for (i = 0; i < WW; i++) {
  accum += ((MM_U_8 *)I->p[y-w_2+i].m)[x]* ((MM_U_16 *)K->p[i].m)[0];
      }
      ((MM_U_8 *)O->p[y].m)[x]=((accum>>8)&0x000000ff); 
    }
  } 
}




//  /*
//  void CRowFilter( DWORD p[][IMAGE_WIDTH],
//       short *h,
//       DWORD q[][IMAGE_WIDTH],
//       short nRows,
//       short nCols, short hLength )
//  {
//    short    r[IMAGE_HEIGHT][IMAGE_WIDTH];
//    short    g[IMAGE_HEIGHT][IMAGE_WIDTH]; 
//    short    b[IMAGE_HEIGHT][IMAGE_WIDTH];
//    short    a[IMAGE_HEIGHT][IMAGE_WIDTH];
  
//    DWORD A, R, G, B;
//    short i, y, x;
  
//    for (y=0; y<nRows; y++)
//      {
//        for (x=0; x<nCols; x++)
//    {
//      b[y][x] = (short) (p[y][x]&0x000000ff);
//      g[y][x] = (short) ((p[y][x]>>8)&0x000000ff);
//      r[y][x] = (short) ((p[y][x]>>16)&0x0000ff);
//      a[y][x] = (short) ((p[y][x]>>24)&0x0000ff);
//    }
//      }
  
//    for (y = 0; y < nRows; y++)
//      {
//        for (x = 0; x < nCols-hLength; x++)
//    {
//      A=0;
//      R=0;
//      G=0;
//      B=0;
//      for ( i=0; i<hLength; i++)
//        {
//          A+= a[y][x+i]*h[i];
//          R+= r[y][x+i]*h[i];
//          G+= g[y][x+i]*h[i];
//          B+= b[y][x+i]*h[i];
//        }
//      A=A+0x0080;
//      R=R+0x0080;
//      G=G+0x0080;
//      B=B+0x0080;
//      q[x] = ((A<<16)&0xff000000) |
//        ((R<<8)&0x00ff0000) |                                                   (G&0x0000ff00) |
//        ((B>>8)&0x000000ff);
//    }
//      }
  
//    //    __asm int 3
  
//  }   // eo row filter

//  6.0. ROW FILTER : MMX CODE LISTING 

//  .486P
//  .Model FLAT, C   


//  IMAGE_WIDTH     EQU     72
//  IMAGE_HEIGHT    EQU     58
//  FILTER_LENGTH   EQU     7      

//  _DATA   SEGMENT

//  Arr16       DWORD   IMAGE_WIDTH*IMAGE_HEIGHT*2  DUP   (mm0)
//  rnduparr    WORD    4                           DUP   (mm128)
  
  
//  srcsize     DWORD    IMAGE_WIDTH*IMAGE_HEIGHT*4
//  Arr16siz    DWORD    (IMAGE_WIDTH*IMAGE_HEIGHT*4*2)-64

//  pixtemp     DWORD   0


//  _DATA   ENDS

//  _TEXT   SEGMENT PUBLIC USE32 'CODE'


//  MMxRowFilter PROC C PUBLIC USES ebx ecx edi esi, src:PTR DWORD, 
//    filt:PTR SWORD,
//    dest:PTR DWORD,
//    temp:PTR DWORD

//  mov     eax, src                        ; Source bitmap pointer
//  mov     ebx, filt                       ; Filter pointer

//  mov     ecx, dest                       ; Destination bitmap pointer
//  mov     ebx, srcsize

//  mov     ecx, temp
//  pxor    mm0, mm0                        ; Initialize unpack register to zero

//  unpack:
//  movq            mm1,[eax]       ; get first two pixels, MSW = PIXn+1 :: LSW = PIXn

//  movq            mm3,8[eax]      ; get next two pixels, MSW = PIXn+3 :: LSW = PIXn+2
//  movq            mm2, mm1        ; duplicate first two pixels

//  punpcklbw       mm1, mm0        ; expand PIXn's bytes to 16-bit words
//  movq            mm4, mm3        ; duplicate next two pixels

//  movq            mm5,16[eax]     ; get next two pixels, MSW = PIXn+5 :: LSW = PIXn+4
//                                       punpckhbw       mm2, mm0        ; expand PIXn+1's bytes to words

//  movq            mm4, mm3        ; duplicate first two pixels
//  punpcklbw       mm3, mm0        ; expand PIXn+2's bytes to 16-bit words

//  movq            0[ecx], mm1     ; PIXn to memory
//  punpckhbw       mm4, mm0        ; expand PIXn+3's bytes to 16-bit words

//  movq            8[ecx], mm2     ; PIXn+1 to memory
//  movq            mm6, mm5        ; duplicate next two pixels

//  movq            16[ecx], mm3    ; PIXn+2 to memory
//  punpcklbw       mm5, mm0        ; expand PIXn+4's bytes to 16-bit words

//  movq            24[ecx], mm4    ; PIXn+3 to memory    
//  punpckhbw       mm6, mm0        ; expand PIXn+5's bytes to 16-bit words

//  movq            32[ecx], mm5    ; PIXn+4 to memory

//  movq            40[ecx], mm6    ; PIXn+5 to memory

//  add             ecx, 48 ; increment temp array by 6 pixels 
//  add             eax, 24 ; increment src array pointer by 6 pixels (n+=6)

//  sub             ebx, 24
//  jnz     unpack

//  mov         eax, temp                   ; U-pipe: load address of temp array
//  mov         ebx, filt                   ; load address filter array[0][0]

//  mov         esi, dest

//  movq        mm5, DWORD PTR rnduparr     ; load rounding constants (mm0x0080)
//  sub         esi, 8                      ; pre decrement dest array pointer for better
//  ; pairing inside the loop
                       
//  mov         edx, Arr16siz
//  movq        mm1, 8[eax]             ; load PIXn+1

//  pixloop:
//  movq        mm0, [eax]      ; load PIXn
//  pxor        mm6, mm6        ; zero out the PIXn accumulator

//  pmullw      mm0, [ebx]      ; PIXn*Hn
//  movq        mm2, mm1        ; duplicate PIXn+1

//  pmullw      mm1, [ebx]      ; PIXn*Hn
//  pxor        mm7, mm7        ; zero out the PIXn+1 accumulator

//  movq        mm3, 16[eax]    ; load PIXn+2

//  pmullw      mm2, 8[ebx]     ; PIXn+1*Hn+1
//  movq        mm4, mm3        ; duplicate PIXn+2

//  pmullw      mm3, 8[ebx]     ; PIXn+2*Hn+1
//  paddw       mm6, mm0        ; accumulate Pn

//  pmullw      mm4, 16[ebx]    ; PIXn+2*Hn+2
//  paddw       mm7, mm1        ; accumulate Pn+1

//  movq        mm0, 24[eax]    ; load PIXn+3
//  paddw       mm6, mm2        ; accumulate Pn

//  movq        mm2, 32[eax]    ; load PIXn+4
//  movq        mm1, mm0        ; duplicate PIXn+3

//  pmullw      mm0, 16[ebx]    ; PIXn+3*Hn+2
//  paddw       mm7, mm3        ; accumulate Pn+1

//  pmullw      mm1, 24[ebx]    ; PIXn+3*Hn+3
//  paddw       mm6, mm4        ; accumulate Pn

//  movq        mm3, mm2        ; duplicate PIXn+4

//  pmullw      mm2, 24[ebx]    ; PIXn+4*Hn+3 
//  paddw       mm7, mm0        ; accumulate Pn+1

//  movq        mm4, 40[eax]  ; load PIXn+5
//  paddw       mm6, mm1      ; accumulate Pn

//  pmullw      mm3, 32[ebx]  ; PIXn+4*Hn+4
//  movq        mm0, mm4      ; duplicate PIXn+5

//  pmullw      mm4, 32[ebx]  ; PIXn+5*Hn+4
//  paddw       mm7, mm2      ; accumulate Pn+1

//  movq        mm1, 48[eax]  ; load PIXn+6

//  pmullw      mm0, 40[ebx]  ; PIXn+5*Hn+5
//  movq        mm2, mm1      ; duplicate PIXn+6

//  pmullw      mm1, 40[ebx]  ; PIXn+6*Hn+5
//  paddw       mm6, mm3      ; accumulate Pn

//  movq        mm3, 56[eax]  ; load PIXn+7
//  paddw       mm7, mm4      ; accumulate Pn+1

//  pmullw      mm2, 48[ebx]  ; PIXn+6*Hn+6
//  paddw       mm6, mm0      ; accumulate Pn

//  pmullw      mm3, 48[ebx]  ; PIXn+7*Hn+6
//  paddw       mm7, mm1      ; accumulate Pn+1

//  ; one clock stall due to pmullw latency with mm2

//  paddw       mm6, mm2      ; accumulate Pn
                                     
//  ; one clock stall due to pmullw latency with mm3

//  paddw       mm7, mm3      ; accumulate Pn+1

//  paddw      mm6, mm5       ; PIXn : a=a+0x80, r=r+0x80, g=g+0x80, b=b+0x80, 
//  paddw      mm7, mm5       ; PIXn+1 : a=a+0x80, r=r+0x80, g=g+0x80, b=b+0x80, 

//  psrlw       mm6, 8        ; shift to obtain whole part of result
//  add         esi, 8        ; increment destination pointer by 2 pixels

//  psrlw       mm7, 8        ; shift to obtain whole part of result
//  add         eax, 16       ; increment temp array pointer by  2 pixels

//  packuswb    mm6, mm7      ; MSW = PIXn+1 :: LSW = PIXn
                                     
//  movq        mm1, 8[eax]   ; load PIXn+1 for next iteration of loop

//  movq        [esi], mm6    ; store to dest array

//  sub         edx, 16
//  jnz         pixloop

//  emms
//  ret     

//  MMxRowFilter ENDP
//  _TEXT   ENDS
//  END










//  //----------------------------------------
//  // column filter

//  INCLUDE iammx.inc

//  TITLE    MMX Code Column Filter
//  .486P

//  include c:\msvc20\include\listing.inc

//  .model FLAT

//  ;***************************************************************************/
//  ;* Constant Definition(s)
//    IMAGE_WIDTH     EQU    72
//  IMAGE_HEIGHT    EQU    58
//  FILTER_LENGTH   EQU    7

//  ;***************************************************************************/
//  ;* Export Declaration(s)
//    PUBLIC  MMxColFilt

//  ;***************************************************************************/
//  ;* Data Segment Definition
//  _DATA    SEGMENT

//  ; Filter coefficient table -
//  ; the table is symmetric inasmuch as h0 == h6, h1 == h5, and h2 == h4.
//  ; Assume filter coefficient table has been built by a previous process.
//    h0_6    SWORD    4 DUP (mm04)
//    h1_5    SWORD    4 DUP (mm24)
//    h2_4    SWORD    4 DUP (mm60)
//    h3      SWORD    4 DUP (80)
  
//  ; Value of 0.1 (binary), added to the accumulated products of pixel values
//  ; and coefficients, to round up to the next nearest bit.
//    rnd_off    SWORD    4 DUP (mm0080h)

//  ; Temporary variables to store intermediate results for each iteration
//  ; of the inner loop.
//    T0       QWORD         (mm0)
//    T1       QWORD         (mm0)
//    T2       QWORD         (mm0)
//    T3       QWORD         (mm0)
//    T4       QWORD         (mm0)
//    T5       QWORD         (mm0)
//    T6       QWORD         (mm0)
//    T7       QWORD         (mm0)
//    T8       QWORD         (mm0)
  
//    _DATA    ENDS

//  ;***************************************************************************/
//  ;* Text (code) Segment Definition
//  _TEXT SEGMENT

//  ;--------------------------------------------------------------------------
//  ; FUNCTION: MMxColFilt
//  ;
//  ;   Performs the column filtering on an input array, by multiplying each
//  ;   pixel in a column by the appropriate filter coefficient, and accumulating
//  ;   the results.
//  ;
//  ;   NOTE: In this implementation, function arguments for the pointer to the
//  ;         array of filter coefficients, the array height and width, and the
//  ;         number of filter coefficients are not used.  This function is not
//  ;         re-entrant as implemented.
//  ;
//  MMxColFilt PROC C PUBLIC USES eax ecx edx esi edi,
//    srcPtr    : DWORD,           ; pointer to source bitmap array
//    filtCoeff : DWORD,           ; pointer to array of filter coeff
//    dstPtr    : DWORD,           ; pointer to destination bitmap array
//    height    : WORD,            ; array height (number of rows)
//    wid       : WORD,            ; array width (number of columns)
//    filtLen   : WORD             ; filter array length
//                                 ; (number of filter coefficients)
//  ;****************************************************************
//  ; Perform the necessary setup including initializing pointers   *
//  ; and loop counters.  The following register conventions are    *
//  ; used.                                                         *
//  ;                                                               *
//  ; eax - offset into the source bitmap array                     *
//  ; ecx - column loop counter                                     *
//  ; edx - row loop counter                                        *
//  ; esi - base address of the source bitmap array                 *
//  ; edi - base address of the destination bitmap array            *
//  ;                                                               *
//  ; mm0 - pixel at [r,c]                                          *
//  ; mm1 - pixel at [r+1,c]                                        *
//  ; mm2 - pixel at [r+2,c]                                        *
//  ;    (Other mmx registers are used for a variety of purposes.)  *
//  ;****************************************************************

//  ; initialize the column loop counter
//  xor        ecx, ecx

//  ; initialize source and destination pointers
//  mov        esi, srcPtr
//  mov        edi, dstPtr

//  ;****************************************************************
//  ; This begins the column loop.  Each iteration of this loop     *
//  ; will cause the preconditioning code to execute, which         *
//  ; initializes the temporary variables for the calculations      *
//  ; performed in the inner loop.                                  *
//  ;****************************************************************

//  col_loop:
//  precondition:

//  ; initialize the row counter - preconditioning code handles
//  ; rows 0 through 5
//  mov        edx, 6

//  ; calculate the offsets for indexing into the row of the source
//  ; bitmap array
//  lea        eax, DWORD PTR [ecx*4]

//  ;****************************************************************
//  ; In the comments that follow, the reader will notice the        *
//  ; following conventions.                                        *
//  ;                                                               *
//  ;   Tx          - Temporary variable x.  There are nine         *
//  ;                 temporary variables, T0 through T8.           *
//  ;   BM[i+k,c]   - access to a particular pixel in the source    *
//  ;                 bitmap array.  Rows are designated as i+k,    *
//  ;                 and 'c' is the current column.  Each 32-bit   *
//  ;                 pixel value is formatted as indicated below.  *
//  ;                                                               *
//  ;                31     24 23    16 15     8 7      0           *
//  ;                +--------+--------+--------+--------+          *
//  ;                | alpha  |   Red  | Green  |  Blue  |          *
//  ;                +--------+--------+--------+--------+          *
//  ;                                                               *
//  ;   hx          - filter coefficient x.                         *
//  ;                                                               *
//  ; Unpacking a pixel converts it from a 32-bit value, to its     *
//  ; four component 8-bit values zero-extended to 16-bits.  For    *
//  ; example, a pixel unpacked from the 32-bit format shown above, *
//  ; will yield a 64-bit value as shown below.                     *
//  ;                                                               *
//  ;       63  56 55 48 47 40 39 32 31 24 23 16 15  8 7   0        *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;       |  0  |alpha|  0  |  R  |  0  |  G  |  0  |  B  |       *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;****************************************************************

//  ;****************************************************************
//  ; The first section of code reads in the first three pixels     *
//  ; and performs all necessary calculations with these values.    *
//  ;                                                               *
//  ;    T0 = BM[0,c]*h0 + BM[1,c]*h1 + BM[2,c]*h2                  *
//  ;    T1 = BM[1,c]*h0 + BM[2,c]*h1                               *
//  ;    T2 = BM[2,c]*h0                                            *
//  ;                                                               *
//  ; In this implementation, a pixel is never read from the source *
//  ; bitmap array more than once.                                  *
//  ;****************************************************************

//  movd        mm2, [esi+eax+IMAGE_WIDTH*4*2]      ; load pixel from BM[2,c]
//  pxor        mm7, mm7                            ; clear mm7

//  movd        mm1, [esi+eax+IMAGE_WIDTH*4]        ; load pixel from BM[1,c]
//  punpcklbw   mm2, mm7                            ; unpack pixel BM[0,c]

//  movd        mm0, [esi+eax]                      ; load pixel from BM[0,c]
//  punpcklbw   mm1, mm7                            ; unpack pixel BM[1,c]

//  movq        mm3, mm2                            ; copy pixel BM[2,c] to mm3
//  punpcklbw   mm0, mm7                            ; unpack pixel BM[0,c]

//  pmullw      mm3, DWORD PTR h0_6                 ; mm3 <- BM[2,c]*h0
//  movq        mm4, mm2                            ; copy pixel BM[2,c] to mm4

//  pmullw      mm4, DWORD PTR h1_5                 ; mm4 <- BM[2,c]*h1
//  movq        mm5, mm1                            ; copy pixel BM[1,c] to mm5

//  pmullw      mm5, DWORD PTR h0_6                 ; mm5 <- BM[1,c]*h0

//  pmullw      mm2, DWORD PTR h2_4                 ; mm2 <- BM[2,c]*h2

//  pmullw      mm1, DWORD PTR h1_5                 ; mm1 <- BM[1,c]*h1

//  pmullw      mm0, DWORD PTR h0_6                 ; mm0 <- BM[0,c]*h0
//  paddw       mm5, mm4                            ; mm5 <- BM[2,c]*h1 + BM[1,c]*h0

//  movq        DWORD PTR T2, mm3                   ; T2 <- BM[2,c]*h0

//  ;****************************************************************
//  ; Read the next set of three pixels, and perform the necessary  *
//  ; calculations.                                                 *
//  ;                                                               *
//  ;    T6 = BM[3,c]*h0 + BM[4,c]*h1 + BM[5,c]*h2                  *
//  ;    T7 = BM[4,c]*h0 + BM[5,c]*h1                               *
//  ;    T8 = BM[5,c]*h0                                            *
//  ;****************************************************************

//  movd        mm6, [esi+eax+IMAGE_WIDTH*4*5] ; load pixel from BM[5,c]
//  paddw       mm1, mm2                       ; mm1 <- BM[2,c]*h2 + BM[1,c]*h1

//  movq        DWORD PTR T1, mm5              ; T1 <- BM[2,c]*h1 + BM[1,c]*h0
//  paddw       mm0, mm1                       ; mm0 <- BM[1,c]*h1 + BM[0,c]*h0

//  movd        mm1, [esi+eax+IMAGE_WIDTH*4*4] ; load pixel from BM[4,c]
//  punpcklbw   mm6, mm7                       ; unpack pixel BM[5,c]

//  movq        DWORD PTR T0, mm0              ; T0 <- BM[2,c]*h2 + BM[1,c]*h1 + BM[0,c]*h0
//  movq         mm2, mm6                      ; save pixel BM[5,c]

//  pmullw       mm6, DWORD PTR h0_6           ; mm6 <- BM[5,c]*h0
//  punpcklbw    mm1, mm7                      ; unpack pixel BM[4,c]

//  movd        mm0, [esi+eax+IMAGE_WIDTH*4*3  ; load pixel from BM[3,c]
//  movq         mm4, mm2                      ; copy pixel BM[5,c]

//  pmullw       mm4, DWORD PTR h1_5           ; mm4 <- BM[5,c]*h1
//  movq         mm3, mm1                      ; copy pixel BM[4,c]

//  pmullw       mm3, DWORD PTR h0_6           ; mm3 <- BM[4,c]*h0
//  movq         mm5, mm2                      ; copy pixel BM[5,c]

//  pmullw       mm5, DWORD PTR h2_4           ; mm5 <- BM[5,c]*h2
//  punpcklbw    mm0, mm7                      ; unpack pixel BM[3,c]

//  movq         DWORD PTR T8, mm6             ; T8 <- BM[5,c]*h0
//  movq         mm7, mm1                      ; copy pixel BM[4,c]

//  movq         mm6, mm0                      ; copy pixel BM[3,c]
//  paddw        mm3, mm4                      ; mm3 <- BM[5,c]*h1 + BM[4,c]*h0

//  pmullw       mm7, DWORD PTR h1_5           ; mm7 <- BM[4,c]*h1

//  pmullw       mm6, DWORD PTR h0_6           ; mm6 <- BM[3,c]*h0

//  ;****************************************************************
//  ; Make additional calculations for previous three pixels.       *
//  ;                                                               *
//  ;    T3 = T0 + BM[3,c]*h3 + BM[4,c]*h4 + BM[5,c]*h5             *
//  ;    T4 = T1 + BM[3,c]*h2 + BM[4,c]*h3 + BM[5,c]*h4             *
//  ;    T5 = T2 + BM[3,c]*h1 + BM[4,c]*h2 + BM[5,c]*h3             *
//  ;****************************************************************

//  movq         DWORD PTR T7, mm3                 ; T7 <- BM[5,c]*h1 + BM[4,c]*h0
//  movq         mm4, mm2                          ; copy pixel BM[5,c]

//  pmullw       mm4, DWORD PTR h1_5               ; mm4 <- BM[5,c]*h5
//  paddw        mm7, mm5                          ; mm7 <- BM[5,c]*h2 + BM[4,c]*h1

//  movq         mm3, mm1                          ; copy pixel BM[4,c]
//  paddw        mm6, mm7                          ; mm6 <- BM[5,c]*h2 + BM[4,c]*h1
//  ; + BM[3,c]*h0
//  pmullw       mm3, DWORD PTR h2_4               ; mm3 <- BM[4,c]*h4
//  movq         mm7, mm0                          ; copy pixel BM[3,c]

//  movq         mm5, DWORD PTR T0                 ; mm5 <- BM[2,c]*h2 + BM[1,c]*h1
//  ; + BM[0,c]*h0

//  pmullw       mm7, DWORD PTR h3                 ; mm7 <- BM[3,c]*h3
//  paddw        mm5, mm4                          ; mm5 <- BM[5,c]*h5 + BM[2,c]*h2
//  ; + BM[1,c]*h1 + BM[0,c]*h0

//  movq         DWORD PTR T6, mm6                 ; T6 <- BM[5,c]*h2 + BM[4,c]*h1
//  ; + BM[3,c]*h0
//  paddw        mm3, mm5                          ; mm3 <- BM[5,c]*h5 + BM[4,c]*h4
//  ; + BM[2,c]*h2 + BM[1,c]*h1
//  ; + BM[0,c]*h0

//  movq         mm5, DWORD PTR T1                 ; mm5 <- BM[2,c]*h1 + BM[1,c]*h0
//  movq         mm6, mm2                          ; copy pixel BM[5,c]

//  pmullw       mm6, DWORD PTR h2_4               ; mm6 <- BM[5,c]*h4
//  paddw        mm7, mm3                          ; mm7 <- BM[5,c]*h5 + BM[4,c]*h4
//  ; + BM[3,c]*h3 + BM[2,c]*h2
//  ; + BM[1,c]*h1 + BM[0,c]*h0

//  pmullw       mm2, DWORD PTR h3                 ; mm2 <- BM[5,c]*h3
//  movq         mm4, mm1                          ; copy pixel BM[4,c]

//  pmullw       mm4, DWORD PTR h3                 ; mm4 <- BM[4,c]*h3
//  movq         mm3, mm0                          ; copy pixel BM[3,c]

//  pmullw       mm3, DWORD PTR h2_4               ; mm3 <- BM[3,c]*h2
//  paddw        mm5, mm6                          ; mm5 <- BM[5,c]*h4 + BM[2,c]*h1
//  ; + BM[1,c]*h0

//  pmullw       mm1, DWORD PTR h2_4               ; mm1 <- BM[4,c]*h2

//  pmullw       mm0, DWORD PTR h1_5               ; mm0 <- BM[3,c]*h1
//  paddw        mm4, mm5                          ; mm4 <- BM[5,c]*h4 + BM[4,c]*h3
//  ; + BM[2,c]*h1 + BM[1,c]*h0

//  movq         mm5, DWORD PTR T2                 ; mm5 <- BM[2,c]*h0
//  paddw        mm3, mm4                          ; mm3 <- BM[5,c]*h4 + BM[4,c]*h3
//  ; + BM[3,c]*h2 + BM[2,c]*h1
//  ; + BM[1,c]*h0

//  movq         DWORD PTR T3, mm7                 ; T3 <- BM[5,c]*h5 + BM[4,c]*h4
//  ; + BM[3,c]*h3 + BM[2,c]*h2
//  ; + BM[1,c]*h1 + BM[0,c]*h0
//  paddw        mm1, mm2                          ; mm1 <- BM[5,c]*h3 + BM[4,c]*h2

//  movq         DWORD PTR T4, mm3                 ; T4 <- BM[5,c]*h4 + BM[4,c]*h3
//  ; + BM[3,c]*h2 + BM[2,c]*h1
//  ; + BM[1,c]*h0
//  paddw        mm5, mm1                          ; mm5 <- BM[5,c]*h3 + BM[4,c]*h2
//  ; + BM[2,c]*h0

//  paddw        mm0, mm5                          ; mm0 <- BM[5,c]*h3 + BM[4,c]*h2
//  ; + BM[3,c]*h1 + BM[2,c]*h0

//  movq         DWORD PTR T5, mm0                 ; T5 <- BM[5,c]*h3 + BM[4,c]*h2
//  ; + BM[3,c]*h1 + BM[2,c]*h0

//  inner_loop:

//  ;****************************************************************
//  ; Calculate the offsets for indexing into the next              *
//  ; row of the source and destination bitmap arrays.              *
//  ;****************************************************************
//  mov        eax, edx

//  lea        eax, DWORD PTR [eax+eax*4]

//  lea        eax, DWORD PTR [eax+eax*4]

//  shl        eax, 4

//  lea        eax, DWORD PTR [eax+ecx*4]

//  ;****************************************************************
//  ; Read the next set of three pixels, and perform the necessary  *
//  ; calculations for pixels BM[r-6,c], BM[r-5,c], and BM[r-4,c].  *
//  ;                                                               *
//  ;    T0 = T3 + BM[r,c]*h6                                       *
//  ;    T1 = T4 + BM[r,c]*h5 + BM[r+1,c]*h6                        *
//  ;    T2 = T5 + BM[r,c]*h4 + BM[r+1,c]*h5 + BM[r+2,c]*h6         *
//  ;****************************************************************

//  movd        mm0, [esi+eax]                   ; load pixel from BM[r,c]
//  pxor         mm7, mm7                        ; clear mm7

//  movd        mm1, [esi+eax+IMAGE_WIDTH*4]    ; load pixel from BM[r+1,c]
//  punpcklbw    mm0, mm7                       ; unpack pixel BM[r,c]

//  movd        mm2, [esi+eax+IMAGE_WIDTH*4*2]  ; load pixel from BM[r+2,c]
//  punpcklbw    mm1, mm7                       ; unpack pixel BM[r+1,c]

//  movq         mm3, mm0                       ; copy pixel BM[r,c]
//  punpcklbw    mm2, mm7                       ; unpack pixel BM[r+2,c]

//  pmullw       mm3, DWORD PTR h0_6            ; mm3 <- BM[r,c]*h6

//  movq         mm4, DWORD PTR T3              ; mm4 <- BM[r-1,c]*h5 + BM[r-2,c]*h4
//  ; + BM[r-3,c]*h3 + BM[r-4,c]*h2
//  ; + BM[r-5,c]*h1 + BM[r-6,c]*h0
//  movq         mm5, mm1                       ; copy pixel BM[r+1,c]

//  pmullw       mm5, DWORD PTR h0_6            ; mm5 <- BM[r+1,c]*h6
//  movq         mm6, mm0                       ; copy pixel BM[r,c]

//  pmullw       mm6, DWORD PTR h1_5            ; mm6 <- BM[r,c]*h5
//  paddw        mm4, mm3                       ; mm4 <- BM[r,c]*h6 + BM[r-1,c]*h5
//  ; + BM[r-2,c]*h4 + BM[r-3,c]*h3
//  ; + BM[r-4,c]*h2 + BM[r-5,c]*h1
//  ; + BM[0,c]*h0

//  movq         mm3, DWORD PTR T4              ; mm3 <- BM[r-1,c]*h4 + BM[r-2,c]*h3
//  ; + BM[r-3,c]*h2 + BM[r-4,c]*h1
//  ; + BM[r-5,c]*h0
//  movq         mm7, mm2                       ; copy pixel BM[r+2]

//  movq         DWORD PTR T0, mm4              ; T0 <- BM[r,c]*h6 + BM[r-1,c]*h5
//  ; + BM[r-2,c]*h4 + BM[r-3,c]*h3
//  ; + BM[r-4,c]*h2 + BM[r-5,c]*h1
//  ; + BM[0,c]*h0
//  paddw        mm3, mm5                       ; mm3 <- BM[r+1,c]*h6 + BM[r-1,c]*h4
//  ; + BM[r-2,c]*h3 + BM[r-3,c]*h2
//  ; + BM[r-4,c]*h1 + BM[r-5,c]*h0

//  pmullw       mm7, DWORD PTR h0_6            ; mm7 <- BM[r+2,c]*h6
//  movq         mm4, mm1                       ; copy pixel BM[r+1,c]

//  pmullw       mm4, DWORD PTR h1_5            ; mm4 <- BM[r+1,c]*h5
//  paddw        mm6, mm3                       ; mm6 <- BM[r+1,c]*h6 + BM[r,c]*h5
//  ; + BM[r-1,c]*h4 + BM[r-2,c]*h3
//  ; + BM[r-3,c]*h2 + BM[r-4,c]*h1
//  ; + BM[r-5,c]*h0

//  movq         mm5, DWORD PTR T               ; mm5 <- BM[r-1,c]*h3 + BM[r-2,c]*h2
//  ; + BM[r-3,c]*h1 + BM[r-4,c]*h0
//  movq         mm3, mm0                       ; copy pixel BM[r,c]

//  pmullw       mm3, DWORD PTR h2_4            ; mm3 <- BM[r,c]*h4
//  paddw        mm5, mm7                       ; mm5 <- BM[r+2,c]*h6 + BM[r-1,c]*h3
//  ; + BM[r-2,c]*h2 + BM[r-3,c]*h1
//  ; + BM[r-4,c]*h0

//  movq         DWORD PTR T1, mm6              ; T1 <- BM[r+1,c]*h6 + BM[r,c]*h5
//  ; + BM[r-1,c]*h4 + BM[r-2,c]*h3
//  ; + BM[r-3,c]*h2 + BM[r-4,c]*h1
//  ; + BM[r-5,c]*h0
//  paddw        mm4, mm5                       ; mm4 <- BM[r+2,c]*h6 + BM[r+1,c]*h5
//  ; + BM[r-1,c]*h3 + BM[r-2,c]*h2
//  ; + BM[r-3,c]*h1 + BM[r-4,c]*h0

//  ;****************************************************************
//  ; Perform the necessary calculations for pixels BM[r-3,c],      *
//  ; BM[r-2,c], and BM[r-1,c].                                     *
//  ;                                                               *
//  ;    T3 = T6 + BM[r,c]*h3 + BM[r+1,c]*h4 + BM[r+2,c]*h5         *
//  ;    T4 = T7 + BM[r,c]*h2 + BM[r+1,c]*h3 + BM[r+2,c]*h4         *
//  ;    T5 = T8 + BM[r,c]*h1 + BM[r+1,c]*h2 + BM[r+2,c]*h3         *
//  ;****************************************************************

//  movq         mm5, DWORD PTR T6              ; mm5 <- BM[r-1,c]*h2 + BM[r-2,c]*h1
//  ; + BM[r-3,c]*h0
//  movq         mm6, mm2                       ; copy pixel BM[r+2,c]

//  pmullw       mm6, DWORD PTR h1_5            ; mm6 <- BM[r+2,c]*h5
//  paddw        mm3, mm4                       ; mm3 <- BM[r+2,c]*h6 + BM[r+1,c]*h5
//  ; + BM[r,c]*h4 + BM[r-1,c]*h3
//  ; + BM[r-2,c]*h2 + BM[r-3,c]*h1
//  ; + BM[r-4,c]*h0

//  movq         mm4, mm1                       ; copy pixel BM[r+1,c]

//  pmullw       mm4, DWORD PTR h2_4            ; mm4 <- BM[r+1,c]*h4
//  movq         mm7, mm0                       ; copy pixel BM[r,c]

//  pmullw       mm7, DWORD PTR h3               ; mm7 <- BM[r,c]*h3
//  paddw        mm5, mm6                       ; mm5 <- BM[r+2,c]*h5 + BM[r-1,c]*h2
//  ; + BM[r-2,c]*h1 + BM[r-3,c]*h0

//  movq         DWORD PTR T2, mm3              ; T2 <- BM[r+2,c]*h6 + BM[r+1,c]*h5
//  ; + BM[r,c]*h4 + BM[r-1,c]*h3
//  ; + BM[r-2,c]*h2 + BM[r-3,c]*h1
//  ; + BM[r-4,c]*h0

//  paddw        mm4, mm5                       ; mm4 <- BM[r+2,c]*h5 + BM[r+1,c]*h4
//  ; + BM[r-1,c]*h2 + BM[r-2,c]*h1
//  ; + BM[r-3,c]*h0
//  movq         mm3, mm2                       ; copy pixel BM[r+2,c]

//  pmullw       mm3, DWORD PTR h2_4            ; mm3 <- BM[r+2,c]*h4
//  paddw        mm7, mm4                       ; mm7 <- BM[r+2,c]*h5 + BM[r+1,c]*h4
//  ; + BM[r,c]*h3 + BM[r-1,c]*h2
//  ; + BM[r-2,c]*h1 + BM[r-3,c]*h0

//  movq         mm6, DWORD PTR T7              ; mm6 <- BM[r-1,c]*h1 + BM[r-2,c]*h0
//  movq         mm5, mm1                       ; copy pixel BM[r+1,c]

//  pmullw       mm5, DWORD PTR h3              ; mm5 <- BM[r+1,c]*h3
//  movq         mm4, mm0                       ; copy pixel BM[r,c]

//  pmullw       mm4, DWORD PTR h2_4            ; mm4 <- BM[r,c]*h2
//  paddw        mm6, mm3                       ; mm6 <- BM[r+2,c]*h4 + BM[r-1,c]*h1
//  ; + BM[r-2,c]*h0

//  movq         DWORD PTR T3, mm               ; T3 <- BM[r+2,c]*h5 + BM[r+1,c]*h4
//  ; + BM[r,c]*h3 + BM[r-1,c]*h2
//  ; + BM[r-2,c]*h1 + BM[r-3,c]*h0
//  movq         mm3, mm2                       ; copy pixel BM[r+2,c]

//  pmullw       mm3, DWORD PTR h3              ; mm3 <- BM[r+2,c]*h3
//  paddw        mm5, mm6                       ; mm5 <- BM[r+2,c]*h4 + BM[r+1,c]*h3
//  ; + BM[r-1,c]*h1 + BM[r-2,c]*h0

//  paddw        mm4, mm5                       ; mm4 <- BM[r+2,c]*h4 + BM[r+1,c]*h3
//  ; + BM[r,c]*h2 + BM[r-1,c]*h1
//  ; + BM[r-2,c]*h0

//  movq         mm5, DWORD PTR T8              ; mm5 <- BM[r-1,c]*h0
//  movq         mm6, mm1                       ; copy pixel BM[r+1,c]

//  pmullw       mm6, DWORD PTR h2_4            ; mm6 <- BM[r+1,c]*h2
//  movq         mm7, mm0                       ; copy pixel BM[r,c]

//  pmullw       mm7, DWORD PTR h1_5            ; mm7 <- BM[r,c]*h1
//  paddw        mm5, mm3                       ; mm5 <- BM[r+2,c]*h3 + BM[r-1,c]*h0

//  ;****************************************************************
//  ; Perform the necessary calculations for pixels BM[r,c],        *
//  ; BM[r+1,c], and BM[r+2,c].                                     *
//  ;                                                               *
//  ;    T6 = BM[r  ,c]*h0 + BM[r+1,c]*h1 + BM[r+2,c]*h2            *
//  ;    T7 = BM[r+1,c]*h0 + BM[r+2,c]*h2                           *
//  ;    T8 = BM[r+2,c]*h0                                          *
//  ;****************************************************************

//  movq         DWORD PTR T4, mm4              ; T4 <- BM[r+2,c]*h4 + BM[r+1,c]*h3
//  ; + BM[r,c]*h2 + BM[r-1,c]*h1
//  ; + BM[r-2,c]*h0
//  movq         mm3, mm2                       ; copy pixel BM[r+2,c]

//  pmullw       mm3, DWORD PTR h2_4            ; mm3 <- BM[r+2,c]*h2
//  movq         mm4, mm1                       ; copy pixel BM[r+1,c]

//  pmullw       mm4, DWORD PTR h1_5            ; mm4 <- BM[r+1,c]*h1
//  paddw        mm6, mm5                       ; mm6 <- BM[r+2,c]*h3 + BM[r+1,c]*h2
//  ; + BM[r-1,c]*h0

//  pmullw       mm0, DWORD PTR h0_6            ; mm0 <- BM[r,c]*h0
//  paddw        mm7, mm6                       ; mm7 <- BM[r+2,c]*h3 + BM[r+1,c]*h2
//  ; + BM[r,c]*h1 + BM[r-1,c]*h0

//  movq         mm5, mm2                       ; copy pixel BM[r+2,c]

//  pmullw       mm5, DWORD PTR h1_5            ; mm5 <- BM[r+2,c]*h1

//  pmullw       mm1, DWORD PTR h0_6            ; mm1 <- BM[r+1,c]*h0
//  paddw        mm4, mm3                       ; mm4 <- BM[r+2,c]*h2 + BM[r+1,c]*h1

//  movq         DWORD PTR T5, mm7              ; T5 <- BM[r+2,c]*h3 + BM[r+1,c]*h2
//  ; + BM[r,c]*h1 + BM[r-1,c]*h0
//  paddw        mm0, mm4                       ; mm0 <- BM[r+2,c]*h2 + BM[r+1,c]*h1
//  ; + BM[r,c]*h0

//  pmullw       mm2, DWORD PTR h0_6            ; mm2 <- BM[r+2,c]*h0

//  movq         DWORD PTR T6, mm0              ; T6 <- BM[r+2,c]*h2 + BM[r+1,c]*h1
//  ; + BM[r,c]*h0
//  paddw        mm1, mm5                       ; mm1 <- BM[r+2,c]*h1 + BM[r+1,c]*h0

//  movq         DWORD PTR T7, mm1              ; T7 <- BM[r+2,c]*h1 + BM[r+1,c]*h0

//  movq         DWORD PTR T8, mm2              ; T8 <- BM[r+2,c]*h0

//  ;****************************************************************
//  ; Each componenet 16-bit value of the pixel (aRGB) must be      *
//  ; rounded up to the next nearest bit.  This is accomplished by  *
//  ; adding 80H to each 8-bit value.  This effectively adds one    *
//  ; half to each value (mm0.1 binary), since the binary point is    *
//  ; between bit 7 and bit 8.  After rounding, the high-order      *
//  ; eight bits of the 16-bit result are taken; the low-order      *
//  ; eight bits are discarded.                                     *
//  ;                                                               *
//  ;       63  56 55 48 47 40 39 32 31 24 23 16 15  8 7   0        *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;       |  ah |  al |  Rh |  Rl |  Gh |  Gl |  Bh |  Bl |       *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;                                                               *
//  ;       |     +     |     +     |     +     |     +     |       *
//  ;                                                               *
//  ;       63  56 55 48 47 40 39 32 31 24 23 16 15  8 7   0        *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;       |  0  | 0x80|  0  | 0x80|  0  | 0x80|  0  | 0x80|       *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;                                                               *
//  ;       |           |           |           |           |       *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;             |           |           |           |             *
//  ;             |           |           |           +--+          *
//  ;             |           |           +--------+     |          *
//  ;             |           +--------------+     |     |          *
//  ;             +--------------------+     |     |     |          *
//  ;                                  |     |     |     |          *
//  ;                                  V     V     V     V          *
//  ;       63  56 55 48 47 40 39 32 31 24 23 16 15  8 7   0        *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;       |   x |  x  |  x  |  x  |  a  |  R  |  G  |  B  |       *
//  ;       +-----+-----+-----+-----+-----+-----+-----+-----+       *
//  ;                                                               *
//  ;****************************************************************

//  ;****************************************************************
//  ; The pixel values stored in T0, T1, and T2 are completely      *
//  ; calculated (except for round-off).  Add round-off and store   *
//  ; to destination bitmap array.                                  *
//  ;****************************************************************
//  movq         mm4, DWORD PTR rnd_off          ; get the round-off value

//  movq         mm0, DWORD PTR T0              ; mm0 <- BM[r-6,c] (mm64-bits)

//  movq         mm1, DWORD PTR T1              ; mm1 <- BM[r-5,c] (mm64-bits)
//  paddw        mm0, mm4                       ; mm0 <- mm0 + round-off

//  movq         mm2, DWORD PTR T2              ; mm2 <- BM[r-4,c] (mm64-bits)
//  paddw        mm1, mm4                       ; mm1 <- mm1 + round-off

//  paddw        mm2, mm4                        ; mm2 <- mm2 + round-off

//  ;******************************************************
//  ;* Update
//  ;******************************************************

//  psrlw        mm0, 8                         ; rotate each 16-bit component
//  ; of 64-bit result right by
//  ; eight bits (discard low-
//          ; order eight bits)

//  psrlw        mm1, 8                         ; rotate each word component

//  psrlw        mm2, 8                         ; rotate each word component
//  pxor         mm7, mm7                       ; clear mm7

//  packuswb     mm0, mm1                       ; pack high-order byte of each
//  ; word component into 32-bit
//  ; result
//  add          edx, 3                         ; increment the row counter

//  movd        [edi+eax-IMAGE_WIDTH*4*6], mm0  ; store BM[r-6,c] (final result)
//  packuswb     mm2, mm7                       ; pack high-order bytes

//  psrlq        mm0, 32                        ; discard low-order 32-bit word
//  ; (high-order word contains pixel
//     ; value)

//  movd        [edi+eax-IMAGE_WIDTH*4*5], mm0  ; store BM[r-5,c] (final result)

//  movd        [edi+eax-IMAGE_WIDTH*4*4], mm2  ; store BM[r-4,c] (final result)

//  ; finished with rows?
//  cmp          edx, IMAGE_HEIGHT - 1
//  jl           inner_loop

//  ; finished with columns?
//  inc          ecx
//  cmp          ecx, IMAGE_WIDTH
//  jl           col_loop

//  ; release MMx context
//  emms

//  ; return to caller
//  ret    0

//  MMxColFilt ENDP
//  _TEXT    ENDS

//  */
