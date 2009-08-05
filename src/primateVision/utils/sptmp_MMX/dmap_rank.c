
#include "depthmap.h"



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

