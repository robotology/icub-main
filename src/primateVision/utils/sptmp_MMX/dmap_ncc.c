
 // --- dmap_ncc.c ---

#include <assert.h>
#include "sptmpmmx.h"
#include "matrices_cache.h"

#define FAST_SQRT

#ifdef FAST_SQRT
#include "fast_sqrt.h"
#endif

// I1_I2MULTISUM
// I1(x1,y1)*I2(x2,y2) +
// I1(x1+1,y1)*I2(x2+1,y2)
#define I1_I2MULTISUM(I1,I2,x1,y1,x2,y2) \
{\
  movq_m2r(((MM_U_8 *)I1->p[y1].m)[x1],mm2);\
  movq_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm3);\
  movq_r2r(mm2,mm0);\
  movq_r2r(mm3,mm1);\
  punpcklbw_r2r(mm7,mm0);\
  punpcklbw_r2r(mm7,mm1);\
  pmaddwd_r2r(mm1,mm0);\
  psrlq_i2r(8,mm2);\
  psrlq_i2r(8,mm3);\
  punpcklbw_r2r(mm7,mm2);\
  punpcklbw_r2r(mm7,mm3);\
  pmaddwd_r2r(mm3,mm2);\
  punpckldq_r2r(mm2,mm0);\
}

// I1_I2MULTISUM2
// I1(x1,y1)*I2(x2,y2) +
// I1(x1+1,y1)*I2(x2+1,y2) +
// I1(x1+2,y1)*I2(x2+2,y2) +
// I1(x1+3,y1)*I2(x2+3,y2)
#define I1_I2MULTISUM2(I1,I2,x1,y1,x2,y2) \
{\
  movq_m2r(((MM_U_8 *)I1->p[y1].m)[x1],mm0);\
  movq_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm1);\
  movq_r2r(mm0,mm2);\
  movq_r2r(mm1,mm3);\
  punpckhbw_r2r(mm7,mm0);\
  punpckhbw_r2r(mm7,mm1);\
  psllq_i2r(8,mm2);\
  psllq_i2r(8,mm3);\
  pmaddwd_r2r(mm1,mm0);\
  punpckhbw_r2r(mm7,mm2);\
  punpckhbw_r2r(mm7,mm3);\
  pmaddwd_r2r(mm3,mm2);\
  movq_r2r(mm2,mm3);\
  punpckhdq_r2r(mm0,mm2);\
  punpckldq_r2r(mm0,mm3);\
  paddd_r2r(mm3,mm2);\
  movq_r2r(mm2,mm0);\
}



//----------------------------------------------------------------
int Recursive_Depthmap_NCC(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out)
{
  static int allocated = 0;
  int y_max,x_max;

  const int win = 16;
  const int d_max = 64;
  
  short d_2 = d_max/2;


  register int i, j, k;
  register int d,x,y, max_d=0;
  int scale_factor = 256/d_max;

  float C,max=0;

  static int ***P, **Q, **N;
  static int **R, *S, *M;
  static float *MINV;
  

  x_max = I1->cols;
  y_max = I1->rows;
  
  if ( !allocated) {  
    /* Numerator matrix */
    P = (int ***)Alloc(sizeof(int **)*x_max);
    Q = (int **)Alloc(sizeof(int *)*x_max);
    N = (int **)Alloc(sizeof(int *)*x_max);
    for (x=0;x<x_max;x++) {
      Q[x] = (int *)Alloc(sizeof(int)*d_max);
      N[x] = (int *)Alloc(sizeof(int)*d_max);
      P[x] = (int **)Alloc(sizeof(int *)*win);
      for (i=0;i<win;i++) {
  P[x][i] = (int *)Alloc(sizeof(int)*d_max);
      }
    }     
    
    /* Denominator matrix */
    R = (int **)Alloc(sizeof(int *)*x_max);
    for (x=0;x<x_max;x++) {
      R[x] = (int *)Alloc(sizeof(int)*win);
    }     
    S = (int *)Alloc(sizeof(int)*x_max);
    M = (int *)Alloc(sizeof(int)*x_max);
    MINV = (float *) Alloc(sizeof(float)*(x_max)); 
    allocated = 1;
  }
  
  // compute M for x=0 y=0
  // M[0] -> M[d_max]
  // S[0] -> S[d_max+win-2]
  for (k=0;k<=d_max;k++){
    //fprintf(stderr,"depthmap: k:%d \n",k);
    M[k]=0;
    // window across image
    for(i=k;i<k+win;i++) {
      S[i]=0;
      // window down image
      for (j=0;j<win;j++) {
        R[i][j]= ((MM_U_8 *)I2->p[j].m)[i] * ((MM_U_8 *)I2->p[j].m)[i];
        S[i] += R[i][j];
      } 
      M[k] += S[i];
    }
    MINV[k] = 1.0/sqrt(M[k]);
  }
  
  // compute N for x=0 & y=0
  // N->p[0][0]  -> N->p[d_max-1][0]
  for (d=0;d<d_max;d++) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    N[0][d]=0;
    // window across image
    for(i=0;i<=win;i++) {
      Q[i][d] = 0;
      // window down image
      for (j=0;j<win;j++) {
  P[i][j][d]= ((MM_U_8 *)I1->p[j].m)[i+d_2] * ((MM_U_8 *)I2->p[j].m)[i+d];
  Q[i][d] += P[i][j][d];       
      } 
      if (i<win)
  N[0][d] += Q[i][d];
    }
  } // for d
  
  //--- Main y_max loop ---
  for(y=-1;y<y_max-win-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    
    // compute M for x=0    
    if (y>=0) {
      for (k=0;k<=d_max;k++){
  //fprintf(stderr,"depthmap: k:%d \n",k);
  M[k]=0;
  // window across image
  for(i=k;i<k+win;i++) {
    S[i] -= R[i][y%win];
    R[i][y%win] = ((MM_U_8 *)I2->p[y+win].m)[i] * ((MM_U_8 *)I2->p[y+win].m)[i];
    S[i] += R[i][y%win];  
    M[k] += S[i];
  }
  MINV[k]= 1.0/sqrt(M[k]);
      } // for k
    } // if y >= 0
    
    //--- Main X loop ---
    for(x=-1;x<x_max-win-d_max-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      // compute M for y=0 
      if (y<0) {
  S[x+d_max+win] = 0;
  for (j=0;j<win;j++) {
    R[x+d_max+win][j]= ((MM_U_8 *)I2->p[j].m)[x+d_max+win] * ((MM_U_8 *)I2->p[j].m)[x+d_max+win];
    S[x+d_max+win] += R[x+d_max+win][j];
  }
      }  
      else
      {
  // P is [x_max][win] the win dimension is a wrap around buffer.    
  S[x+d_max+win] -= R[x+d_max+win][y%win];
  R[x+d_max+win][y%win] = ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win] * ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win];
  S[x+d_max+win] += R[x+d_max+win][y%win];
      }
      M[x+d_max+1] = M[x+d_max] - S[x+d_max] + S[x+d_max+win];
      MINV[x+d_max+1]= 1.0/sqrt(M[x+d_max+1]);
            
      //--- Main D loop ---
      for (d=0;d<d_max;d++) {  
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if (x<0) {
    if (y>=0) {
      // Equivalent C function
      N[0][d]=0;
      for(i=0;i<win;i++) {
        Q[i][d] -= P[i][y%win][d];
        P[i][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[i+d_2] * ((MM_U_8 *)I2->p[y+win].m)[i+d];
        Q[i][d] += P[i][y%win][d];
        N[0][d] += Q[i][d];
      }
      Q[win][d] -= P[win][y%win][d];
      P[win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[win+d_2] * ((MM_U_8 *)I2->p[y+win].m)[win+d];
      Q[win][d] += P[win][y%win][d];    
    } // y>=0
  } // x <0
  else
  {
    if (y<0) {
      Q[x+win][d] = 0;
      for (j=0;j<win;j++) {
        P[x+win][j][d]= ((MM_U_8 *)I1->p[j].m)[x+win+d_2] * ((MM_U_8 *)I2->p[j].m)[x+win+d];
        Q[x+win][d] += P[x+win][j][d];
      }
      
    } // y<0 
    else {        
      // P is [x_max][win][d_max] the win dimension is a wrap around buffer.
      Q[x+win][d] -= P[x+win][y%win][d];
      P[x+win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[x+win+d_2] * ((MM_U_8 *)I2->p[y+win].m)[x+win+d];
      Q[x+win][d] += P[x+win][y%win][d];
      
    } // y>=0
    
    N[x+1][d] = N[x][d] - Q[x][d] + Q[x+win][d];
    
  } // x>=0
  C = N[x+1][d]*MINV[x+d+1];
  if ((d==0)||(C > max)) {
    max = C;
    max_d = d;
  }  
      }// for d          
     ((MM_U_8 *)Out->p[y+1].m)[x+1] = max_d*scale_factor;      
    }// for x
  }// for y

  if (allocated) {
    /* Numerator matrix */
    for (x=0;x<x_max;x++) {
      for (i=0;i<win;i++) {
  free(P[x][i]);
      }
      free(P[x]);
      free(Q[x]);
      free(N[x]);
    }     
    free(P);
    free(Q);
    free(N);
    
    /* Denominator matrix */
    for (x=0;x<x_max;x++) {
      free(R[x]);
    }     
    free(R);
    free(S);
    free(M);
    free(MINV);
  }
  return 0;
}






//#define MAX_THRESH 2048

//----------------------------------------------------------------
int MMXRecursive_Depthmap_NCC(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out)
{

  static int allocated = 0;
  int y_max;
  int x_max;

  const int win = 16;
  const int d_max = 64;

  //  short d_2 = d_max/2;
  short d_2 = 0;


  register int i, j, k;
  register int d,x,y;

  register int y_win, x_win, yMODwin;

#ifdef FAST_SQRT
  unsigned long sqrt2,sqrt3;
#endif

  mmx_t curr, curr_d, max, max_d;

  static  MMXMatrix **P, *Q, *N;
  static MMXMatrix *R;
  static MM_U_32 *S, *M, *MINV;

  x_max = I1->cols;
  y_max = I1->rows;
  

  if ( !allocated) {
    // Numerator matrix
    P = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*win); 
    for (i=0;i<win;i++)
      P[i] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
    Q = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
    N = MMXMatrixAlloc(MMT_U_32, x_max, d_max);

    // Denominator matrix
    S = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
    //M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
    R = MMXMatrixAlloc(MMT_U_32, x_max, win);
    M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
    MINV = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
    allocated = 1;
  }

  emms();
  // clear reg 7
  pxor_r2r (mm7,mm7);
 
  // compute M for x=0 y=0
  // M[0] -> M[d_max]
  // S[0] -> S[d_max+win-2]
  for (k=0;k<=d_max;k+=2){
    //fprintf(stderr,"depthmap: k:%d \n",k);
    //M[k]=0;
    //// window across image
    //for(i=k;i<k+win;i++) {
    //  S[i]=0;
    //  // window down image
    //  for (j=0;j<win;j++) {
    //    R[i][j]= ((MM_U_8 *)I2->p[j].m)[i] * ((MM_U_8 *)I2->p[j].m)[i];
    //    S[i] += R[i][j];
    //  } 
    //  M[k] += S[i];
    //}

    // reg 6 = M
    pxor_r2r (mm6,mm6);
    for(i=k;i<k+win;i+=2) {
      // reg 5 = S
      pxor_r2r (mm5,mm5);
      for(j=0;j<win;j++) {
  // calc R
  I1_I2MULTISUM(I2, I2, i,j,i,j);
  // Store new R
  movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[i]);
  // Add R to S
  paddd_r2r (mm0,mm5);
      }
      // Store new S
      movq_r2m (mm5,((MM_U_32 *)S)[i]);
      // Add curr S to M
      paddd_r2r (mm5,mm6);
    }
    // Store M[0]
    movq_r2m (mm6,((MM_U_32 *)M)[k]);    

#ifdef FAST_SQRT
    sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
    sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
    ((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
    ((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
#else
    ((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k]))/((MM_U_32 *)M)[k]);
    ((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k+1]))/((MM_U_32 *)M)[k+1]);
#endif
  }

  // compute N for x=0 & y=0
  // N->p[0][0]  -> N->p[d_max-1][0]
  for (d=0;d<d_max;d++) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    //N[0][d]=0;
    //// window across image
    //for(i=0;i<=win;i++) {
    //  Q[i][d] = 0;
    //  // window down image
    // for (j=0;j<win;j++) {
    //   P[i][j][d]= ((MM_U_8 *)I1->p[j].m)[i+d_2] * ((MM_U_8 *)I2->p[j].m)[i+d];
    //   Q[i][d] += P[i][j][d];       
    // } 
    // if (i<win)
    //   N[0][d] += Q[i][d];
    //}

    // reg 6 = N
    pxor_r2r (mm6,mm6);
    for(i=0;i<win;i+=2) {
      // reg 5 = Q
      pxor_r2r (mm5,mm5);
      for (j=0;j<win;j++) {
  // calc new P   
  I1_I2MULTISUM(I1, I2, i+d_2,j,i+d,j);
  // Store P
  movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[i]);
  // Add P to Q
  paddd_r2r (mm0,mm5);
      }
      // Store Q
      movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
      // Add curr Q to N
      paddd_r2r (mm5,mm6);
    }
    // Store N[d][0]
    movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);
    
    // Do for i=win
    // reg 5 = Q
    pxor_r2r (mm5,mm5);
    for (j=0;j<win;j++) {
      // calc new P  
      I1_I2MULTISUM(I1, I2, win+d_2,j,win+d,j);
      // Store P
      movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[win]);
      // Add P to Q
      paddd_r2r (mm0,mm5);
    }
    // Store Q
    movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[win]);
  } // for d

  //--- Main y_max loop ---
  for(y=-1;y<y_max-win-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    y_win = y+win;
    yMODwin = y%win;


    // compute M for x=0    
    if (y>=0) {
      for (k=0;k<=d_max;k+=2){
  //fprintf(stderr,"depthmap: k:%d \n",k);
  // M[k]=0;
  // window across image
  //for(i=k;i<k+win;i++) {
  //  S[i] -= R[i][y%win];
  //  R[i][y%win] = ((MM_U_8 *)I2->p[y+win].m)[i] * ((MM_U_8 *)I2->p[y+win].m)[i];
  //  S[i] += R[i][y%win];  
  //  M[k] += S[i];
  //}
  // reg 6&6 = M
  pxor_r2r (mm6,mm6);
  for(i=k;i<k+win;i+=2) {
    // reg 5 & 4 = S
    movq_m2r (((MM_U_32 *)S)[i],mm5);  
    // Sub prev R from S
    psubd_m2r (((MM_U_32 *)R->p[yMODwin].m)[i],mm5);
    // calc new R
    I1_I2MULTISUM(I2, I2, i,y_win,i,y_win);
    // Store R
    movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[i]);
    // Add R to S
    paddd_r2r (mm0,mm5);
    // Store S
    movq_r2m (mm5,((MM_U_32 *)S)[i]);
    // Add curr S to M
    paddd_r2r (mm5,mm6);
  } // for i
  // Store M[0]
  movq_r2m (mm6,((MM_U_32 *)M)[k]);
#ifdef FAST_SQRT
  sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
  sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
  ((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
  ((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
#else
  ((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k]))/((MM_U_32 *)M)[k]);
  ((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k+1]))/((MM_U_32 *)M)[k+1]);
#endif
      } // for k
    } // if y >= 0

    //--- Main X loop ---
    for(x=-2;x<x_max-win-d_max-2;x+=2) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      x_win= x+win;
   
      // compute M for y=0 
      if (y<0) {
  // Equivalent C function
  //S[x+d_max+win] = 0;
  //for (j=0;j<win;j++) {
  //  R[x+d_max+win][j]= ((MM_U_8 *)I2->p[j].m)[x+d_max+win] * ((MM_U_8 *)I2->p[j].m)[x+d_max+win];
  //  S[x+d_max+win] += R[x+d_max+win][j];
  //}
  // reg 5 = S
  pxor_r2r (mm5,mm5);
  // window down image
  for (j=0;j<win;j++) {
    //fprintf(stderr,"depthmap: j:%d\n",j);
    // Calc P
    I1_I2MULTISUM(I2, I2,x_win+d_max,j,x_win+d_max,j);
    // Store R
    movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[x_win+d_max]);
    // Add curr R to S
    paddd_r2r (mm0,mm5);
  }
  // Store S
  movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max]);

      }  
      else
      {
  // Equivalent C function
  // P is [x_max][win] the win dimension is a wrap around buffer.    
  //S[x+d_max+win] -= R[x+d_max+win][y%win];
  //R[x+d_max+win][y%win] = ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win] * ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win];
  //S[x+d_max+win] += R[x+d_max+win][y%win];
  
  // reg 5 = S
  movq_m2r (((MM_U_32 *)S)[x_win+d_max],mm5);
  // subtract prev R
  psubd_m2r (((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max],mm5);
  // Add curr R to S
  I1_I2MULTISUM(I2, I2,x_win+d_max,y_win,x_win+d_max,y_win);
  paddd_r2r (mm0,mm5);
  // Store S
  movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max]);
  // Store R
  movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max]);
      }
  
      // Equivalent C function
      //M[x+d_max+1] = M[x+d_max] - S[x+d_max] + S[x+d_max+win];
      // reg 6 & 6 = M
      movq_m2r (((MM_U_32 *)M)[x+d_max],mm6);  
      // Sub S[x+d_max] from M
      psubd_m2r (((MM_U_32 *)S)[x+d_max],mm6);
      // Add S[x+d_max+win] to M
      movq_m2r (((MM_U_32 *)S)[x_win+d_max],mm1);
      paddd_r2r (mm1,mm6);
      // Store M[x+1][d]
      movq_r2m (mm6,((MM_U_32 *)M)[x+d_max+2]);
#ifdef FAST_SQRT
      sqrt2 = fast_sqrt(((MM_U_32 *)M)[x+d_max+2]);
      sqrt3 = fast_sqrt(((MM_U_32 *)M)[x+d_max+3]);
      ((MM_U_32 *)MINV)[x+d_max+2] = ((((unsigned long long)0xffffffff) + sqrt2 )/ sqrt2);
      ((MM_U_32 *)MINV)[x+d_max+3] = ((((unsigned long long)0xffffffff) + sqrt3 )/ sqrt3);
#else
      ((MM_U_32 *)MINV)[x+d_max+2] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+d_max+2])/((MM_U_32 *)M)[x+d_max+2]);
      ((MM_U_32 *)MINV)[x+d_max+3] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+d_max+3])/((MM_U_32 *)M)[x+d_max+3]);
#endif
      //printf("%d %d %ld %ld\n",y,x,((MM_U_32 *)M)[x+2],((MM_U_32 *)M)[x+3]);
      
      max_d.ud[0] = 0;
      max_d.ud[1] = 0;    

      max.ud[0] = 0;
      max.ud[1] = 0;    

      
      //--- Main D loop ---
      for (d=0;d<d_max;d++) {
    
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if (x<0) {
    if (y>=0) {
      // Equivalent C function
      //N[0][d]=0;
      //for(i=0;i<win;i++) {
      //  Q[i][d] -= P[i][y%win][d];
      //  P[i][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[i+d_2] * ((MM_U_8 *)I2->p[y+win].m)[i+d];
      //  Q[i][d] += P[i][y%win][d];
      //  N[0][d] += Q[i][d];
      //}
      //Q[win][d] -= P[win][y%win][d];
      //P[win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[win+d_2] * ((MM_U_8 *)I2->p[y+win].m)[win+d];
      //Q[win][d] += P[win][y%win][d];
    
      // reg 6&6 = N
      pxor_r2r (mm6,mm6);
      for(i=0;i<win;i+=2) {
        // reg 5 & 4 = Q
        movq_m2r (((MM_U_32 *)Q->p[d].m)[i],mm5);  
        // Sub prev P from Q
        psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[i],mm5);
        // Add curr P to Q
        I1_I2MULTISUM(I1, I2, i+d_2,y_win,i+d,y_win);
        // Add new P
        paddd_r2r (mm0,mm5);
        // Store Q
        movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
        // Store P
        movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
        // Add curr Q to N
        paddd_r2r (mm5,mm6);
      } // for i
      // Store N[d][0]
      movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);

      // Do for i=win
      // reg 5 & 4 = Q
      movq_m2r (((MM_U_32 *)Q->p[d].m)[i],mm5);  
      // Sub prev P from Q
      psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[i],mm5);
      // Add curr P to Q
      // lbw in 0
      I1_I2MULTISUM(I1, I2, i+d_2,y_win,i+d,y_win);
      // Add new P
      paddd_r2r (mm0,mm5);
      // Store Q
      movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
      // Store P
      movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
    } // y>=0
  } // x <0
  else
  {
    if (y<0) {
      // Equivalent C function
      //Q[x+win][d] = 0;
      //for (j=0;j<win;j++) {
      //  P[x+win][j][d]= ((MM_U_8 *)I1->p[j].m)[x+win+d_2] * ((MM_U_8 *)I2->p[j].m)[x+win+d];
      //  Q[x+win][d] += P[x+win][j][d];
      //}

      // reg 5&4 = Q
      pxor_r2r (mm5,mm5);
      
      // window down image
      for (j=0;j<win;j++) {
        I1_I2MULTISUM(I1, I2,x_win+d_2,j,x_win+d,j);
        movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[x_win]);
        // Add curr P to Q
        paddd_r2r (mm0,mm5);
      }
      // Store Q
      movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);
      
    } // y<0 
    else
    {
        
      // Equivalent C function
      // P is [x_max][win][d_max] the win dimension is a wrap around buffer.
      //Q[x+win][d] -= P[x+win][y%win][d];
      //P[x+win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[x+win+d_2] * ((MM_U_8 *)I2->p[y+win].m)[x+win+d];
      //Q[x+win][d] += P[x+win][y%win][d];
      
      // reg 5&4 = Q
      movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm5);
      // subtract prev P
      psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[x+win],mm5);
      // Add curr P to Q
      I1_I2MULTISUM(I1, I2,x_win+d_2,y_win,x_win+d,y_win);
      paddd_r2r (mm0,mm5);
        
      // Store Q
      movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);
      
      // Store P
      movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[x_win]);
      
      } // y>=0
        
    // Equivalent C function
    //N[x+1][d] = N[x][d] - Q[x][d] + Q[x+win][d];
    
    // reg 6 & 6 = N
    movq_m2r (((MM_U_32 *)N->p[d].m)[x],mm6);  

    // Sub curr Q[x][d] from N
    psubd_m2r (((MM_U_32 *)Q->p[d].m)[x],mm6);
      
    // Add Q[x+win][d] to N
    movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm1);
    paddd_r2r (mm1,mm6);
      
    // Store N[x+1][d]
    movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[x+2]);
    
  } // x>=0
  // Equivalent C function              
  //C1 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
  //if (C1 > max1) {
  //   max1 = C1;
  //   max1_d = d;
  //}
  //C2 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);
  //if (C2 > max2) {
  //  max2 = C2;
  //  max2_d = d;
  //}
  
#ifdef FAST_SQRT
  curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
  curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#else
  curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
  curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#endif
    
  
  //curr.ud[0] = ((MM_U_32 *)N->p[d].m)[x+2]/fast_sqrt(((MM_U_32 *)M)[x+d+2]);
  //curr.ud[1] = ((MM_U_32 *)N->p[d].m)[x+3]/fast_sqrt(((MM_U_32 *)M)[x+d+3]);
  

  //curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
  //curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);
  
  // reg 5 = curr 
  movq_m2r(curr,mm5);
        movq_r2r(mm5,mm0);
  //PRINTREG32(mm5,"curr");

  // reg 6 = max
        movq_m2r(max,mm6);
  movq_r2r(mm6,mm2);
  //PRINTREG32(mm6,"max");
        
  // reg 2 becomes mask
        pcmpgtd_r2r(mm0,mm2);
        movq_r2r(mm2,mm1);
  //PRINTREG32(mm2,"mask");

        // reg 0 is preserved values larger than max
  movq_r2r(mm6,mm0);
  pand_r2r(mm2,mm0);
  
  // reg 1 is curr
  pandn_r2r(mm5,mm1);
  
  // reg 1 is new max
  por_r2r(mm0,mm1);
  movq_r2m(mm1,max);
  //PRINTREG32(mm1,"new_max");

  // reg 1 is preserved d values
  movq_m2r(max_d,mm3);
        movq_r2r(mm2,mm1);
  pand_r2r(mm3,mm1);

  // reg 0 = current d value
  curr_d.ud[0] = d;
  curr_d.ud[1] = d;    

  movq_m2r(curr_d,mm4);
  pandn_r2r(mm4,mm2);
   
  // reg 6 = new d's for minimum values
  por_r2r(mm2,mm1);
  movq_r2m(mm1,max_d);
  //PRINTREG32(mm1,"new_max_d");
    
  
      }// for d          
      //printf("%u\n",max.ud[0]);
      //printf("%u\n",max.ud[1]);
      // if (max.ud[0]<MAX_THRESH)
  ((MM_U_8 *)Out->p[y+1].m)[x+2] = max_d.ub[0]<<2;
  //else
  //((MM_U_8 *)Out->p[y+1].m)[x+2] = 0;
  //if (max.ud[1]<MAX_THRESH)
  ((MM_U_8 *)Out->p[y+1].m)[x+3] = max_d.ub[4]<<2;
  //else
  //((MM_U_8 *)Out->p[y+1].m)[x+3] = 0;

      
    }// for x
  }// for y

  emms();
  return 0;
}



//----------------------------------------------------------------
//----------------------------------------------------------------
//----------------------------------------------------------------
//----------------------------------------------------------------

// Equivalent C function
// Q(x,y+1,d) = Q(x,y,d) - P(x,y) + P(x,y+win)
// assumes new P is in reg 0
// Q is in mm5 for later use
#define Qinc(Q,P,x,y,d) \
{\
      /*  reg 5&4 = Q */ \
      movq_m2r (((MM_U_32 *)Q->p[d].m)[x],mm5);\
      paddd_r2r (mm0,mm5);\
            /* subtract prev P */ \
      psubd_m2r (((MM_U_32 *)P[y]->p[d].m)[x],mm5);\
            /* Store Q */ \
      movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x]);\
}

// Equivalent C function
//N[x+1][d] = N[x][d] - Q[x][d] + Q[x+win][d];
#define Ninc(N,x,d,x_win) \
{\
  /* reg 6 & 6 = N */ \
  movq_m2r (((MM_U_32 *)N->p[d].m)[x],mm6);\
  /* Sub curr Q[x][d] from N */ \
  psubd_m2r (((MM_U_32 *)Q->p[d].m)[x],mm6);\
  /* Add Q[x+win][d] to N */ \
  movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm1);\
  paddd_r2r (mm1,mm6);\
  /* Store N[x+1][d] */ \
  movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[x+2]);\
}

// Equivalent C function
// S(x,y+1) = S(x,y) - R(x,y) + R(x,y+win)
// assumes new R is in reg 0
// S is in mm5 for later use
#define Sinc(S,R,x,y) \
{\
      /*  reg 5 = S */ \
      movq_m2r (((MM_U_32 *)S)[x],mm5);\
            /* Add R to S */ \
            paddd_r2r (mm0,mm5);\
            /* subtract prev R */ \
      psubd_m2r (((MM_U_32 *)R->p[y].m)[x],mm5);\
            /* Store Q */ \
      movq_r2m (mm5,((MM_U_32 *)S)[x]);\
}

// Equivalent C function
//M[x+1][y] = M[x][y] - S[x][y] + S[x+win][y];
#define Minc(M,S,x,x_win) \
{\
  /* reg 6 & 6 = M */ \
  movq_m2r (((MM_U_32 *)M)[x],mm6);\
  /* Sub curr S[x][y] from M */ \
  psubd_m2r (((MM_U_32 *)S)[x],mm6);\
  /* Add S[x+win][y] to M */ \
  movq_m2r (((MM_U_32 *)S)[x_win],mm1);\
  paddd_r2r (mm1,mm6);\
  /* Store M[x+1][y] */ \
  movq_r2m (mm6,((MM_U_32 *)M)[x+2]);\
}


int MMXRecursive_Depthmap_NCC2(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out)
{

  static int allocated = 0;
  int y_max;
  int x_max;

  const int win = 16;
  const int d_max = 128;

  short d_2 = d_max/2;


  register int i, j, k;
  register int d,x,y;

  register int y_win, x_win, yMODwin;

  mmx_t curr, curr_d, max, max_d;
  sse_t fcurr;
  static  MMXMatrix **P, *Q, *N;
  static MMXMatrix *R;
  static MM_U_32 *S, *M;
  //static MMXMatrix *sse_tmp;
  static MM_F_32 *MFINV;
  
  x_max = I1->cols;
  y_max = I1->rows;
  

  if ( !allocated) {
    // Numerator matrix
    P = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*win); 
    for (i=0;i<win;i++)
      P[i] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
    Q = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
    N = MMXMatrixAlloc(MMT_U_32, x_max, d_max);

    // Denominator matrix
    S = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
    M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
    R = MMXMatrixAlloc(MMT_U_32, x_max, win);
    //MINV = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
    //sse_tmp = MMXMatrixAlloc(MMT_F_32, 1, x_max);
    //MFINV = (MM_F_32 *)sse_tmp->p[0].s;
    MFINV = (MM_F_32 *) Alloc(sizeof(MM_F_32)*(x_max)); 

    allocated = 1;
  }

  emms();
  // clear reg 7
  pxor_r2r (mm7,mm7);
 
  // compute M for x=0 y=0
  // M[0] -> M[d_max]
  // S[0] -> S[d_max+win-2]
  for (k=0;k<=d_max;k+=2){
    //fprintf(stderr,"depthmap: k:%d \n",k);
    //M[k]=0;
    //// window across image
    //for(i=k;i<k+win;i++) {
    //  S[i]=0;
    //  // window down image
    //  for (j=0;j<win;j++) {
    //    R[i][j]= ((MM_U_8 *)I2->p[j].m)[i] * ((MM_U_8 *)I2->p[j].m)[i];
    //    S[i] += R[i][j];
    //  } 
    //  M[k] += S[i];
    //}

    // reg 6 = M
    pxor_r2r (mm6,mm6);
    for(i=k;i<k+win;i+=2) {
      // reg 5 = S
      pxor_r2r (mm5,mm5);
      for(j=0;j<win;j++) {
  // calc R
  I1_I2MULTISUM(I2, I2, i,j,i,j);
  // Store new R
  movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[i]);
  // Add R to S
  paddd_r2r (mm0,mm5);
      }
      // Store new S
      movq_r2m (mm5,((MM_U_32 *)S)[i]);
      // Add curr S to M
      paddd_r2r (mm5,mm6);
    }
    // Store M[0]
    movq_r2m (mm6,((MM_U_32 *)M)[k]);    

    //sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
    //sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
    //((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
    //((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
    cvtpi2ps_r2r(mm6, xmm6);
    rsqrtps_r2r(xmm6,xmm6);
    // Store MFINV[0]
    movlps_r2m(xmm6,((MM_F_32 *)MFINV)[k]);    
    
  }

  // compute N for x=0 & y=0
  // N->p[0][0]  -> N->p[d_max-1][0]
  for (d=0;d<d_max;d++) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    //N[0][d]=0;
    //// window across image
    //for(i=0;i<=win;i++) {
    //  Q[i][d] = 0;
    //  // window down image
    // for (j=0;j<win;j++) {
    //   P[i][j][d]= ((MM_U_8 *)I1->p[j].m)[i+d_2] * ((MM_U_8 *)I2->p[j].m)[i+d];
    //   Q[i][d] += P[i][j][d];       
    // } 
    // if (i<win)
    //   N[0][d] += Q[i][d];
    //}

    // reg 6 = N
    pxor_r2r (mm6,mm6);
    for(i=0;i<win;i+=2) {
      // reg 5 = Q
      pxor_r2r (mm5,mm5);
      for (j=0;j<win;j++) {
  // calc new P   
  I1_I2MULTISUM(I1, I2, i+d_2,j,i+d,j);
  // Store P
  movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[i]);
  // Add P to Q
  paddd_r2r (mm0,mm5);
      }
      // Store Q
      movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
      // Add curr Q to N
      paddd_r2r (mm5,mm6);
    }
    // Store N[d][0]
    movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);
    
    // Do for i=win
    // reg 5 = Q
    pxor_r2r (mm5,mm5);
    for (j=0;j<win;j++) {
      // calc new P  
      I1_I2MULTISUM(I1, I2, win+d_2,j,win+d,j);
      // Store P
      movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[win]);
      // Add P to Q
      paddd_r2r (mm0,mm5);
    }
    // Store Q
    movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[win]);
  } // for d

  //--- Main y_max loop ---
  for(y=-1;y<y_max-win-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    y_win = y+win;
    yMODwin = y%win;


    // compute M for x=0    
    if (y>=0) {
      for (k=0;k<=d_max;k+=2){
  //fprintf(stderr,"depthmap: k:%d \n",k);
  // M[k]=0;
  // window across image
  //for(i=k;i<k+win;i++) {
  //  S[i] -= R[i][y%win];
  //  R[i][y%win] = ((MM_U_8 *)I2->p[y+win].m)[i] * ((MM_U_8 *)I2->p[y+win].m)[i];
  //  S[i] += R[i][y%win];  
  //  M[k] += S[i];
  //}
  // reg 6&6 = M
  pxor_r2r (mm6,mm6);
  for(i=k;i<k+win;i+=2) {
    // calc new R
    I1_I2MULTISUM(I2, I2, i,y_win,i,y_win);
    Sinc(S,R,i,yMODwin);
    // Store R
    movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[i]);
    // Add curr S to M
    paddd_r2r (mm5,mm6);
  } // for i
  // Store M[0]
  movq_r2m (mm6,((MM_U_32 *)M)[k]);

  //sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
  //sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
  //((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
  //((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);

  cvtpi2ps_r2r(mm6, xmm6);
  rsqrtps_r2r(xmm6,xmm6);
  // Store MFINV[0]
  movlps_r2m(xmm6,((MM_F_32 *)MFINV)[k]);    

      } // for k
    } // if y >= 0

    //--- Main X loop ---
    for(x=-2;x<x_max-win-d_max-2;x+=2) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      x_win= x+win;
   
      // compute M for y=0 
      if (y<0) {
  // Equivalent C function
  //S[x+d_max+win] = 0;
  //for (j=0;j<win;j++) {
  //  R[x+d_max+win][j]= ((MM_U_8 *)I2->p[j].m)[x+d_max+win] * ((MM_U_8 *)I2->p[j].m)[x+d_max+win];
  //  S[x+d_max+win] += R[x+d_max+win][j];
  //}
  // reg 5 = S
  pxor_r2r (mm5,mm5);
  // window down image
  for (j=0;j<win;j++) {
    //fprintf(stderr,"depthmap: j:%d\n",j);
    // Calc R
    I1_I2MULTISUM(I2, I2,x_win+d_max,j,x_win+d_max,j);
    // Store R
    movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[x_win+d_max]);
    // Add curr R to S
    paddd_r2r (mm0,mm5);
  }
  // Store S
  movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max]);

      }  
      else
      {
  // Equivalent C function
  // P is [x_max][win] the win dimension is a wrap around buffer.    
  //S[x+d_max+win] -= R[x+d_max+win][y%win];
  //R[x+d_max+win][y%win] = ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win] * ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win];
  //S[x+d_max+win] += R[x+d_max+win][y%win];

  I1_I2MULTISUM(I2, I2,x_win+d_max,y_win,x_win+d_max,y_win);
  Sinc(S,R,x_win+d_max,yMODwin);
  // Store R
  movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max]);
      }
  
      // Equivalent C function
      //M[x+d_max+1] = M[x+d_max] - S[x+d_max] + S[x+d_max+win];
      Minc(M,S,x+d_max,x_win+d_max);

      //sqrt2 = fast_sqrt(((MM_U_32 *)M)[x+d_max+2]);
      //sqrt3 = fast_sqrt(((MM_U_32 *)M)[x+d_max+3]);
      //((MM_U_32 *)MINV)[x+d_max+2] = ((((unsigned long long)0xffffffff) + sqrt2 )/ sqrt2);
      //((MM_U_32 *)MINV)[x+d_max+3] = ((((unsigned long long)0xffffffff) + sqrt3 )/ sqrt3);

      cvtpi2ps_r2r(mm6, xmm6);
      rsqrtps_r2r(xmm6,xmm6);
      // Store MFINV[0]
      movlps_r2m(xmm6,((MM_F_32 *)MFINV)[k]);    


      //printf("%d %d %ld %ld\n",y,x,((MM_U_32 *)M)[x+2],((MM_U_32 *)M)[x+3]);
      
      max_d.ud[0] = 0;
      max_d.ud[1] = 0;    

      max.ud[0] = 0;
      max.ud[1] = 0;    

      //--- Main D loop ---
      for (d=0;d<d_max;d++) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if (x<0) {
    if (y>=0) {
      // Equivalent C function
      //N[0][d]=0;
      //for(i=0;i<win;i++) {
      //  Q[i][d] -= P[i][y%win][d];
      //  P[i][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[i+d_2] * ((MM_U_8 *)I2->p[y+win].m)[i+d];
      //  Q[i][d] += P[i][y%win][d];
      //  N[0][d] += Q[i][d];
      //}
      //Q[win][d] -= P[win][y%win][d];
      //P[win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[win+d_2] * ((MM_U_8 *)I2->p[y+win].m)[win+d];
      //Q[win][d] += P[win][y%win][d];
    
      // reg 6&6 = N
      pxor_r2r (mm6,mm6);
      for(i=0;i<win;i+=2) {
        I1_I2MULTISUM(I1, I2, i+d_2,y_win,i+d,y_win);
        Qinc(Q,P,i,yMODwin,d);
        // Store P
        movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
        // Add curr Q to N
        paddd_r2r (mm5,mm6);
      } // for i
      // Store N[d][0]
      movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);

      // Do for i=win

      // lbw in 0
      I1_I2MULTISUM(I1, I2, i+d_2,y_win,i+d,y_win);
            Qinc(Q,P,i,yMODwin,d);
      // Store new P
      movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
    } // y>=0
  } // x <0
  else
  {
    if (y<0) {
      // Equivalent C function
      //Q[x+win][d] = 0;
      //for (j=0;j<win;j++) {
      //  P[x+win][j][d]= ((MM_U_8 *)I1->p[j].m)[x+win+d_2] * ((MM_U_8 *)I2->p[j].m)[x+win+d];
      //  Q[x+win][d] += P[x+win][j][d];
      //}

      // reg 5&4 = Q
      pxor_r2r (mm5,mm5);
      
      // window down image
      for (j=0;j<win;j++) {
        I1_I2MULTISUM(I1, I2,x_win+d_2,j,x_win+d,j);
        movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[x_win]);
        // Add curr P to Q
        paddd_r2r (mm0,mm5);
      }
      // Store Q
      movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);
      
    } // y<0 
    else
    {
      // Equivalent C function
      // P is [x_max][win][d_max] the win dimension is a wrap around buffer.
      //Q[x+win][d] -= P[x+win][y%win][d];
      //P[x+win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[x+win+d_2] * ((MM_U_8 *)I2->p[y+win].m)[x+win+d];
      //Q[x+win][d] += P[x+win][y%win][d];

      // Add curr P to Q
      I1_I2MULTISUM(I1, I2,x_win+d_2,y_win,x_win+d,y_win);

            Qinc(Q,P,x_win,yMODwin,d);
      
      // Store P
      movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[x_win]);

    } // y>=0

          Ninc(N,x,d,x_win);
    
  } // x>=0
  // Equivalent C function              
  //C1 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
  //if (C1 > max1) {
  //   max1 = C1;
  //   max1_d = d;
  //}
  //C2 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);
  //if (C2 > max2) {
  //  max2 = C2;
  //  max2_d = d;
  //}

  cvtpi2ps_r2r(mm6, xmm5);
  movups_m2r(((MM_F_32 *)MFINV)[x+d+2],xmm0);      
  movhlps_r2r(xmm0,xmm0);      
  mulps_r2r(xmm0,xmm5);      

  movlps_r2m(xmm5,fcurr);      

  //cvtps2pi_r2r(xmm5, mm5);

  curr.ud[0] = (unsigned long) ((MM_F_32 *)fcurr.sf)[2];
  curr.ud[1] = (unsigned long) ((MM_F_32 *)fcurr.sf)[3];
  
  // reg 5 = curr 
  movq_m2r(curr,mm5);
        movq_r2r(mm5,mm0);
  //PRINTREG32(mm5,"curr");

  // reg 6 = max
        movq_m2r(max,mm6);
  movq_r2r(mm6,mm2);
  //PRINTREG32(mm6,"max");
        
  // reg 2 becomes mask
        pcmpgtd_r2r(mm0,mm2);
        movq_r2r(mm2,mm1);
  //PRINTREG32(mm2,"mask");

        // reg 0 is preserved values larger than max
  movq_r2r(mm6,mm0);
  pand_r2r(mm2,mm0);
  
  // reg 1 is curr
  pandn_r2r(mm5,mm1);
  
  // reg 1 is new max
  por_r2r(mm0,mm1);
  movq_r2m(mm1,max);
  //PRINTREG32(mm1,"new_max");

  // reg 1 is preserved d values
  movq_m2r(max_d,mm3);
        movq_r2r(mm2,mm1);
  pand_r2r(mm3,mm1);

  // reg 0 = current d value
  curr_d.ud[0] = d;
  curr_d.ud[1] = d;    

  movq_m2r(curr_d,mm4);
  pandn_r2r(mm4,mm2);
   
  // reg 6 = new d's for minimum values
  por_r2r(mm2,mm1);
  movq_r2m(mm1,max_d);
  //PRINTREG32(mm1,"new_max_d");


      }// for d            
      ((MM_U_8 *)Out->p[y+1].m)[x+2] = max_d.ub[0];
      ((MM_U_8 *)Out->p[y+1].m)[x+3] = max_d.ub[4];
      
      //emms();
      // clear reg 7
      //pxor_r2r (mm7,mm7);

    }// for x
  }// for y

  emms();
  return 0;
}


int MMXRecursive_Depthmap_NCC4(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, int offset)
{
	static int allocated = 0;
	int y_max;
	int x_max;

	const int win = 16;
	const int d_max = 64;
	const int E = 0;

	//short d_2 = 0;


	register int i, j, k;
	register int d,x,y;

	register int y_win, x_win, yMODwin;

#ifdef FAST_SQRT
	unsigned long sqrt2,sqrt3;
#endif

	mmx_t curr, curr_d, max, max_d;

	static  MMXMatrix **P, *Q, *N;
	static MMXMatrix *R;
	static MM_U_32 *S, *M, *MINV;

	x_max = I1->cols;
	y_max = I1->rows;


	if ( !allocated) {
		// Numerator matrix
		P = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*win); 
		for (i=0;i<win;i++)
			P[i] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		Q = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		N = MMXMatrixAlloc(MMT_U_32, x_max, d_max);

		// Denominator matrix
		S = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
		//M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
		R = MMXMatrixAlloc(MMT_U_32, x_max, win);
		M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
		MINV = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
		allocated = 1;
	}

	emms();
	// clear reg 7
	pxor_r2r (mm7,mm7);

	// compute M for x=0 y=0
	// M[0] -> M[d_max]
	// S[0] -> S[d_max+win-2]
	for (k=0;k<=d_max;k+=2){
		//fprintf(stderr,"depthmap: k:%d \n",k);
		//M[k]=0;
		//// window across image
		//for(i=k;i<k+win;i++) {
		//  S[i]=0;
		//  // window down image
		//  for (j=0;j<win;j++) {
		//    R[i][j]= ((MM_U_8 *)I2->p[j].m)[i] * ((MM_U_8 *)I2->p[j].m)[i];
		//    S[i] += R[i][j];
		//  } 
		//  M[k] += S[i];
		//}

		// reg 6 = M
		pxor_r2r (mm6,mm6);
		for(i=k;i<k+win;i+=2) {
			// reg 5 = S
			pxor_r2r (mm5,mm5);
			for(j=0;j<win;j++) {
				// calc R
				I1_I2MULTISUM(I2, I2, i,j,i,j+E);
				// Store new R
				movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[i]);
				// Add R to S
				paddd_r2r (mm0,mm5);
			}
			// Store new S
			movq_r2m (mm5,((MM_U_32 *)S)[i]);
			// Add curr S to M
			paddd_r2r (mm5,mm6);
		}
		// Store M[0]
		movq_r2m (mm6,((MM_U_32 *)M)[k]);    

#ifdef FAST_SQRT
		sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
		sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
		((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
		((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
#else
		((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k]))/((MM_U_32 *)M)[k]);
		((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k+1]))/((MM_U_32 *)M)[k+1]);
#endif
	}

	// compute N for x=0 & y=0
	// N->p[0][0]  -> N->p[d_max-1][0]
	for (d=0;d<d_max;d++) {
		//fprintf(stderr,"depthmap: d:%d \n",d);
		//N[0][d]=0;
		//// window across image
		//for(i=0;i<=win;i++) {
		//  Q[i][d] = 0;
		//  // window down image
		// for (j=0;j<win;j++) {
		//   P[i][j][d]= ((MM_U_8 *)I1->p[j].m)[i+offset] * ((MM_U_8 *)I2->p[j].m)[i+d];
		//   Q[i][d] += P[i][j][d];       
		// } 
		// if (i<win)
		//   N[0][d] += Q[i][d];
		//}

		// reg 6 = N
		pxor_r2r (mm6,mm6);
		for(i=0;i<win;i+=2) {
			// reg 5 = Q
			pxor_r2r (mm5,mm5);
			for (j=0;j<win;j++) {
				// calc new P   
				I1_I2MULTISUM(I1, I2, i+offset,j,i+d,j+E);
				// Store P
				movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[i]);
				// Add P to Q
				paddd_r2r (mm0,mm5);
			}
			// Store Q
			movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
			// Add curr Q to N
			paddd_r2r (mm5,mm6);
		}
		// Store N[d][0]
		movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);

		// Do for i=win
		// reg 5 = Q
		pxor_r2r (mm5,mm5);
		for (j=0;j<win;j++) {
			// calc new P  
			I1_I2MULTISUM(I1, I2, win+offset,j,win+d,j+E);
			// Store P
			movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[win]);
			// Add P to Q
			paddd_r2r (mm0,mm5);
		}
		// Store Q
		movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[win]);
	} // for d

	//--- Main y_max loop ---
	for(y=-1;y<y_max-win-E-1;y++){
		//fprintf(stderr,"depthmap: y:%d \n",y);
		y_win = y+win;
		yMODwin = y%win;


		// compute M for x=0    
		if (y>=0) {
			for (k=0;k<=d_max;k+=2){
				//fprintf(stderr,"depthmap: k:%d \n",k);
				// M[k]=0;
				// window across image
				//for(i=k;i<k+win;i++) {
				//  S[i] -= R[i][y%win];
				//  R[i][y%win] = ((MM_U_8 *)I2->p[y+win].m)[i] * ((MM_U_8 *)I2->p[y+win].m)[i];
				//  S[i] += R[i][y%win];  
				//  M[k] += S[i];
				//}
				// reg 6&6 = M
				pxor_r2r (mm6,mm6);
				for(i=k;i<k+win;i+=2) {
					// reg 5 & 4 = S
					movq_m2r (((MM_U_32 *)S)[i],mm5);  
					// Sub prev R from S
					psubd_m2r (((MM_U_32 *)R->p[yMODwin].m)[i],mm5);
					// calc new R
					I1_I2MULTISUM(I2, I2, i,y_win,i,y_win+E);
					// Store R
					movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[i]);
					// Add R to S
					paddd_r2r (mm0,mm5);
					// Store S
					movq_r2m (mm5,((MM_U_32 *)S)[i]);
					// Add curr S to M
					paddd_r2r (mm5,mm6);
				} // for i
				// Store M[0]
				movq_r2m (mm6,((MM_U_32 *)M)[k]);
#ifdef FAST_SQRT
				sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
				sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
				((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
				((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
#else
				((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k]))/((MM_U_32 *)M)[k]);
				((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k+1]))/((MM_U_32 *)M)[k+1]);
#endif
			} // for k
		} // if y >= 0

		//--- Main X loop ---
		for(x=-2;x<x_max-win-d_max-2;x+=2) {
			//fprintf(stderr,"depthmap: x:%d \n",x);
			x_win= x+win;

			// compute M for y=0 
			if (y<0) {
				// Equivalent C function
				//S[x+d_max+win] = 0;
				//for (j=0;j<win;j++) {
				//  R[x+d_max+win][j]= ((MM_U_8 *)I2->p[j].m)[x+d_max+win] * ((MM_U_8 *)I2->p[j].m)[x+d_max+win];
				//  S[x+d_max+win] += R[x+d_max+win][j];
				//}
				// reg 5 = S
				pxor_r2r (mm5,mm5);
				// window down image
				for (j=0;j<win;j++) {
					//fprintf(stderr,"depthmap: j:%d\n",j);
					// Calc P
					I1_I2MULTISUM(I2, I2,x_win+d_max,j,x_win+d_max,j+E);
					// Store R
					movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[x_win+d_max]);
					// Add curr R to S
					paddd_r2r (mm0,mm5);
				}
				// Store S
				movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max]);

			}  
			else
			{
				// Equivalent C function
				// P is [x_max][win] the win dimension is a wrap around buffer.    
				//S[x+d_max+win] -= R[x+d_max+win][y%win];
				//R[x+d_max+win][y%win] = ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win] * ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win];
				//S[x+d_max+win] += R[x+d_max+win][y%win];

				// reg 5 = S
				movq_m2r (((MM_U_32 *)S)[x_win+d_max],mm5);
				// subtract prev R
				psubd_m2r (((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max],mm5);
				// Add curr R to S
				I1_I2MULTISUM(I2, I2,x_win+d_max,y_win,x_win+d_max,y_win+E);
				paddd_r2r (mm0,mm5);
				// Store S
				movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max]);
				// Store R
				movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max]);
			}

			// Equivalent C function
			//M[x+d_max+1] = M[x+d_max] - S[x+d_max] + S[x+d_max+win];
			// reg 6 & 6 = M
			movq_m2r (((MM_U_32 *)M)[x+d_max],mm6);  
			// Sub S[x+d_max] from M
			psubd_m2r (((MM_U_32 *)S)[x+d_max],mm6);
			// Add S[x+d_max+win] to M
			movq_m2r (((MM_U_32 *)S)[x_win+d_max],mm1);
			paddd_r2r (mm1,mm6);
			// Store M[x+1][d]
			movq_r2m (mm6,((MM_U_32 *)M)[x+d_max+2]);
#ifdef FAST_SQRT
			sqrt2 = fast_sqrt(((MM_U_32 *)M)[x+d_max+2]);
			sqrt3 = fast_sqrt(((MM_U_32 *)M)[x+d_max+3]);
			((MM_U_32 *)MINV)[x+d_max+2] = ((((unsigned long long)0xffffffff) + sqrt2 )/ sqrt2);
			((MM_U_32 *)MINV)[x+d_max+3] = ((((unsigned long long)0xffffffff) + sqrt3 )/ sqrt3);
#else
			((MM_U_32 *)MINV)[x+d_max+2] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+d_max+2])/((MM_U_32 *)M)[x+d_max+2]);
			((MM_U_32 *)MINV)[x+d_max+3] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+d_max+3])/((MM_U_32 *)M)[x+d_max+3]);
#endif
			//printf("%d %d %ld %ld\n",y,x,((MM_U_32 *)M)[x+2],((MM_U_32 *)M)[x+3]);

			max_d.ud[0] = 0;
			max_d.ud[1] = 0;    

			max.ud[0] = 0;
			max.ud[1] = 0;    


			//--- Main D loop ---
			for (d=0;d<d_max;d++) {

				//fprintf(stderr,"depthmap: d:%d \n",d);
				if (x<0) {
					if (y>=0) {
						// Equivalent C function
						//N[0][d]=0;
						//for(i=0;i<win;i++) {
						//  Q[i][d] -= P[i][y%win][d];
						//  P[i][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[i+offset] * ((MM_U_8 *)I2->p[y+win].m)[i+d];
						//  Q[i][d] += P[i][y%win][d];
						//  N[0][d] += Q[i][d];
						//}
						//Q[win][d] -= P[win][y%win][d];
						//P[win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[win+offset] * ((MM_U_8 *)I2->p[y+win].m)[win+d];
						//Q[win][d] += P[win][y%win][d];

						// reg 6&6 = N
						pxor_r2r (mm6,mm6);
						for(i=0;i<win;i+=2) {
							// reg 5 & 4 = Q
							movq_m2r (((MM_U_32 *)Q->p[d].m)[i],mm5);  
							// Sub prev P from Q
							psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[i],mm5);
							// Add curr P to Q
							I1_I2MULTISUM(I1, I2, i+offset,y_win,i+d,y_win+E);
							// Add new P
							paddd_r2r (mm0,mm5);
							// Store Q
							movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
							// Store P
							movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
							// Add curr Q to N
							paddd_r2r (mm5,mm6);
						} // for i
						// Store N[d][0]
						movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);

						// Do for i=win
						// reg 5 & 4 = Q
						movq_m2r (((MM_U_32 *)Q->p[d].m)[i],mm5);  
						// Sub prev P from Q
						psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[i],mm5);
						// Add curr P to Q
						// lbw in 0
						I1_I2MULTISUM(I1, I2, i+offset,y_win,i+d,y_win+E);
						// Add new P
						paddd_r2r (mm0,mm5);
						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
						// Store P
						movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
					} // y>=0
				} // x <0
				else
				{
					if (y<0) {
						// Equivalent C function
						//Q[x+win][d] = 0;
						//for (j=0;j<win;j++) {
						//  P[x+win][j][d]= ((MM_U_8 *)I1->p[j].m)[x+win+offset] * ((MM_U_8 *)I2->p[j].m)[x+win+d];
						//  Q[x+win][d] += P[x+win][j][d];
						//}

						// reg 5&4 = Q
						pxor_r2r (mm5,mm5);

						// window down image
						for (j=0;j<win;j++) {
							I1_I2MULTISUM(I1, I2,x_win+offset,j,x_win+d,j+E);
							movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[x_win]);
							// Add curr P to Q
							paddd_r2r (mm0,mm5);
						}
						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);

					} // y<0 
					else
					{

						// Equivalent C function
						// P is [x_max][win][d_max] the win dimension is a wrap around buffer.
						//Q[x+win][d] -= P[x+win][y%win][d];
						//P[x+win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[x+win+offset] * ((MM_U_8 *)I2->p[y+win].m)[x+win+d];
						//Q[x+win][d] += P[x+win][y%win][d];

						// reg 5&4 = Q
						movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm5);
						// subtract prev P
						psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[x+win],mm5);
						// Add curr P to Q
						I1_I2MULTISUM(I1, I2,x_win+offset,y_win,x_win+d,y_win+E);
						paddd_r2r (mm0,mm5);

						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);

						// Store P
						movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[x_win]);

					} // y>=0

					// Equivalent C function
					//N[x+1][d] = N[x][d] - Q[x][d] + Q[x+win][d];

					// reg 6 & 6 = N
					movq_m2r (((MM_U_32 *)N->p[d].m)[x],mm6);  

					// Sub curr Q[x][d] from N
					psubd_m2r (((MM_U_32 *)Q->p[d].m)[x],mm6);

					// Add Q[x+win][d] to N
					movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm1);
					paddd_r2r (mm1,mm6);

					// Store N[x+1][d]
					movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[x+2]);

				} // x>=0
				// Equivalent C function              
				//C1 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
				//if (C1 > max1) {
				//   max1 = C1;
				//   max1_d = d;
				//}
				//C2 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);
				//if (C2 > max2) {
				//  max2 = C2;
				//  max2_d = d;
				//}

#ifdef FAST_SQRT
				curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
				curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#else
				curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
				curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#endif


				//curr.ud[0] = ((MM_U_32 *)N->p[d].m)[x+2]/fast_sqrt(((MM_U_32 *)M)[x+d+2]);
				//curr.ud[1] = ((MM_U_32 *)N->p[d].m)[x+3]/fast_sqrt(((MM_U_32 *)M)[x+d+3]);


				//curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
				//curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);

				// reg 5 = curr 
				movq_m2r(curr,mm5);
				movq_r2r(mm5,mm0);
				//PRINTREG32(mm5,"curr");

				// reg 6 = max
				movq_m2r(max,mm6);
				movq_r2r(mm6,mm2);
				//PRINTREG32(mm6,"max");

				// reg 2 becomes mask
				pcmpgtd_r2r(mm0,mm2);
				movq_r2r(mm2,mm1);
				//PRINTREG32(mm2,"mask");

				// reg 0 is preserved values larger than max
				movq_r2r(mm6,mm0);
				pand_r2r(mm2,mm0);

				// reg 1 is curr
				pandn_r2r(mm5,mm1);

				// reg 1 is new max
				por_r2r(mm0,mm1);
				movq_r2m(mm1,max);
				//PRINTREG32(mm1,"new_max");

				// reg 1 is preserved d values
				movq_m2r(max_d,mm3);
				movq_r2r(mm2,mm1);
				pand_r2r(mm3,mm1);

				// reg 0 = current d value
				curr_d.ud[0] = d;
				curr_d.ud[1] = d;    

				movq_m2r(curr_d,mm4);
				pandn_r2r(mm4,mm2);

				// reg 6 = new d's for minimum values
				por_r2r(mm2,mm1);
				movq_r2m(mm1,max_d);
				//PRINTREG32(mm1,"new_max_d");


			}// for d          
			//printf("%u\n",max.ud[0]);
			//printf("%u\n",max.ud[1]);
			// if (max.ud[0]<MAX_THRESH)
			((MM_U_8 *)Out->p[y+1].m)[x+2] = (max_d.ub[0]<<2);
			//else
			//((MM_U_8 *)Out->p[y+1].m)[x+2] = 0;
			//if (max.ud[1]<MAX_THRESH)
			((MM_U_8 *)Out->p[y+1].m)[x+3] = (max_d.ub[4]<<2);
			//else
			//((MM_U_8 *)Out->p[y+1].m)[x+3] = 0;


		}// for x
	}// for y

	emms();
	return 0;
}

#ifndef USE_NCC_LEANNE
int MMXRecursive_Depthmap_NCC4_leanne(MMXMatrix *I1, MMXMatrix *I2, 
		MMXMatrix  *Out, int d_max, int offset, const int win, 
		MMXMatricesCachePtr * vcache)
{
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
	int y_max;
	int x_max;

	const int E = 0;
	int w_2 = win/2;

	//short d_2 = 0;


	register int i, j, k;
	register int d,x,y;

	register int y_win, x_win, yMODwin;

#ifdef FAST_SQRT
	unsigned long sqrt2,sqrt3;
#endif

	mmx_t curr, curr_d, max, max_d;

	MMXMatrix **P, *Q, *N;
	MMXMatrix *R;
	MM_U_32 *S, *M, *MINV;

	x_max = I1->cols;
	y_max = I1->rows;

	if ((*cache != 0) && ((*cache)->nmatrices != win+3)) {
		MMXMatricesCacheFree(*cache);
		*cache = 0;
	}

	if (*cache==0) {
		*cache = MMXMatricesCacheAlloc(win+3,3,0);
		// Numerator matrix
		for (i=0;i<win;i++)
			(*cache)->M[i] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		(*cache)->M[win+0] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		(*cache)->M[win+1] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		(*cache)->M[win+2] = MMXMatrixAlloc(MMT_U_32, x_max, win);

		// Denominator matrix
		(*cache)->V[0] = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
		//M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
		(*cache)->V[1] = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
		(*cache)->V[2] = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
	} 
	P = (*cache)->M;
	Q = (*cache)->M[win+0];
	N = (*cache)->M[win+1];
	R = (*cache)->M[win+2];

	// Denominator matrix
	S = (*cache)->V[0];
	M = (*cache)->V[1];
	MINV = (*cache)->V[2];

	emms();
	// clear reg 7
	pxor_r2r (mm7,mm7);

	// compute M for x=0 y=0
	// M[0] -> M[d_max]
	// S[0] -> S[d_max+win-2]
	for (k=0;k<=d_max;k+=2){
		//fprintf(stderr,"depthmap: k:%d \n",k);
		//M[k]=0;
		//// window across image
		//for(i=k;i<k+win;i++) {
		//  S[i]=0;
		//  // window down image
		//  for (j=0;j<win;j++) {
		//    R[i][j]= ((MM_U_8 *)I2->p[j].m)[i] * ((MM_U_8 *)I2->p[j].m)[i];
		//    S[i] += R[i][j];
		//  } 
		//  M[k] += S[i];
		//}

		// reg 6 = M
		pxor_r2r (mm6,mm6);
		for(i=k;i<k+win;i+=2) {
			// reg 5 = S
			pxor_r2r (mm5,mm5);
			for(j=0;j<win;j++) {
				// calc R
				I1_I2MULTISUM(I2, I2, i,j,i,j+E);
				// Store new R
				movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[i]);
				// Add R to S
				paddd_r2r (mm0,mm5);
			}
			// Store new S
			movq_r2m (mm5,((MM_U_32 *)S)[i]);
			// Add curr S to M
			paddd_r2r (mm5,mm6);
		}
		// Store M[0]
		movq_r2m (mm6,((MM_U_32 *)M)[k]);    

#ifdef FAST_SQRT
		sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
		sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
		((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
		((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
#else
		((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k]))/((MM_U_32 *)M)[k]);
		((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k+1]))/((MM_U_32 *)M)[k+1]);
#endif
	}

	// compute N for x=0 & y=0
	// N->p[0][0]  -> N->p[d_max-1][0]
	for (d=0;d<d_max;d++) {
		//fprintf(stderr,"depthmap: d:%d \n",d);
		//N[0][d]=0;
		//// window across image
		//for(i=0;i<=win;i++) {
		//  Q[i][d] = 0;
		//  // window down image
		// for (j=0;j<win;j++) {
		//   P[i][j][d]= ((MM_U_8 *)I1->p[j].m)[i+offset] * ((MM_U_8 *)I2->p[j].m)[i+d];
		//   Q[i][d] += P[i][j][d];       
		// } 
		// if (i<win)
		//   N[0][d] += Q[i][d];
		//}

		// reg 6 = N
		pxor_r2r (mm6,mm6);
		for(i=0;i<win;i+=2) {
			// reg 5 = Q
			pxor_r2r (mm5,mm5);
			for (j=0;j<win;j++) {
				// calc new P   
				I1_I2MULTISUM(I1, I2, i+offset,j,i+d,j+E);
				// Store P
				movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[i]);
				// Add P to Q
				paddd_r2r (mm0,mm5);
			}
			// Store Q
			movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
			// Add curr Q to N
			paddd_r2r (mm5,mm6);
		}
		// Store N[d][0]
		movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);

		// Do for i=win
		// reg 5 = Q
		pxor_r2r (mm5,mm5);
		for (j=0;j<win;j++) {
			// calc new P  
			I1_I2MULTISUM(I1, I2, win+offset,j,win+d,j+E);
			// Store P
			movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[win]);
			// Add P to Q
			paddd_r2r (mm0,mm5);
		}
		// Store Q
		movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[win]);
	} // for d

	//--- Main y_max loop ---
	for(y=-1;y<y_max-win-E-1;y++){
		//fprintf(stderr,"depthmap: y:%d \n",y);
		y_win = y+win;
		yMODwin = y%win;


		// compute M for x=0    
		if (y>=0) {
			for (k=0;k<=d_max;k+=2){
				//fprintf(stderr,"depthmap: k:%d \n",k);
				// M[k]=0;
				// window across image
				//for(i=k;i<k+win;i++) {
				//  S[i] -= R[i][y%win];
				//  R[i][y%win] = ((MM_U_8 *)I2->p[y+win].m)[i] * ((MM_U_8 *)I2->p[y+win].m)[i];
				//  S[i] += R[i][y%win];  
				//  M[k] += S[i];
				//}
				// reg 6&6 = M
				pxor_r2r (mm6,mm6);
				for(i=k;i<k+win;i+=2) {
					// reg 5 & 4 = S
					movq_m2r (((MM_U_32 *)S)[i],mm5);  
					// Sub prev R from S
					psubd_m2r (((MM_U_32 *)R->p[yMODwin].m)[i],mm5);
					// calc new R
					I1_I2MULTISUM(I2, I2, i,y_win,i,y_win+E);
					// Store R
					movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[i]);
					// Add R to S
					paddd_r2r (mm0,mm5);
					// Store S
					movq_r2m (mm5,((MM_U_32 *)S)[i]);
					// Add curr S to M
					paddd_r2r (mm5,mm6);
				} // for i
				// Store M[0]
				movq_r2m (mm6,((MM_U_32 *)M)[k]);
#ifdef FAST_SQRT
				sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
				sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
				((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
				((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
#else
				((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k]))/((MM_U_32 *)M)[k]);
				((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k+1]))/((MM_U_32 *)M)[k+1]);
#endif
			} // for k
		} // if y >= 0

		//--- Main X loop ---
		for(x=-2;x<x_max-win-d_max-2;x+=2) {
			//fprintf(stderr,"depthmap: x:%d \n",x);
			x_win= x+win;

			// compute M for y=0 
			if (y<0) {
				// Equivalent C function
				//S[x+d_max+win] = 0;
				//for (j=0;j<win;j++) {
				//  R[x+d_max+win][j]= ((MM_U_8 *)I2->p[j].m)[x+d_max+win] * ((MM_U_8 *)I2->p[j].m)[x+d_max+win];
				//  S[x+d_max+win] += R[x+d_max+win][j];
				//}
				// reg 5 = S
				pxor_r2r (mm5,mm5);
				// window down image
				for (j=0;j<win;j++) {
					//fprintf(stderr,"depthmap: j:%d\n",j);
					// Calc P
					I1_I2MULTISUM(I2, I2,x_win+d_max,j,x_win+d_max,j+E);
					// Store R
					movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[x_win+d_max]);
					// Add curr R to S
					paddd_r2r (mm0,mm5);
				}
				// Store S
				movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max]);

			}  
			else
			{
				// Equivalent C function
				// P is [x_max][win] the win dimension is a wrap around buffer.    
				//S[x+d_max+win] -= R[x+d_max+win][y%win];
				//R[x+d_max+win][y%win] = ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win] * ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win];
				//S[x+d_max+win] += R[x+d_max+win][y%win];

				// reg 5 = S
				movq_m2r (((MM_U_32 *)S)[x_win+d_max],mm5);
				// subtract prev R
				psubd_m2r (((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max],mm5);
				// Add curr R to S
				I1_I2MULTISUM(I2, I2,x_win+d_max,y_win,x_win+d_max,y_win+E);
				paddd_r2r (mm0,mm5);
				// Store S
				movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max]);
				// Store R
				movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max]);
			}

			// Equivalent C function
			//M[x+d_max+1] = M[x+d_max] - S[x+d_max] + S[x+d_max+win];
			// reg 6 & 6 = M
			movq_m2r (((MM_U_32 *)M)[x+d_max],mm6);  
			// Sub S[x+d_max] from M
			psubd_m2r (((MM_U_32 *)S)[x+d_max],mm6);
			// Add S[x+d_max+win] to M
			movq_m2r (((MM_U_32 *)S)[x_win+d_max],mm1);
			paddd_r2r (mm1,mm6);
			// Store M[x+1][d]
			movq_r2m (mm6,((MM_U_32 *)M)[x+d_max+2]);
#ifdef FAST_SQRT
			sqrt2 = fast_sqrt(((MM_U_32 *)M)[x+d_max+2]);
			sqrt3 = fast_sqrt(((MM_U_32 *)M)[x+d_max+3]);
			((MM_U_32 *)MINV)[x+d_max+2] = ((((unsigned long long)0xffffffff) + sqrt2 )/ sqrt2);
			((MM_U_32 *)MINV)[x+d_max+3] = ((((unsigned long long)0xffffffff) + sqrt3 )/ sqrt3);
#else
			((MM_U_32 *)MINV)[x+d_max+2] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+d_max+2])/((MM_U_32 *)M)[x+d_max+2]);
			((MM_U_32 *)MINV)[x+d_max+3] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+d_max+3])/((MM_U_32 *)M)[x+d_max+3]);
#endif
			//printf("%d %d %ld %ld\n",y,x,((MM_U_32 *)M)[x+2],((MM_U_32 *)M)[x+3]);

			max_d.ud[0] = 0;
			max_d.ud[1] = 0;    

			max.ud[0] = 0;
			max.ud[1] = 0;    


			//--- Main D loop ---
			for (d=0;d<d_max;d++) {

				//fprintf(stderr,"depthmap: d:%d \n",d);
				if (x<0) {
					if (y>=0) {
						// Equivalent C function
						//N[0][d]=0;
						//for(i=0;i<win;i++) {
						//  Q[i][d] -= P[i][y%win][d];
						//  P[i][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[i+offset] * ((MM_U_8 *)I2->p[y+win].m)[i+d];
						//  Q[i][d] += P[i][y%win][d];
						//  N[0][d] += Q[i][d];
						//}
						//Q[win][d] -= P[win][y%win][d];
						//P[win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[win+offset] * ((MM_U_8 *)I2->p[y+win].m)[win+d];
						//Q[win][d] += P[win][y%win][d];

						// reg 6&6 = N
						pxor_r2r (mm6,mm6);
						for(i=0;i<win;i+=2) {
							// reg 5 & 4 = Q
							movq_m2r (((MM_U_32 *)Q->p[d].m)[i],mm5);  
							// Sub prev P from Q
							psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[i],mm5);
							// Add curr P to Q
							I1_I2MULTISUM(I1, I2, i+offset,y_win,i+d,y_win+E);
							// Add new P
							paddd_r2r (mm0,mm5);
							// Store Q
							movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
							// Store P
							movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
							// Add curr Q to N
							paddd_r2r (mm5,mm6);
						} // for i
						// Store N[d][0]
						movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);

						// Do for i=win
						// reg 5 & 4 = Q
						movq_m2r (((MM_U_32 *)Q->p[d].m)[i],mm5);  
						// Sub prev P from Q
						psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[i],mm5);
						// Add curr P to Q
						// lbw in 0
						I1_I2MULTISUM(I1, I2, i+offset,y_win,i+d,y_win+E);
						// Add new P
						paddd_r2r (mm0,mm5);
						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
						// Store P
						movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
					} // y>=0
				} // x <0
				else
				{
					if (y<0) {
						// Equivalent C function
						//Q[x+win][d] = 0;
						//for (j=0;j<win;j++) {
						//  P[x+win][j][d]= ((MM_U_8 *)I1->p[j].m)[x+win+offset] * ((MM_U_8 *)I2->p[j].m)[x+win+d];
						//  Q[x+win][d] += P[x+win][j][d];
						//}

						// reg 5&4 = Q
						pxor_r2r (mm5,mm5);

						// window down image
						for (j=0;j<win;j++) {
							I1_I2MULTISUM(I1, I2,x_win+offset,j,x_win+d,j+E);
							movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[x_win]);
							// Add curr P to Q
							paddd_r2r (mm0,mm5);
						}
						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);

					} // y<0 
					else
					{

						// Equivalent C function
						// P is [x_max][win][d_max] the win dimension is a wrap around buffer.
						//Q[x+win][d] -= P[x+win][y%win][d];
						//P[x+win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[x+win+offset] * ((MM_U_8 *)I2->p[y+win].m)[x+win+d];
						//Q[x+win][d] += P[x+win][y%win][d];

						// reg 5&4 = Q
						movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm5);
						// subtract prev P
						psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[x+win],mm5);
						// Add curr P to Q
						I1_I2MULTISUM(I1, I2,x_win+offset,y_win,x_win+d,y_win+E);
						paddd_r2r (mm0,mm5);

						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);

						// Store P
						movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[x_win]);

					} // y>=0

					// Equivalent C function
					//N[x+1][d] = N[x][d] - Q[x][d] + Q[x+win][d];

					// reg 6 & 6 = N
					movq_m2r (((MM_U_32 *)N->p[d].m)[x],mm6);  

					// Sub curr Q[x][d] from N
					psubd_m2r (((MM_U_32 *)Q->p[d].m)[x],mm6);

					// Add Q[x+win][d] to N
					movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm1);
					paddd_r2r (mm1,mm6);

					// Store N[x+1][d]
					movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[x+2]);

				} // x>=0
				// Equivalent C function              
				//C1 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
				//if (C1 > max1) {
				//   max1 = C1;
				//   max1_d = d;
				//}
				//C2 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);
				//if (C2 > max2) {
				//  max2 = C2;
				//  max2_d = d;
				//}

#ifdef FAST_SQRT
				curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
				curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#else
				curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
				curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#endif


				//curr.ud[0] = ((MM_U_32 *)N->p[d].m)[x+2]/fast_sqrt(((MM_U_32 *)M)[x+d+2]);
				//curr.ud[1] = ((MM_U_32 *)N->p[d].m)[x+3]/fast_sqrt(((MM_U_32 *)M)[x+d+3]);


				//curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
				//curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);

				// reg 5 = curr 
				movq_m2r(curr,mm5);
				movq_r2r(mm5,mm0);
				//PRINTREG32(mm5,"curr");

				// reg 6 = max
				movq_m2r(max,mm6);
				movq_r2r(mm6,mm2);
				//PRINTREG32(mm6,"max");

				// reg 2 becomes mask
				pcmpgtd_r2r(mm0,mm2);
				movq_r2r(mm2,mm1);
				//PRINTREG32(mm2,"mask");

				// reg 0 is preserved values larger than max
				movq_r2r(mm6,mm0);
				pand_r2r(mm2,mm0);

				// reg 1 is curr
				pandn_r2r(mm5,mm1);

				// reg 1 is new max
				por_r2r(mm0,mm1);
				movq_r2m(mm1,max);
				//PRINTREG32(mm1,"new_max");

				// reg 1 is preserved d values
				movq_m2r(max_d,mm3);
				movq_r2r(mm2,mm1);
				pand_r2r(mm3,mm1);

				// reg 0 = current d value
				curr_d.ud[0] = d;
				curr_d.ud[1] = d;    

				movq_m2r(curr_d,mm4);
				pandn_r2r(mm4,mm2);

				// reg 6 = new d's for minimum values
				por_r2r(mm2,mm1);
				movq_r2m(mm1,max_d);
				//PRINTREG32(mm1,"new_max_d");


			}// for d          
			//printf("%u\n",max.ud[0]);
			//printf("%u\n",max.ud[1]);
			// if (max.ud[0]<MAX_THRESH)
			((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+2] = (max_d.ub[0]<<2);
			//else
			//((MM_U_8 *)Out->p[y+1].m)[x+2] = 0;
			//if (max.ud[1]<MAX_THRESH)
			((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+3] = (max_d.ub[4]<<2);
			//else
			//((MM_U_8 *)Out->p[y+1].m)[x+3] = 0;


		}// for x
	}// for y

	emms();
	return 0;
}
#else
int MMXRecursive_Depthmap_NCC4_leanne(MMXMatrix *I1, MMXMatrix *I2, 
		MMXMatrix  *Out, int d_max, int offset, const int win, 
		MMXMatricesCachePtr * vcache)
{
	//printf("Depth Map : d_max %d, offset %d, win %d\n",
	//		d_max,offset,win);
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
	//static int allocated = 0;
	int y_max;
	int x_max;

	//const int win = 16;
	//const int d_max = 64;
	const int E = 0;
	const int shift = (d_max>32)?2:3;


	int d_max_edge = 0;
	int x_total = 0;

	int overall_min_d_lr=0;
	int overall_min_lr=0;
	int subpix_d,subpix_denom;

	register int i, j, k;
	register int d,x,y;

	register int y_win, x_win, yMODwin;

#ifdef FAST_SQRT
	unsigned long sqrt2,sqrt3;
#endif

	mmx_t curr, curr_d, max, max_d;

	static MMXMatrix **P, *Q, *N; 
	static MMXMatrix *R;
	static MM_U_32 *S, *M, *MINV;
	MMXMatrix *C1, *C2, *C, *C_prev;

	x_max = I1->cols;
	y_max = I1->rows;

	if ((*cache != 0) && ((*cache)->nmatrices != win+5)) {
		MMXMatricesCacheFree(*cache);
		*cache = 0;
	}

	if (*cache==0) {
		*cache = MMXMatricesCacheAlloc(win+5,3,0);
		// Numerator matrix
		for (i=0;i<win;i++)
			(*cache)->M[i] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		(*cache)->M[win+0] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		(*cache)->M[win+1] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		(*cache)->M[win+2] = MMXMatrixAlloc(MMT_U_32, x_max, win);

		// Denominator matrix
		(*cache)->V[0] = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
		//M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
		(*cache)->V[1] = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
		(*cache)->V[2] = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 

		(*cache)->M[win+3] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);
		(*cache)->M[win+4] = MMXMatrixAlloc(MMT_U_32, x_max, d_max);

	} 
	P = (*cache)->M;
	Q = (*cache)->M[win+0];
	N = (*cache)->M[win+1];
	R = (*cache)->M[win+2];

	// Denominator matrix
	S = (*cache)->V[0];
	M = (*cache)->V[1];
	MINV = (*cache)->V[2];

	C1 = (*cache)->M[win+3]; 
	C2 = (*cache)->M[win+4]; 

	emms();
	// clear reg 7
	pxor_r2r (mm7,mm7);

	// compute M for x=0 y=0
	// M[0] -> M[d_max]
	// S[0] -> S[d_max+win-2]
	for (k=0;k<=d_max;k+=2){
		//fprintf(stderr,"depthmap: k:%d \n",k);
		//M[k]=0;
		//// window across image
		//for(i=k;i<k+win;i++) {
		//  S[i]=0;
		//  // window down image
		//  for (j=0;j<win;j++) {
		//    R[i][j]= ((MM_U_8 *)I2->p[j].m)[i] * ((MM_U_8 *)I2->p[j].m)[i];
		//    S[i] += R[i][j];
		//  } 
		//  M[k] += S[i];
		//}

		// reg 6 = M
		pxor_r2r (mm6,mm6);
		for(i=k;i<k+win;i+=2) {
			// reg 5 = S
			pxor_r2r (mm5,mm5);
			for(j=0;j<win;j++) {
				// calc R
				I1_I2MULTISUM(I2, I2, i,j,i,j+E);
				// Store new R
				movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[i]);
				// Add R to S
				paddd_r2r (mm0,mm5);
			}
			// Store new S
			movq_r2m (mm5,((MM_U_32 *)S)[i]);
			// Add curr S to M
			paddd_r2r (mm5,mm6);
		}
		// Store M[0]
		movq_r2m (mm6,((MM_U_32 *)M)[k]);    

#ifdef FAST_SQRT
		sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
		sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
		((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
		((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
#else
		((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k]))/((MM_U_32 *)M)[k]);
		((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k+1]))/((MM_U_32 *)M)[k+1]);
#endif
	}

	// compute N for x=0 & y=0
	// N->p[0][0]  -> N->p[d_max-1][0]
	for (d=0;d<d_max;d++) {
		//N[0][d]=0;
		//// window across image
		//for(i=0;i<=win;i++) {
		//  Q[i][d] = 0;
		//  // window down image
		// for (j=0;j<win;j++) {
		//   P[i][j][d]= ((MM_U_8 *)I1->p[j].m)[i+offset] * ((MM_U_8 *)I2->p[j].m)[i+d];
		//   Q[i][d] += P[i][j][d];       
		// } 
		// if (i<win)
		//   N[0][d] += Q[i][d];
		//}

		// reg 6 = N
		pxor_r2r (mm6,mm6);
		for(i=0;i<win;i+=2) {
			// reg 5 = Q
			pxor_r2r (mm5,mm5);
			for (j=0;j<win;j++) {
				// calc new P   
				I1_I2MULTISUM(I1, I2, i+offset,j,i+d,j+E);
				// Store P
				movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[i]);
				// Add P to Q
				paddd_r2r (mm0,mm5);
			}
			// Store Q
			movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
			// Add curr Q to N
			paddd_r2r (mm5,mm6);
		}
		// Store N[d][0]
		movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);

		// Do for i=win
		// reg 5 = Q
		pxor_r2r (mm5,mm5);
		for (j=0;j<win;j++) {
			// calc new P  
			I1_I2MULTISUM(I1, I2, win+offset,j,win+d,j+E);
			// Store P
			movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[win]);
			// Add P to Q
			paddd_r2r (mm0,mm5);
		}
		// Store Q
		movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[win]);
	} // for d



	C = C2;
	C_prev = C1;

	/****************************
	  --- Main y_max loop ---
	 *****************************/
	for(y=-1;y<y_max-win-E-1;y++){
		//fprintf(stderr,"depthmap: y:%d \n",y);
		y_win = y+win;
		yMODwin = y%win;

		if (C==C1) {
			C = C2;
			C_prev = C1;
		}
		else {
			C = C1;
			C_prev = C2;
		}

		// compute M for x=0    
		if (y>=0) {
			for (k=0;k<=d_max;k+=2){
				//fprintf(stderr,"depthmap: k:%d \n",k);
				// M[k]=0;
				// window across image
				//for(i=k;i<k+win;i++) {
				//  S[i] -= R[i][y%win];
				//  R[i][y%win] = ((MM_U_8 *)I2->p[y+win].m)[i] * ((MM_U_8 *)I2->p[y+win].m)[i];
				//  S[i] += R[i][y%win];  
				//  M[k] += S[i];
				//}
				// reg 6&6 = M
				pxor_r2r (mm6,mm6);
				for(i=k;i<k+win;i+=2) {
					// reg 5 & 4 = S
					movq_m2r (((MM_U_32 *)S)[i],mm5);  
					// Sub prev R from S
					psubd_m2r (((MM_U_32 *)R->p[yMODwin].m)[i],mm5);
					// calc new R
					I1_I2MULTISUM(I2, I2, i,y_win,i,y_win+E);
					// Store R
					movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[i]);
					// Add R to S
					paddd_r2r (mm0,mm5);
					// Store S
					movq_r2m (mm5,((MM_U_32 *)S)[i]);
					// Add curr S to M
					paddd_r2r (mm5,mm6);
				} // for i
				// Store M[0]
				movq_r2m (mm6,((MM_U_32 *)M)[k]);
#ifdef FAST_SQRT
				sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
				sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
				((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
				((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
#else
				((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k]))/((MM_U_32 *)M)[k]);
				((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + ((unsigned long long)((MM_U_32 *)M)[k+1]))/((MM_U_32 *)M)[k+1]);
#endif
			} // for k
		} // if y >= 0

		/**********************************************/
		//--- Main X loop ---
		/**********************************************/

		for(x=-2;x<x_max-win-d_max-2;x+=2) {


			//fprintf(stderr,"depthmap: x:%d \n",x);
			x_win= x+win;
			//printf("x WIN, y: %d, %d\n", x_win, y);

			/* Check to make sure that the search has not gone off the
			   image. If it has, decrease d_max. */
			x_total = x_win+d_max;
			if ( x_total > x_max) {
				d_max_edge = d_max - (x_total-x_max);
			} else {
				d_max_edge = d_max;
			}

			// compute M for y=0 
			if (y<0) {
				// Equivalent C function
				//S[x+d_max+win] = 0;
				//for (j=0;j<win;j++) {
				//  R[x+d_max+win][j]= ((MM_U_8 *)I2->p[j].m)[x+d_max+win] * ((MM_U_8 *)I2->p[j].m)[x+d_max+win];
				//  S[x+d_max+win] += R[x+d_max+win][j];
				//}
				// reg 5 = S
				pxor_r2r (mm5,mm5);
				// window down image
				for (j=0;j<win;j++) {
					// Calc R 
					I1_I2MULTISUM(I2, I2,x_win+d_max_edge,j,x_win+d_max_edge,j+E);
					// Store R
					movq_r2m (mm0,((MM_U_32 *)R->p[j].m)[x_win+d_max_edge]);
					// Add curr R to S
					paddd_r2r (mm0,mm5);
				}
				// Store S
				movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max_edge]);
			}  
			else
			{
				// Equivalent C function
				// P is [x_max][win] the win dimension is a wrap around buffer.    
				//S[x+d_max+win] -= R[x+d_max+win][y%win];
				//R[x+d_max+win][y%win] = ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win] * ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win];
				//S[x+d_max+win] += R[x+d_max+win][y%win];

				// reg 5 = S
				movq_m2r (((MM_U_32 *)S)[x_win+d_max_edge],mm5);
				// subtract prev R
				psubd_m2r (((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max_edge],mm5);
				// Add curr R to S
				I1_I2MULTISUM(I2, I2,x_win+d_max_edge,y_win,x_win+d_max_edge,y_win+E);
				paddd_r2r (mm0,mm5);
				// Store S
				movq_r2m (mm5,((MM_U_32 *)S)[x_win+d_max_edge]);
				// Store R
				movq_r2m (mm0,((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max_edge]);
			}

			// Equivalent C function
			//M[x+d_max+1] = M[x+d_max] - S[x+d_max] + S[x+d_max+win];
			// reg 6 & 6 = M
			movq_m2r (((MM_U_32 *)M)[x+d_max_edge],mm6);  
			// Sub S[x+d_max] from M
			psubd_m2r (((MM_U_32 *)S)[x+d_max_edge],mm6);
			// Add S[x+d_max+win] to M
			// TODO : Check that S[x_win+d_max_edge] has been affected before!
			movq_m2r (((MM_U_32 *)S)[x_win+d_max_edge],mm1);
			paddd_r2r (mm1,mm6);
			// Store M[x+1][d]
			movq_r2m (mm6,((MM_U_32 *)M)[x+d_max_edge+2]);
#ifdef FAST_SQRT
			// TODO : this is 0 on completely black images
			sqrt2 = fast_sqrt(((MM_U_32 *)M)[x+d_max_edge+2]);
			sqrt3 = fast_sqrt(((MM_U_32 *)M)[x+d_max_edge+3]);
			// TODO : so we cannot divide here!
			((MM_U_32 *)MINV)[x+d_max_edge+2] = ((((unsigned long long)0xffffffff) + sqrt2 )/ sqrt2);
			((MM_U_32 *)MINV)[x+d_max_edge+3] = ((((unsigned long long)0xffffffff) + sqrt3 )/ sqrt3);
#else
			((MM_U_32 *)MINV)[x+d_max_edge+2] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+d_max_edge+2])/((MM_U_32 *)M)[x+d_max_edge+2]);
			((MM_U_32 *)MINV)[x+d_max_edge+3] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+d_max_edge+3])/((MM_U_32 *)M)[x+d_max_edge+3]);
#endif

			max_d.ud[0] = 0;
			max_d.ud[1] = 0;    

			max.ud[0] = 0;
			max.ud[1] = 0;    

			//printf("before main d loop\n");
			/**********************************************/
			//--- Main D loop ---
			/**********************************************/
			for (d=0;d<d_max_edge;d++) {

				//fprintf(stderr,"depthmap: d:%d \n",d);
				if (x<0) {
					if (y>=0) {
						// Equivalent C function
						//N[0][d]=0;
						//for(i=0;i<win;i++) {
						//  Q[i][d] -= P[i][y%win][d];
						//  P[i][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[i+offset] * ((MM_U_8 *)I2->p[y+win].m)[i+d];
						//  Q[i][d] += P[i][y%win][d];
						//  N[0][d] += Q[i][d];
						//}
						//Q[win][d] -= P[win][y%win][d];
						//P[win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[win+offset] * ((MM_U_8 *)I2->p[y+win].m)[win+d];
						//Q[win][d] += P[win][y%win][d];

						// reg 6&6 = N
						pxor_r2r (mm6,mm6);
						for(i=0;i<win;i+=2) {
							// reg 5 & 4 = Q
							movq_m2r (((MM_U_32 *)Q->p[d].m)[i],mm5);  
							// Sub prev P from Q
							psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[i],mm5);
							// Add curr P to Q
							I1_I2MULTISUM(I1, I2, i+offset,y_win,i+d,y_win+E);
							// Add new P
							paddd_r2r (mm0,mm5);
							// Store Q
							movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
							// Store P
							movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
							// Add curr Q to N
							paddd_r2r (mm5,mm6);
						} // for i
						// Store N[d][0]
						movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[0]);

						// Do for i=win
						// reg 5 & 4 = Q
						movq_m2r (((MM_U_32 *)Q->p[d].m)[i],mm5);  
						// Sub prev P from Q
						psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[i],mm5);
						// Add curr P to Q
						// lbw in 0
						I1_I2MULTISUM(I1, I2, i+offset,y_win,i+d,y_win+E);
						// Add new P
						paddd_r2r (mm0,mm5);
						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[i]);
						// Store P
						movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
					} // y>=0
				} // x <0
				else
				{
					if (y<0) {
						// Equivalent C function
						//Q[x+win][d] = 0;
						//for (j=0;j<win;j++) {
						//  P[x+win][j][d]= ((MM_U_8 *)I1->p[j].m)[x+win+offset] * ((MM_U_8 *)I2->p[j].m)[x+win+d];
						//  Q[x+win][d] += P[x+win][j][d];
						//}

						// reg 5&4 = Q
						pxor_r2r (mm5,mm5);

						// window down image
						for (j=0;j<win;j++) {
							I1_I2MULTISUM(I1, I2,x_win+offset,j,x_win+d,j+E);
							movq_r2m (mm0,((MM_U_32 *)P[j]->p[d].m)[x_win]);
							// Add curr P to Q
							paddd_r2r (mm0,mm5);
						}
						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);

					} // y<0 
					else
					{

						// Equivalent C function
						// P is [x_max][win][d_max] the win dimension is a wrap around buffer.
						//Q[x+win][d] -= P[x+win][y%win][d];
						//P[x+win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[x+win+offset] * ((MM_U_8 *)I2->p[y+win].m)[x+win+d];
						//Q[x+win][d] += P[x+win][y%win][d];

						// reg 5&4 = Q
						movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm5);
						// subtract prev P
						psubd_m2r (((MM_U_32 *)P[yMODwin]->p[d].m)[x+win],mm5);
						// Add curr P to Q
						I1_I2MULTISUM(I1, I2,x_win+offset,y_win,x_win+d,y_win+E);
						paddd_r2r (mm0,mm5);

						// Store Q
						movq_r2m (mm5,((MM_U_32 *)Q->p[d].m)[x_win]);

						// Store P
						movq_r2m (mm0,((MM_U_32 *)P[yMODwin]->p[d].m)[x_win]);

					} // y>=0

					// Equivalent C function
					//N[x+1][d] = N[x][d] - Q[x][d] + Q[x+win][d];

					// reg 6 & 6 = N
					movq_m2r (((MM_U_32 *)N->p[d].m)[x],mm6);  

					// Sub curr Q[x][d] from N
					psubd_m2r (((MM_U_32 *)Q->p[d].m)[x],mm6);

					// Add Q[x+win][d] to N
					movq_m2r (((MM_U_32 *)Q->p[d].m)[x_win],mm1);
					paddd_r2r (mm1,mm6);

					// Store N[x+1][d]
					movq_r2m (mm6,((MM_U_32 *)N->p[d].m)[x+2]);

				} // x>=0
				// Equivalent C function              
				//C1 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
				//if (C1 > max1) {
				//   max1 = C1;
				//   max1_d = d;
				//}
				//C2 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);
				//if (C2 > max2) {
				//  max2 = C2;
				//  max2_d = d;
				//}

#ifdef FAST_SQRT
				curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
				curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#else
				curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
				curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#endif


				//curr.ud[0] = ((MM_U_32 *)N->p[d].m)[x+2]/fast_sqrt(((MM_U_32 *)M)[x+d+2]);
				//curr.ud[1] = ((MM_U_32 *)N->p[d].m)[x+3]/fast_sqrt(((MM_U_32 *)M)[x+d+3]);


				//curr.ud[0] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
				//curr.ud[1] = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);

				// reg 5 = curr 
				movq_m2r(curr,mm5);
				movq_r2r(mm5,mm0);
				//PRINTREG32(mm5,"curr");

				// reg 6 = max
				movq_m2r(max,mm6);
				movq_r2r(mm6,mm2);
				//PRINTREG32(mm6,"max");

				// reg 2 becomes mask
				pcmpgtd_r2r(mm0,mm2);
				movq_r2r(mm2,mm1);
				//PRINTREG32(mm2,"mask");

				// reg 0 is preserved values larger than max
				movq_r2r(mm6,mm0);
				pand_r2r(mm2,mm0);

				// reg 1 is curr
				pandn_r2r(mm5,mm1);

				// reg 1 is new max
				por_r2r(mm0,mm1);
				movq_r2m(mm1,max);
				//PRINTREG32(mm1,"new_max");

				// reg 1 is preserved d values
				movq_m2r(max_d,mm3);
				movq_r2r(mm2,mm1);
				pand_r2r(mm3,mm1);

				// reg 0 = current d value
				curr_d.ud[0] = d;
				curr_d.ud[1] = d;    

				movq_m2r(curr_d,mm4);
				pandn_r2r(mm4,mm2);

				// reg 6 = new d's for minimum values
				por_r2r(mm2,mm1);
				movq_r2m(mm1,max_d);
				//PRINTREG32(mm1,"new_max_d");


			}// for d          
			//printf("%u\n",max.ud[0]);
			//printf("%u\n",max.ud[1]);
			// if (max.ud[0]<MAX_THRESH)

			/***************************************************
			  Subpixel interpolation
			  weighted average of best match and each neighbour
			  TODO : have look to see if this does something ! 
			 *****************************************************/

			if ((overall_min_d_lr>0)&&(overall_min_d_lr<d_max)) {

				subpix_denom = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1] - overall_min_lr - overall_min_lr +((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1];
				subpix_d = (((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1]- ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1])<<shift;

				//fprintf(stderr,"%d %d %d ",overall_min_d_lr,subpix_d,subpix_denom);
				if (subpix_denom) {
					subpix_d /= (subpix_denom);
					subpix_d += (overall_min_d_lr<<shift);
				} else {
					subpix_d = 0;
					//fprintf(stderr," %d  %d\n ",subpix_d,subpix_d/8);

					((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+1] = subpix_d;
				} 
			} else {
				((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+1] = (overall_min_d_lr<<shift);
			}

			((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+2] = (max_d.ub[0]<<shift);
			//else
			//((MM_U_8 *)Out->p[y+1].m)[x+2] = 0;
			//if (max.ud[1]<MAX_THRESH)
			((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+3] = (max_d.ub[4]<<shift);
			//else
			//((MM_U_8 *)Out->p[y+1].m)[x+3] = 0;


		}// for x
	}// for y

	emms();
	return 0;
}
#endif

//----------------------------------------------------------------
//----------------------------------------------------------------
//----------------------------------------------------------------
//----------------------------------------------------------------
#ifndef movdqu_m2r
#define  movdqu_m2r(var, reg)  sse_m2r(movdqu, var, reg)
#endif
#ifndef movdqu_r2m
#define  movdqu_r2m(reg, var)  sse_r2m(movdqu, reg, var)
#endif
#ifndef movdqa_m2r
#define  movdqa_m2r(var, reg)  sse_m2r(movdqa, var, reg)
#endif
#ifndef movdqa_r2m
#define  movdqa_r2m(reg, var)  sse_r2m(movdqa, reg, var)
#endif

#ifndef movdqa_r2r
#define  movdqa_r2r(reg1, reg2)  sse_r2r(movdqa, reg1, reg2)
#endif

// XI1_I2MULTISUM2
// I1(x1,y1)*I2(x2,y2) +
// I1(x1+1,y1)*I2(x2+1,y2)
#define XI1_I2MULTISUM2(I1,I2,x1,y1,x2,y2) \
{\
  movdqu_m2r(((MM_U_8 *)I1->p[y1].m)[x1],xmm2);\
  movdqu_m2r(((MM_U_8 *)I2->p[y2].m)[x2],xmm3);\
  movdqa_r2r(xmm2,xmm0);\
  movdqa_r2r(xmm3,xmm1);\
  punpcklbw_r2r(xmm7,xmm0);\
  punpcklbw_r2r(xmm7,xmm1);\
  pmaddwd_r2r(xmm1,xmm0);\
  psrlq_i2r(8,xmm2);\
  psrlq_i2r(8,xmm3);\
  punpcklbw_r2r(xmm7,xmm2);\
  punpcklbw_r2r(xmm7,xmm3);\
  pmaddwd_r2r(xmm3,xmm2);\
  punpckldq_r2r(xmm2,xmm0);\
}


// XI1_I2MULTISUM4
// I1(x1,y1)*I2(x2,y2) +
// I1(x1+1,y1)*I2(x2+1,y2) +
// I1(x1+2,y1)*I2(x2+2,y2) +
// I1(x1+3,y1)*I2(x2+3,y2)
#define XI1_I2MULTISUM4(I1,I2,x1,y1,x2,y2) \
{\
  movdqu_m2r(((MM_U_8 *)I1->p[y1].m)[x1],xmm0);\
  movdqu_m2r(((MM_U_8 *)I2->p[y2].m)[x2],xmm1);\
  movdqa_r2r(xmm0,xmm2);\
  movdqa_r2r(xmm1,xmm3);\
  punpckhbw_r2r(xmm7,xmm0);\
  punpckhbw_r2r(xmm7,xmm1);\
  psllq_i2r(8,xmm2);\
  psllq_i2r(8,xmm3);\
  pmaddwd_r2r(xmm1,xmm0);\
  punpckhbw_r2r(xmm7,xmm2);\
  punpckhbw_r2r(xmm7,xmm3);\
  pmaddwd_r2r(xmm3,xmm2);\
  movdqa_r2r(xmm2,xmm3);\
  punpckhdq_r2r(xmm0,xmm2);\
  punpckldq_r2r(xmm0,xmm3);\
  paddd_r2r(xmm3,xmm2);\
  movdqa_r2r(xmm2,xmm0);\
}


// Equivalent C function
// Q(x,y+1,d) = Q(x,y,d) - P(x,y) + P(x,y+win)
// assumes new P is in reg 0
// Q is in mm5 for later use
#define XQinc(Q,P,x,y,d) \
{\
      /*  reg 5&4 = Q */ \
      movdqu_m2r (((MM_U_32 *)Q->p[d].m)[x],xmm5);\
      paddd_r2r (xmm0,xmm5);\
            /* subtract prev P */ \
      movdqu_m2r (((MM_U_32 *)P[y]->p[d].m)[x],xmm1);\
      psubd_r2r (xmm1,xmm5);\
            /* Store Q */ \
      movdqa_r2m (xmm5,((MM_U_32 *)Q->p[d].m)[x]);\
}

// Equivalent C function
//N[x+1][d] = N[x][d] - Q[x][d] + Q[x+win][d];
#define XNinc(N,x,d,x_win) \
{\
  /* reg 6 & 6 = N */ \
  movdqu_m2r (((MM_U_32 *)N->p[d].m)[x],xmm6);\
  /* Sub curr Q[x][d] from N */ \
  movdqu_m2r (((MM_U_32 *)Q->p[d].m)[x],xmm1);\
  psubd_m2r (xmm1,xmm6);\
  /* Add Q[x+win][d] to N */ \
  movdqu_m2r (((MM_U_32 *)Q->p[d].m)[x_win],xmm1);\
  paddd_r2r (xmm1,xmm6);\
  /* Store N[x+1][d] */ \
  movdqa_r2m (xmm6,((MM_U_32 *)N->p[d].m)[x+4]);\
}

// Equivalent C function
// S(x,y+1) = S(x,y) - R(x,y) + R(x,y+win)
// assumes new R is in reg 0
// S is in mm5 for later use
#define XSinc(S,R,x,y) \
{\
      /*  reg 5 = S */ \
      movdqu_m2r (((MM_U_32 *)S)[x],xmm5);\
            /* Add R to S */ \
            paddd_r2r (xmm0,xmm5);\
            /* subtract prev R */ \
      movdqu_m2r (((MM_U_32 *)R->p[y].m)[x],xmm1);\
      psubd_r2r (xmm1,xmm5);\
            /* Store Q */ \
      movdqa_r2m (xmm5,((MM_U_32 *)S)[x]);\
}

// Equivalent C function
//M[x+1][y] = M[x][y] - S[x][y] + S[x+win][y];
#define XMinc(M,S,x,x_win) \
{\
  /* reg 6 & 6 = M */ \
  movdqu_m2r (((MM_U_32 *)M)[x],xmm6);\
  /* Sub curr S[x][y] from M */ \
  movdqu_m2r (((MM_U_32 *)S)[x],xmm1);\
  psubd_r2r (xmm1,xmm6);\
  /* Add S[x+win][y] to M */ \
  movdqu_m2r (((MM_U_32 *)S)[x_win],xmm1);\
  paddd_r2r (xmm1,xmm6);\
  /* Store M[x+1][y] */ \
  movdqu_r2m (xmm6,((MM_U_32 *)M)[x+4]);\
}


int MMXRecursive_Depthmap_NCC3(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out)
{

  static int allocated = 0;
  int y_max;
  int x_max;

  const int win = 16;
  const int d_max = 64;

  short d_2 = d_max/2;


  register int i, j, k;
  register int d,x,y;

  register int y_win, x_win, yMODwin;

  sse_t curr, curr_d, max, max_d;
  sse_t fcurr;
  static  MMXMatrix **P, *Q, *N;
  static MMXMatrix *R;
  static MM_U_32 *S, *M;
  //static MMXMatrix *sse_tmp;
  static MM_F_32 *MFINV;
  
  x_max = I1->cols;
  y_max = I1->rows;
  

  if ( !allocated) {
    // Numerator matrix
    P = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*win); 
    for (i=0;i<win;i++)
      P[i] = MMXMatrixAlloc(MMT_F_32, x_max, d_max);
    Q = MMXMatrixAlloc(MMT_F_32, x_max, d_max);
    N = MMXMatrixAlloc(MMT_F_32, x_max, d_max);

    // Denominator matrix
    S = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
    M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*x_max); 
    R = MMXMatrixAlloc(MMT_F_32, x_max, win);
    //MINV = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(x_max)); 
    //sse_tmp = MMXMatrixAlloc(MMT_F_32, 1, x_max);
    //MFINV = (MM_F_32 *)sse_tmp->p[0].s;
    MFINV = (MM_F_32 *) Alloc(sizeof(MM_F_32)*(x_max)); 

    allocated = 1;
  }

  emms();
  // clear reg 7
  pxor_r2r (xmm7,xmm7);
 
  // compute M for x=0 y=0
  // M[0] -> M[d_max]
  // S[0] -> S[d_max+win-2]
  for (k=0;k<=d_max;k+=4){
    //fprintf(stderr,"depthmap: k:%d \n",k);
    //M[k]=0;
    //// window across image
    //for(i=k;i<k+win;i++) {
    //  S[i]=0;
    //  // window down image
    //  for (j=0;j<win;j++) {
    //    R[i][j]= ((MM_U_8 *)I2->p[j].m)[i] * ((MM_U_8 *)I2->p[j].m)[i];
    //    S[i] += R[i][j];
    //  } 
    //  M[k] += S[i];
    //}

    // reg 6 = M
    pxor_r2r (xmm6,xmm6);
    for(i=k;i<k+win;i+=4) {
      // reg 5 = S
      pxor_r2r (xmm5,xmm5);
      for(j=0;j<win;j++) {
  // calc R
  XI1_I2MULTISUM4(I2, I2, i,j,i,j);
  // Store new R
  movdqu_r2m (xmm0,((MM_U_32 *)R->p[j].m)[i]);
  // Add R to S
  paddd_r2r (xmm0,xmm5);
      }
      // Store new S
      movdqa_r2m (xmm5,((MM_U_32 *)S)[i]);
      // Add curr S to M
      paddd_r2r (xmm5,xmm6);
    }
    // Store M[0]
    movdqa_r2m (xmm6,((MM_U_32 *)M)[k]);    

    //sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
    //sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
    //((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
    //((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);
    //cvtpi2ps_r2r(xmm6, xmm6);
    //movlps_r2r(xmm6, mm6);
    cvtpi2ps_r2r(mm6, xmm6);
    movlhps_r2r(xmm6,xmm6);
    //movhps_r2r(xmm6, mm6);
    cvtpi2ps_r2r(mm6, xmm6);
    rsqrtps_r2r(xmm6,xmm6);
    // Store MFINV[0]
    movlps_r2m(xmm6,((MM_F_32 *)MFINV)[k]);    
    
  }

  // compute N for x=0 & y=0
  // N->p[0][0]  -> N->p[d_max-1][0]
  for (d=0;d<d_max;d++) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    //N[0][d]=0;
    //// window across image
    //for(i=0;i<=win;i++) {
    //  Q[i][d] = 0;
    //  // window down image
    // for (j=0;j<win;j++) {
    //   P[i][j][d]= ((MM_U_8 *)I1->p[j].m)[i+d_2] * ((MM_U_8 *)I2->p[j].m)[i+d];
    //   Q[i][d] += P[i][j][d];       
    // } 
    // if (i<win)
    //   N[0][d] += Q[i][d];
    //}

    // reg 6 = N
    pxor_r2r (xmm6,xmm6);
    for(i=0;i<win;i+=4) {
      // reg 5 = Q
      pxor_r2r (xmm5,xmm5);
      for (j=0;j<win;j++) {
  // calc new P   
  XI1_I2MULTISUM4(I1, I2, i+d_2,j,i+d,j);
  // Store P
  movdqa_r2m (xmm0,((MM_U_32 *)P[j]->p[d].m)[i]);
  // Add P to Q
  paddd_r2r (xmm0,xmm5);
      }
      // Store Q
      movdqa_r2m (xmm5,((MM_U_32 *)Q->p[d].m)[i]);
      // Add curr Q to N
      paddd_r2r (xmm5,xmm6);
    }
    // Store N[d][0]
    movdqa_r2m (xmm6,((MM_U_32 *)N->p[d].m)[0]);
    
    // Do for i=win
    // reg 5 = Q
    pxor_r2r (xmm5,xmm5);
    for (j=0;j<win;j++) {
      // calc new P  
      XI1_I2MULTISUM4(I1, I2, win+d_2,j,win+d,j);
      // Store P
      movdqa_r2m (xmm0,((MM_U_32 *)P[j]->p[d].m)[win]);
      // Add P to Q
      paddd_r2r (xmm0,xmm5);
    }
    // Store Q
    movdqa_r2m (xmm5,((MM_U_32 *)Q->p[d].m)[win]);
  } // for d

  //--- Main y_max loop ---
  for(y=-1;y<y_max-win-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    y_win = y+win;
    yMODwin = y%win;


    // compute M for x=0    
    if (y>=0) {
      for (k=0;k<=d_max;k+=4){
  //fprintf(stderr,"depthmap: k:%d \n",k);
  // M[k]=0;
  // window across image
  //for(i=k;i<k+win;i++) {
  //  S[i] -= R[i][y%win];
  //  R[i][y%win] = ((MM_U_8 *)I2->p[y+win].m)[i] * ((MM_U_8 *)I2->p[y+win].m)[i];
  //  S[i] += R[i][y%win];  
  //  M[k] += S[i];
  //}
  // reg 6&6 = M
  pxor_r2r (xmm6,xmm6);
  for(i=k;i<k+win;i+=4) {
    // calc new R
    XI1_I2MULTISUM4(I2, I2, i,y_win,i,y_win);
    XSinc(S,R,i,yMODwin);
    // Store R
    movdqa_r2m (xmm0,((MM_U_32 *)R->p[yMODwin].m)[i]);
    // Add curr S to M
    paddd_r2r (xmm5,xmm6);
  } // for i
  // Store M[0]
  movdqa_r2m (xmm6,((MM_U_32 *)M)[k]);

  //sqrt2 = fast_sqrt(((MM_U_32 *)M)[k]);
  //sqrt3 = fast_sqrt(((MM_U_32 *)M)[k+1]);
  //((MM_U_32 *)MINV)[k] = ((((unsigned long long)0xffffffff) + sqrt2)/sqrt2);
  //((MM_U_32 *)MINV)[k+1] = ((((unsigned long long)0xffffffff) + sqrt3)/sqrt3);

  //cvtpi2ps_r2r(xmm6, xmm6);
  //movlps_r2r(xmm6, mm6);
  cvtpi2ps_r2r(mm6, xmm6);
  //movlhps_r2r(xmm6,xmm6);
  //movhps_r2r(xmm6, mm6);
  cvtpi2ps_r2r(mm6, xmm6);
  rsqrtps_r2r(xmm6,xmm6);
  // Store MFINV[0]
  movlps_r2m(xmm6,((MM_F_32 *)MFINV)[k]);    

      } // for k
    } // if y >= 0

    //--- Main X loop ---
    for(x=-4;x<x_max-win-d_max-4;x+=4) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      x_win= x+win;
   
      // compute M for y=0 
      if (y<0) {
  // Equivalent C function
  //S[x+d_max+win] = 0;
  //for (j=0;j<win;j++) {
  //  R[x+d_max+win][j]= ((MM_U_8 *)I2->p[j].m)[x+d_max+win] * ((MM_U_8 *)I2->p[j].m)[x+d_max+win];
  //  S[x+d_max+win] += R[x+d_max+win][j];
  //}
  // reg 5 = S
  pxor_r2r (xmm5,xmm5);
  // window down image
  for (j=0;j<win;j++) {
    //fprintf(stderr,"depthmap: j:%d\n",j);
    // Calc R
    XI1_I2MULTISUM4(I2, I2,x_win+d_max,j,x_win+d_max,j);
    // Store R
    movdqa_r2m (xmm0,((MM_U_32 *)R->p[j].m)[x_win+d_max]);
    // Add curr R to S
    paddd_r2r (xmm0,xmm5);
  }
  // Store S
  movdqa_r2m (xmm5,((MM_U_32 *)S)[x_win+d_max]);

      }  
      else
      {
  // Equivalent C function
  // P is [x_max][win] the win dimension is a wrap around buffer.    
  //S[x+d_max+win] -= R[x+d_max+win][y%win];
  //R[x+d_max+win][y%win] = ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win] * ((MM_U_8 *)I2->p[y+win].m)[x+d_max+win];
  //S[x+d_max+win] += R[x+d_max+win][y%win];

  XI1_I2MULTISUM4(I2, I2,x_win+d_max,y_win,x_win+d_max,y_win);
  XSinc(S,R,x_win+d_max,yMODwin);
  // Store R
  movdqa_r2m (xmm0,((MM_U_32 *)R->p[yMODwin].m)[x_win+d_max]);
      }
  
      // Equivalent C function
      //M[x+d_max+1] = M[x+d_max] - S[x+d_max] + S[x+d_max+win];
      XMinc(M,S,x+d_max,x_win+d_max);

      //sqrt2 = fast_sqrt(((MM_U_32 *)M)[x+d_max+2]);
      //sqrt3 = fast_sqrt(((MM_U_32 *)M)[x+d_max+3]);
      //((MM_U_32 *)MINV)[x+d_max+2] = ((((unsigned long long)0xffffffff) + sqrt2 )/ sqrt2);
      //((MM_U_32 *)MINV)[x+d_max+3] = ((((unsigned long long)0xffffffff) + sqrt3 )/ sqrt3);

      //cvtpi2ps_r2r(xmm6, xmm6);
      //movlps_r2r(xmm6, mm6);
      cvtpi2ps_r2r(mm6, xmm6);
      movlhps_r2r(xmm6,xmm6);
      //movhps_r2r(xmm6, mm6);
      cvtpi2ps_r2r(mm6, xmm6);
      rsqrtps_r2r(xmm6,xmm6);
      // Store MFINV[0]
      movlps_r2m(xmm6,((MM_F_32 *)MFINV)[k]);    


      //printf("%d %d %ld %ld\n",y,x,((MM_U_32 *)M)[x+2],((MM_U_32 *)M)[x+3]);
      
      max_d.i[0] = 0;
      max_d.i[1] = 0;    
      max_d.i[2] = 0;
      max_d.i[3] = 0;    

      max.i[0] = 0;
      max.i[1] = 0;    
      max.i[2] = 0;
      max.i[3] = 0;    

      //--- Main D loop ---
      for (d=0;d<d_max;d++) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if (x<0) {
    if (y>=0) {
      // Equivalent C function
      //N[0][d]=0;
      //for(i=0;i<win;i++) {
      //  Q[i][d] -= P[i][y%win][d];
      //  P[i][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[i+d_2] * ((MM_U_8 *)I2->p[y+win].m)[i+d];
      //  Q[i][d] += P[i][y%win][d];
      //  N[0][d] += Q[i][d];
      //}
      //Q[win][d] -= P[win][y%win][d];
      //P[win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[win+d_2] * ((MM_U_8 *)I2->p[y+win].m)[win+d];
      //Q[win][d] += P[win][y%win][d];
    
      // reg 6&6 = N
      pxor_r2r (xmm6,xmm6);
      for(i=0;i<win;i+=2) {
        XI1_I2MULTISUM4(I1, I2, i+d_2,y_win,i+d,y_win);
        XQinc(Q,P,i,yMODwin,d);
        // Store P
        movdqa_r2m (xmm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
        // Add curr Q to N
        paddd_r2r (xmm5,xmm6);
      } // for i
      // Store N[d][0]
      movq_r2m (xmm6,((MM_U_32 *)N->p[d].m)[0]);

      // Do for i=win

      // lbw in 0
      XI1_I2MULTISUM4(I1, I2, i+d_2,y_win,i+d,y_win);
            XQinc(Q,P,i,yMODwin,d);
      // Store new P
      movdqa_r2m (xmm0,((MM_U_32 *)P[yMODwin]->p[d].m)[i]);
    } // y>=0
  } // x <0
  else
  {
    if (y<0) {
      // Equivalent C function
      //Q[x+win][d] = 0;
      //for (j=0;j<win;j++) {
      //  P[x+win][j][d]= ((MM_U_8 *)I1->p[j].m)[x+win+d_2] * ((MM_U_8 *)I2->p[j].m)[x+win+d];
      //  Q[x+win][d] += P[x+win][j][d];
      //}

      // reg 5&4 = Q
      pxor_r2r (xmm5,xmm5);
      
      // window down image
      for (j=0;j<win;j++) {
        XI1_I2MULTISUM4(I1, I2,x_win+d_2,j,x_win+d,j);
        movdqa_r2m (xmm0,((MM_U_32 *)P[j]->p[d].m)[x_win]);
        // Add curr P to Q
        paddd_r2r (xmm0,xmm5);
      }
      // Store Q
      movdqa_r2m (xmm5,((MM_U_32 *)Q->p[d].m)[x_win]);
      
    } // y<0 
    else
    {
      // Equivalent C function
      // P is [x_max][win][d_max] the win dimension is a wrap around buffer.
      //Q[x+win][d] -= P[x+win][y%win][d];
      //P[x+win][y%win][d] = ((MM_U_8 *)I1->p[y+win].m)[x+win+d_2] * ((MM_U_8 *)I2->p[y+win].m)[x+win+d];
      //Q[x+win][d] += P[x+win][y%win][d];

      // Add curr P to Q
      XI1_I2MULTISUM4(I1, I2,x_win+d_2,y_win,x_win+d,y_win);

            XQinc(Q,P,x_win,yMODwin,d);
      
      // Store P
      movdqa_r2m (xmm0,((MM_U_32 *)P[yMODwin]->p[d].m)[x_win]);

    } // y>=0

          Ninc(N,x,d,x_win);
    
  } // x>=0
  // Equivalent C function              
  //C1 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+2]*((MM_U_32 *)N->p[d].m)[x+2]/((MM_U_32 *)M)[x+d+2]);
  //if (C1 > max1) {
  //   max1 = C1;
  //   max1_d = d;
  //}
  //C2 = ((unsigned long long)((MM_U_32 *)N->p[d].m)[x+3]*((MM_U_32 *)N->p[d].m)[x+3]/((MM_U_32 *)M)[x+d+3]);
  //if (C2 > max2) {
  //  max2 = C2;
  //  max2_d = d;
  //}

  //cvtpi2ps_r2r(xmm6, xmm5);
  //movlps_r2r(xmm6, mm6);
  cvtpi2ps_r2r(mm6, xmm6);
  movlhps_r2r(xmm6,xmm6);
  //movhps_r2r(xmm6, mm6);
  cvtpi2ps_r2r(mm6, xmm5);
  movdqu_m2r(((MM_F_32 *)MFINV)[x+d+2],xmm0);      
  //movhlps_r2r(xmm0,xmm0);      
  mulps_r2r(xmm0,xmm5);      

  movlps_r2m(xmm5,fcurr);      

  //cvtps2pi_r2r(xmm5, mm5);

  curr.i[0] = (unsigned long) ((MM_F_32 *)fcurr.sf)[2];
  curr.i[1] = (unsigned long) ((MM_F_32 *)fcurr.sf)[3];
  
  // reg 5 = curr 
  movdqa_m2r(curr,xmm5);
        movdqa_r2r(xmm5,xmm0);
  //PRINTREG32(mm5,"curr");

  // reg 6 = max
        movdqa_m2r(max,xmm6);
  movdqa_r2r(xmm6,xmm2);
  //PRINTREG32(mm6,"max");
        
  // reg 2 becomes mask
        pcmpgtd_r2r(xmm0,xmm2);
        movdqa_r2r(xmm2,xmm1);
  //PRINTREG32(mm2,"mask");

        // reg 0 is preserved values larger than max
  movdqa_r2r(xmm6,xmm0);
  pand_r2r(xmm2,xmm0);
  
  // reg 1 is curr
  pandn_r2r(xmm5,xmm1);
  
  // reg 1 is new max
  por_r2r(xmm0,xmm1);
  movdqa_r2m(xmm1,max);
  //PRINTREG32(mm1,"new_max");

  // reg 1 is preserved d values
  movdqa_m2r(max_d,xmm3);
        movdqa_r2r(xmm2,xmm1);
  pand_r2r(xmm3,xmm1);

  // reg 0 = current d value
  curr_d.i[0] = d;
  curr_d.i[1] = d;    

  movdqa_m2r(curr_d,xmm4);
  pandn_r2r(xmm4,xmm2);
   
  // reg 6 = new d's for minimum values
  por_r2r(xmm2,xmm1);
  movdqa_r2m(xmm1,max_d);
  //PRINTREG32(mm1,"new_max_d");


      }// for d            
      ((MM_U_8 *)Out->p[y+1].m)[x+4] = max_d.i[0];
      ((MM_U_8 *)Out->p[y+1].m)[x+5] = max_d.i[4];
      ((MM_U_8 *)Out->p[y+1].m)[x+6] = max_d.i[8];
      ((MM_U_8 *)Out->p[y+1].m)[x+7] = max_d.i[12];
      
      //emms();
      // clear reg 7
      //pxor_r2r (mm7,mm7);

    }// for x
  }// for y

  emms();
  return 0;
}

















