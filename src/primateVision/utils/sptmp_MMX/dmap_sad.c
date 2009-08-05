


#include "sptmpmmx.h"
#include "matrices_cache.h"

#include "sse.h"



//#define I1_I2(I1,I2,a,b,c,d) abs(((MM_U_8 *)(I1)->p[(b)].m)[(a)]-((MM_U_8 *)(I2)->p[(d)].m)[(c)])


// I1_I2:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1+1,y1)-I2(x2+1,y2))
// abs(I1(x1+2,y1)-I2(x2+2,y2))
// abs(I1(x1+3,y1)-I2(x2+3,y2))
#define I1_I2(I1,I2,x1,y1,x2,y2) \
{\
  movq_m2r(((MM_U_8 *)I1->p[y1].m)[x1],mm0);\
  psubusb_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm0);\
  movq_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm2);\
  psubusb_m2r(((MM_U_8 *)I1->p[y1].m)[x1], mm2);\
  por_r2r(mm2,mm0);\
  punpcklbw_r2r(mm7,mm0);\
}

// I1_I2S
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1+1,y1)-I2(x2+1,y2))
// abs(I1(x1+2,y1)-I2(x2+2,y2))
// abs(I1(x1+3,y1)-I2(x2+3,y2))
#define I1_I2S(I1,I2,x1,y1,x2,y2) \
{\
  movq_m2r(((MM_U_8 *)I1->p[y1].m)[x1],mm0);\
  psubusb_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm0);\
  movq_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm1);\
  psubusb_m2r(((MM_U_8 *)I1->p[y1].m)[x1], mm1);\
  por_r2r(mm1,mm0);\
  movq_r2r(mm0,mm2);\
  punpckhbw_r2r(mm7,mm2);\
  movq_r2r(mm2,mm1);\
  movq_r2r(mm0,mm3);\
  psllq_i2r(8,mm3);\
  punpckhbw_r2r(mm7,mm3);\
  paddw_r2r(mm3,mm1);\
  movq_r2r(mm0,mm2);\
  psllq_i2r(16,mm2);\
  punpckhbw_r2r(mm7,mm2);\
  paddw_r2r(mm2,mm1);\
  movq_r2r(mm0,mm3);\
  psllq_i2r(24,mm3);\
  punpckhbw_r2r(mm7,mm3);\
  paddw_r2r(mm3,mm1);\
}


// I1_I2Sptr
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1+1,y1)-I2(x2+1,y2))
// abs(I1(x1+2,y1)-I2(x2+2,y2))
// abs(I1(x1+3,y1)-I2(x2+3,y2))
#define I1_I2Sptr(I1,I2) \
{\
  movq_m2r(*I1,mm0);\
  psubusb_m2r(*I2,mm0);\
  movq_m2r(*I2,mm1);\
  psubusb_m2r(*I1, mm1);\
  por_r2r(mm1,mm0);\
  movq_r2r(mm0,mm2);\
  punpckhbw_r2r(mm7,mm2);\
  movq_r2r(mm2,mm1);\
  movq_r2r(mm0,mm3);\
  psllq_i2r(8,mm3);\
  punpckhbw_r2r(mm7,mm3);\
  paddw_r2r(mm3,mm1);\
  movq_r2r(mm0,mm2);\
  psllq_i2r(16,mm2);\
  punpckhbw_r2r(mm7,mm2);\
  paddw_r2r(mm2,mm1);\
  movq_r2r(mm0,mm3);\
  psllq_i2r(24,mm3);\
  punpckhbw_r2r(mm7,mm3);\
  paddw_r2r(mm3,mm1);\
}


// I1_I2D:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1,y1)-I2(x2+1,y2))
// abs(I1(x1,y1)-I2(x2+2,y2))
// abs(I1(x1,y1)-I2(x2+3,y2))
#define I1_I2D(I1,I2,x1,y1,x2,y2) \
{\
  movq_m2r(((MM_U_8 *)I1->p[y1].m)[x1],mm0);\
  punpcklbw_r2r(mm0,mm0);\
  punpcklwd_r2r(mm0,mm0);\
  punpckldq_r2r(mm0,mm0);\
  movq_r2r(mm0,mm1);\
  movq_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm2);\
  movq_r2r(mm2,mm3);\
  psubusb_r2r(mm0, mm2);\
  psubusb_r2r(mm3, mm1);\
  por_r2r(mm2,mm1);\
  punpcklbw_r2r(mm7,mm1);\
}


// I1_I2D:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1,y1)-I2(x2+1,y2))
// abs(I1(x1,y1)-I2(x2+2,y2))
// abs(I1(x1,y1)-I2(x2+3,y2))
#define I1_I2Dptr(pI1,pI2) \
{\
  movq_m2r(*pI1,mm0);\
  punpcklbw_r2r(mm0,mm0);\
  punpcklwd_r2r(mm0,mm0);\
  punpckldq_r2r(mm0,mm0);\
  movq_r2r(mm0,mm1);\
  movq_m2r(*pI2,mm2);\
  movq_r2r(mm2,mm3);\
  psubusb_r2r(mm0, mm2);\
  psubusb_r2r(mm3, mm1);\
  por_r2r(mm2,mm1);\
  punpcklbw_r2r(mm7,mm1);\
}

// I1_I2DU8:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1,y1)-I2(x2+1,y2))
// abs(I1(x1,y1)-I2(x2+2,y2))
// abs(I1(x1,y1)-I2(x2+3,y2))
// abs(I1(x1,y1)-I2(x2+4,y2))
// abs(I1(x1,y1)-I2(x2+5,y2))
// abs(I1(x1,y1)-I2(x2+6,y2))
// abs(I1(x1,y1)-I2(x2+7,y2))
#define I1_I2DU8(I1,I2,x1,y1,x2,y2) \
{\
  movq_m2r(((MM_U_8 *)I1->p[y1].m)[x1],mm0);\
  punpcklbw_r2r(mm0,mm0);\
  punpcklwd_r2r(mm0,mm0);\
  punpckldq_r2r(mm0,mm0);\
  movq_r2r(mm0,mm1);\
  movq_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm2);\
  movq_r2r(mm2,mm3);\
  psubusb_r2r(mm0, mm2);\
  psubusb_r2r(mm3, mm1);\
  por_r2r(mm2,mm1);\
  pand_r2r(mm7,mm1);\
  psrlw_i2r(4,mm1);\
}


// I1_I2DU8:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1,y1)-I2(x2+1,y2))
// abs(I1(x1,y1)-I2(x2+2,y2))
// abs(I1(x1,y1)-I2(x2+3,y2))
// abs(I1(x1,y1)-I2(x2+4,y2))
// abs(I1(x1,y1)-I2(x2+5,y2))
// abs(I1(x1,y1)-I2(x2+6,y2))
// abs(I1(x1,y1)-I2(x2+7,y2))
#define I1_I2DU8ptr(I1,I2) \
{\
  movq_m2r(*I1,mm0);\
  punpcklbw_r2r(mm0,mm0);\
  punpcklwd_r2r(mm0,mm0);\
  punpckldq_r2r(mm0,mm0);\
  movq_r2r(mm0,mm1);\
  movq_m2r(*I2,mm2);\
  movq_r2r(mm2,mm3);\
  psubusb_r2r(mm0, mm2);\
  psubusb_r2r(mm3, mm1);\
  por_r2r(mm2,mm1);\
  pand_r2r(mm7,mm1);\
  psrlw_i2r(4,mm1);\
}


// I1_I2DEIGHT:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1,y1)-I2(x2+1,y2))
// abs(I1(x1,y1)-I2(x2+2,y2))
// abs(I1(x1,y1)-I2(x2+3,y2))
// abs(I1(x1,y1)-I2(x2+4,y2))
// abs(I1(x1,y1)-I2(x2+5,y2))
// abs(I1(x1,y1)-I2(x2+6,y2))
// abs(I1(x1,y1)-I2(x2+7,y2))
#define I1_I2DEIGHT(I1,I2,x1,y1,x2,y2) \
{\
  movq_m2r(((MM_U_8 *)I1->p[y1].m)[x1],mm0);\
  punpcklbw_r2r(mm0,mm0);\
  punpcklwd_r2r(mm0,mm0);\
  punpckldq_r2r(mm0,mm0);\
  movq_r2r(mm0,mm1);\
  movq_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm2);\
  psubusb_r2r(mm0, mm2);\
  psubusb_m2r(((MM_U_8 *)I2->p[y2].m)[x2], mm1);\
  por_r2r(mm2,mm1);\
  movq_r2r(mm1,mm0);\
  punpcklbw_r2r(mm7,mm0);\
  punpckhbw_r2r(mm7,mm1);\
}


// I1_I2DXMM:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1,y1)-I2(x2+1,y2))
// abs(I1(x1,y1)-I2(x2+2,y2))
// abs(I1(x1,y1)-I2(x2+3,y2))
// abs(I1(x1,y1)-I2(x2+4,y2))
// abs(I1(x1,y1)-I2(x2+5,y2))
// abs(I1(x1,y1)-I2(x2+6,y2))
// abs(I1(x1,y1)-I2(x2+7,y2))
#define I1_I2DXMM(I1,I2,x1,y1,x2,y2) \
{\
  movdqu_m2r(((MM_U_8 *)I1->p[y1].s)[x1],xmm0);\
  punpcklbw_r2r(xmm7,xmm0);\
  punpcklwd_r2r(xmm0,xmm0);\
  punpckldq_r2r(xmm0,xmm0);\
  movdqa_r2r(xmm0,xmm1);\
  punpckhdq_r2r(xmm0,xmm1);\
  movdqa_r2r(xmm1,xmm3);\
  movdqu_m2r(((MM_U_8 *)I2->p[y2].s)[x2],xmm2);\
  punpcklbw_r2r(xmm7,xmm2);\
  movdqa_r2r(xmm2,xmm0);\
  psubusw_r2r(xmm3, xmm2);\
  psubusw_r2r(xmm0, xmm1);\
  por_r2r(xmm2,xmm1);\
}


//-----------------------------------------------------------------------
//----------------------------------------------------------------
int Depthmap_SAD(const MMXMatrix *I1,const MMXMatrix *I2, MMXMatrix  *O, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{
  const int W = 16;
  const int E = 1;


  int X,Y, i, j;
  int sum, best=0, best_d=0;

  register int d,e,x,y;


  
  int w_2 = W/2;
  int e_2 = E/2;


  X = I1->cols;
  Y = I1->rows;

  
  for(y=0;y<Y;y++){
    fprintf(stderr,"depthmap: y:%d \n",y);

    for(x=0;x<X;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
  
      if ((y<(e_2+w_2))||(x<(D_OFF+w_2))||(y>(Y-1-e_2-w_2))||(x>(X-1-D_OFF-w_2))) {
  ((MM_U_8 *)O->p[y].m)[x]=0;
      }
      else
      {
        for (d=0;d<D;d++) {
          //fprintf(stderr,"depthmap: d:%d \n",d);
          for (e=0;e<E;e++) {
            //fprintf(stderr,"depthmap: d:%d \n",d);
            sum = 0;
            for(j=0;j<W;j++) {
              //fprintf(stderr,"depthmap: j:%d \n",j);
              
              for(i=0;i<W;i++) {
                //fprintf(stderr,"depthmap: i:%d \n",i);
                
                sum+= abs(((MM_U_8 *)I1->p[y-w_2+j].m)[x-w_2+i]-((MM_U_8 *)I2->p[y+e-e_2-w_2+j].m)[x+d-D_OFF-w_2+i]);
              }    
            }
            if (((d==0)&&(e==0))||(sum < best)) {
              best = sum;
              //best_d = e;
              best_d = d;
            }
          }
        }
        ((MM_U_8 *)O->p[y].m)[x]= best_d;
        
      }
    }
  }
  
  return 0;
}



//----------------------------------------------------------------
int Recursive_Depthmap_SAD(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;

  const int W = 16;


  register int i, j;
  int X,Y,min=0, min_d=0, P_temp;

  register int d,x,y;
  
  short ***P;
  short **Q;
  short **C;

  X = I1->cols;
  Y = I1->rows;
  

  if (*cache==0) {
    j=0;    
    *cache = MMXMatricesCacheAlloc(0,0,3 + X * (3 + W));
    (*cache)->G[j++] = P = (short ***)Alloc(sizeof(short **)*X);
    (*cache)->G[j++] = Q = (short **)Alloc(sizeof(short *)*X);
    (*cache)->G[j++] = C = (short **)Alloc(sizeof(short *)*X);
    for (x=0;x<X;x++) {
      (*cache)->G[j++] = Q[x] = (short *)Alloc(sizeof(short)*D);
      (*cache)->G[j++] = C[x] = (short *)Alloc(sizeof(short)*D);
      (*cache)->G[j++] = P[x] = (short **)Alloc(sizeof(short *)*W);
      for (i=0;i<W;i++) {
        (*cache)->G[j++] = P[x][i] = (short *)Alloc(sizeof(short)*D);
      }
    }     
  }
  else { 
    P = (short ***) (*cache)->G[0];
    Q =  (short **) (*cache)->G[1];
    C = (short **) (*cache)->G[2];
  }


  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    
    for(x=-1;x<X-D-1-W;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      for (d=0;d<D;d++) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {
          
          C[0][d]=0;
          // window across image
          for(i=0;i<W;i++) {
            Q[i][d] = 0;
            // window down image
            for (j=0;j<W;j++) {
              P[i][j][d]= abs(((MM_U_8 *)I1->p[j].m)[i+D_OFF] - ((MM_U_8 *)I2->p[j].m)[i+d]);
              Q[i][d] += P[i][j][d];
              //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
              //printf("P(%d,%d,%d):%d\n",i,j,d,P[i][j][d]);
              
            } 
            //printf("Q(%d,%d):%d",i,d,Q[i][d]);
            
            if (i<W)
              C[0][d] += Q[i][d];
          }
        }
        else if (x<0) {
          
          C[0][d]=0;
          for(i=0;i<W;i++) {
            P_temp = abs(((MM_U_8 *)I1->p[y+W].m)[i+D_OFF] - ((MM_U_8 *)I2->p[y+W].m)[i+d]);
            
            Q[i][d] = Q[i][d] - P[i][y%W][d] + P_temp ;
            P[i][y%W][d] = P_temp;
            if (i<W)
              C[0][d] += Q[i][d];
          }
        }
        else
          {
            if (y<0) {
              
              Q[x+W][d] = 0;
              for (j=0;j<W;j++) {
                P[x+W][j][d]= abs(((MM_U_8 *)I1->p[j].m)[x+W+D_OFF] - ((MM_U_8 *)I2->p[j].m)[x+W+d]);
                Q[x+W][d] += P[x+W][j][d];
              }
            }  
            else
              {
                // P is [X][W][D] the W dimension is a wrap around buffer.
                
                P_temp = abs(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF] - ((MM_U_8 *)I2->p[y+W].m)[x+W+d]);
                Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
                P[x+W][y%W][d] = P_temp;
              }
            
            C[x+1][d] = C[x][d] - Q[x][d] + Q[x+W][d];
          }
        
        
        //printf("%d,%d,%d: %d\n",x,y,d, C[x+1][d]);
        
        if ((d==0)||(C[x+1][d] < min)) {
          min = C[x+1][d];
          min_d = d;
        }
      }// for d
      ((MM_U_8 *)Out->p[y+1].m)[x+1]= min_d; 
    }// for x
  }// for y
  
  /* Deallocate memory */
  //for (x=0;x<X;x++) {
  //  for (i=0;i<W;i++) {
  //    free(P[x][i]);
  //  }
  //  free(Q[x]);
  //  free(C[x]);
  //  free(P[x]);
  //} 
  //free(P);
  //free(Q);
  //free(C);
  return 0;
}










#define Qinc(Q,x,y,d,P) \
{\
     /* reg 4 = Q*/ \
     movq_m2r (((MM_U_16 *)Q->p[x].m)[d],mm4); \
     /* subtract prev P */\
     psubw_m2r (((MM_U_16 *)P[y]->p[x].m)[d],mm4);\
     /* Add curr P to Q*/\
     paddw_r2r (mm1,mm4);\
     /* Store Q*/ \
     movq_r2m (mm4,((MM_U_16 *)Q->p[x].m)[d]);\
}


// Equivalent C function
//C[d][x+1] = C[d][x] - Q[d][x] + Q[d][x+W];
// assumes new Q is in reg 4
#define Cinc(C,x,d,Q) \
{\
   /* reg 5 = C */ \
   movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);\
   /* Sub curr Q[x][d] from C */ \
   psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);\
   /* Add Q[x+W][d] to C */\
   /* paddw_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm5);*/\
   paddw_r2r (mm4,mm5);\
   /* Store C[x+1][d] */\
   movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);\
}


inline void find_min(MMXMatrix *C, mmx_t *pcurr_d_lr,
         mmx_t *pmin_lr, mmx_t *pmin_d_lr)
{
  // ---------------------------------------------
  // find minimum C Left->Right for C[x+1][d]      
  // ---------------------------------------------
  // first create & store 0x0001000100010001 in 6
  // for future reference
  pxor_r2r(mm6,mm6);
  pcmpeqw_r2r(mm1,mm1);
  psubw_r2r(mm1,mm6);


  // reg 5 = C 
  //movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
        movq_r2r(mm5,mm0);

  //reg 4 = minimum C
  movq_m2r(*pmin_lr,mm4);
  movq_r2r(mm4,mm2);

  //fill 1 with 0x80008000s (sign bit)
  movq_r2r(mm6,mm1);
  psllw_i2r(15,mm1);

  paddw_r2r(mm1,mm2);
  paddw_r2r(mm1,mm0);
  //PRINTREG16(mm2,"min_lr");
  //PRINTREG16(mm0,"C");
        
  // reg 2 becomes mask
        pcmpgtw_r2r(mm0,mm2);
  movq_r2r(mm2,mm1);
  
  //PRINTREG16(mm2,"mask");
  
  // reg 0 is C values smaller than minimum
  movq_r2r(mm5,mm0);
  pand_r2r(mm2,mm0);
    
  // reg 4 is preserved minimum
  pandn_r2r(mm4,mm1);
  
  // reg 1 is new minimum
  por_r2r(mm0,mm1);
  movq_r2m(mm1,*pmin_lr);
  //PRINTREG16(mm6,"new_min_lr");

  // reg 4 is d values for minimum
  movq_m2r(*pmin_d_lr,mm4);
  // reg 1 is preserved d values
        movq_r2r(mm2,mm1);
  pandn_r2r(mm4,mm1);

  // reg 3 = current d value
  // increase current d values by 4
  movq_m2r(*pcurr_d_lr,mm3);
  movq_r2r(mm3,mm0);


  // reg 0 = current d value for updated minimum
  pand_r2r(mm2,mm0);
  
  // reg 1 = new d's for minimum values
  por_r2r(mm0,mm1);
  movq_r2m(mm1,*pmin_d_lr);
  //PRINTREG16(mm1,"new_min_d_lr");

  // increase current d's
  // fill with 0x0004000400040004 
  psllw_i2r(2,mm6);
         paddw_r2r(mm6,mm3);
  
         //paddw_m2r(plus_four,mm3);
  movq_r2m(mm3,*pcurr_d_lr);
}


//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD2(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  const short W = 16;
  //const short W = 8;


  short Y,X;

  register short i, j;
  register short d,x,y;
  
  short w_2 = (W/2);
  short d_w_2_1 = D_OFF+w_2+1;

  unsigned short overall_min_lr;


  unsigned char overall_min_d_lr;

  unsigned char *pI1=NULL, *pI2=NULL;
  unsigned short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;

  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C1;
  MMXMatrix *C2;
  MMXMatrix *C;
  MMXMatrix *C_prev;
  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  
  X = I1->cols;
  Y = I1->rows;

  if (*cache==0) {
    i=0;    
    *cache = MMXMatricesCacheAlloc(W+3,0,0);
    (*cache)->M[i] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i] = C1 = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i] = C2 = MMXMatrixAlloc(MMT_U_16, D, X);
    P = (*cache)->M + 3;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, D, X);
  }
  else { 
    Q = (*cache)->M[0];
    C1 = (*cache)->M[1];
    C2 = (*cache)->M[2];
    P = (*cache)->M + 3;
  }


  emms();

  C = C2;
  C_prev = C1;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }
    
    for(x=-1;x<X-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      
      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);
      
      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;
      
      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4); 
      
      pxor_r2r (mm7,mm7); 

      if (x>=0) {
        pCd = &(((MM_U_16 *)C->p[x].m)[0]);
        pC1d = &(((MM_U_16 *)C->p[x+1].m)[0]);
        pQd  = &(((MM_U_16 *)Q->p[x].m)[0]);
        pQWd  = &(((MM_U_16 *)Q->p[x+W].m)[0]);
        if (y>=0) {
          pPWd = &(((MM_U_16 *)P[y%W]->p[x+W].m)[0]);
        }
        pI1  = &(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF]);
        pI2  = &(((MM_U_8 *)I2->p[y+W].m)[x+W]);
      }

      
      for (d=0;d<D-3;d+=4) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {      
          // Equivalent C function
          //C[d][0]=0; 
          //for(i=0;i<=W;i+=4) {
          //  Q[d][i] = 0;
          // window down image
          //  for (j=0;j<W;j++) {
          //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
          //  Q[d][i] += P[j][d][i];
          //  } 
          //  if (i<W)
          //    C[d][0] += Q[d][i];
          //}
    
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2D(I1, I2, i+D_OFF,j,i+d,j);
              //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
              //printf("P(%d,%d,%d):",i,j,d);
              //PRINTREG16(mm1,"P");
        
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
            //printf("Q(%d,%d):",i,d);
            // PRINTREG16(mm4,"Q");
    
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          
          // compute for i=W
          // reg 4 = Q
          pxor_r2r (mm4,mm4);  
          // window down image
          for (j=0;j<W;j++) {
            // lbw in 0
            I1_I2D(I1, I2, W+D_OFF,j,W+d,j);
            
            movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
            // Add curr P to Q
            paddw_r2r (mm1,mm4);
          }
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
         
    
        }
        else if (x<0) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i++) {
          // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
          // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
          // P[y%W][d][i] = P_temp;
          // if (i<W)
          //   C[d][0] += Q[d][i];
          //}
          
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            movq_m2r (((MM_U_16 *)Q->p[i].m)[d],mm4);  
            // Sub prev P from Q
            psubw_m2r (((MM_U_16 *)P[y%W]->p[i].m)[d],mm4);
            // Add curr P to Q
            // lbw in 0
            I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
            // Store P
            movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[i].m)[d]);
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C[d][0]
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          //PRINTREG16(mm5,"C");
          
          
          // Do for i=W
          // reg 4 = Q
          movq_m2r (((MM_U_16 *)Q->p[W].m)[d],mm4);  
          // Sub prev P from Q
          psubw_m2r (((MM_U_16 *)P[y%W]->p[W].m)[d],mm4);
          // Add curr P to Q
          // lbw in 0
          I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
          //PRINTREG16(mm1,"mmx1");
          // P_temp is reg 1
          paddw_r2r (mm1,mm4);
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
          // Store P
          movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[W].m)[d]);
        }
        else {
          if (y<0) {
            // Equivalent C function
            //Q[x+W][d] = 0;
            //for (j=0;j<W;j++) {
            //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
            //  Q[x+W][d] += P[x+W][j][d];
            //}
            
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
              //PRINTREG16(mm1,"mmx1");
              movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            
          }  
          else {
            // Equivalent C function
            // P is [N][W][D] the W dimension is a wrap around buffer.
            //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
            //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
            //P[x+W][y%W][d] = P_temp;
            
            // reg 4 = Q
            //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
            movq_m2r (*pQWd,mm4);

            // subtract prev P
            //psubw_m2r (((MM_U_16 *)P[y%W]->p[x+W].m)[d],mm4);
            psubw_m2r (*pPWd,mm4);

            // Add curr P to Q
            // lbw in 0
            //I1_I2D(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
            I1_I2Dptr(pI1, pI2);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            
            // Store Q
            //movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            movq_r2m (mm4,*pQWd);

            // Store P
            //movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[x+W].m)[d]);
            movq_r2m (mm1,*pPWd);

          }
          // Equivalent C function
          //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
          
          // reg 5 = C
          //movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);  
          movq_m2r (*pCd,mm5);

          
          // Sub curr Q[x][d] from C
          //psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
          psubusb_m2r (*pQd,mm5);

          //PRINTREG16(mm5,"C-P");
          
          // Add Q[x+W][d] to C
          //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
          movq_m2r (*pQWd,mm4);
          paddw_r2r (mm4,mm5);
          //PRINTREG16(mm5,"C-P+P");
          
          // Store C[x+1][d]
          //movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
          movq_r2m (mm5,*pC1d);
         
           
          pPWd += 4;
          pQd += 4;
          pQWd += 4;
          pCd += 4;
          pC1d += 4;
          pI2 += 4;
          
        }

        // ---------------------------------------------
        // find minimum C Left->Right for C[x+1][d]      
        // ---------------------------------------------
        // first create & store 0x0001000100010001 in 6
        // for future reference
        pxor_r2r(mm6,mm6);
        pcmpeqw_r2r(mm1,mm1);
        psubw_r2r(mm1,mm6);
        

        // reg 5 = C 
        //movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
        movq_r2r(mm5,mm0);
        
        //reg 4 = minimum C
        movq_m2r(min_lr,mm4);
        movq_r2r(mm4,mm2);
        
        //fill 1 with 0x80008000s (sign bit)
        movq_r2r(mm6,mm1);
        psllw_i2r(15,mm1);

        paddw_r2r(mm1,mm2);
        paddw_r2r(mm1,mm0);
        //PRINTREG16(mm2,"min_lr");
        //PRINTREG16(mm0,"C");
        
        // reg 2 becomes mask
        pcmpgtw_r2r(mm0,mm2);
        movq_r2r(mm2,mm1);
        
        //PRINTREG16(mm2,"mask");
        
        // reg 0 is C values smaller than minimum
        movq_r2r(mm5,mm0);
        pand_r2r(mm2,mm0);
        
        // reg 4 is preserved minimum
        pandn_r2r(mm4,mm1);
        
        // reg 1 is new minimum
        por_r2r(mm0,mm1);
        movq_r2m(mm1,min_lr);
        //PRINTREG16(mm6,"new_min_lr");
        
        // reg 4 is d values for minimum
        movq_m2r(min_d_lr,mm4);
        // reg 1 is preserved d values
        movq_r2r(mm2,mm1);
        pandn_r2r(mm4,mm1);
        
        // reg 3 = current d value
        // increase current d values by 4
        movq_m2r(curr_d_lr,mm3);
        movq_r2r(mm3,mm0);
        

        // reg 0 = current d value for updated minimum
        pand_r2r(mm2,mm0);
        
        // reg 1 = new d's for minimum values
        por_r2r(mm0,mm1);
        movq_r2m(mm1,min_d_lr);
        //PRINTREG16(mm1,"new_min_d_lr");

        // increase current d's
        // fill with 0x0004000400040004 
        psllw_i2r(2,mm6);
        paddw_r2r(mm6,mm3);
  
        //paddw_m2r(plus_four,mm3);
        movq_r2m(mm3,curr_d_lr);
  
        // ---------------------------------------------
        // find minimum C Right->Left for C[x][d]      
        // ---------------------------------------------
        
  
      }// for d

      
      // first create & store 0x0001000100010001 in 6
      // for future reference
      //pxor_r2r(mm6,mm6);
      //pcmpeqw_r2r(mm0,mm0);
      //psubw_r2r(mm0,mm6);

      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]
      // --------------------------------------------
      
      movq_m2r(min_lr,mm1);
      movq_m2r(min_d_lr,mm2);
      //movq_r2r(mm1,mm3);
      //movq_r2r(mm2,mm4);

      //fill 0 with 0x80008000s (sign bit)
      //movq_r2r(mm6,mm0);
      psllw_i2r(13,mm6);
      // remove sign
      paddw_r2r(mm6,mm3);

      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm3,mm0);
      pcmpeqw_r2r(mm1,mm0);

      pand_r2r(mm0,mm2);
      pandn_r2r(mm4,mm0);
      por_r2r(mm0,mm2);

      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm3,mm0);
      pcmpeqw_r2r(mm1,mm0);

      pand_r2r(mm0,mm2);
      pandn_r2r(mm4,mm0);
      por_r2r(mm0,mm2);

      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm3,mm0);
      pcmpeqw_r2r(mm1,mm0);

      pand_r2r(mm0,mm2);
      pandn_r2r(mm4,mm0);
      por_r2r(mm0,mm2);

      movq_r2m(mm3,min_lr);
      movq_r2m(mm2,min_d_lr);
      //PRINTREG8(mm3,"min_lr:");
      //PRINTREG8(mm1,"orig_min_lr:");
      //PRINTREG8(mm2,"min_d_lr:");
      
      overall_min_lr = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = (overall_min_d_lr <<2);
      

      
    }// for x
  }// for y

  emms();
  return 0;
}


//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD3(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  const int W = 16;
  //const int W = 8;


  int Y,X;

  register int i, j;
  register int d,x,y;
  
  int w_2 = (W/2);
  int d_w_2_1 = D_OFF+w_2+1;

  int d_shft = (int)((double) log(D)/log(2));
  int w_mod = W-1;
  unsigned short overall_min_lr;


  unsigned char overall_min_d_lr;

  unsigned char *pI1=NULL, *pI2=NULL;
  unsigned short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;

  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C1;
  MMXMatrix *C2;
  MMXMatrix *C;
  MMXMatrix *C_prev;
  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  
  X = I1->cols;
  Y = I1->rows;


  if (*cache==0) {
    i = 0;
    *cache = MMXMatricesCacheAlloc(3+W,0,0);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, X<<d_shft, 1);
    (*cache)->M[i++] = C1 = MMXMatrixAlloc(MMT_U_16, X<<d_shft, 1);
    (*cache)->M[i++] = C2 = MMXMatrixAlloc(MMT_U_16, X<<d_shft, 1);
    P = (*cache)->M+3;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, X<<d_shft, 1);
  }
  else { 
    P = (*cache)->M+3;
    Q = (*cache)->M[0];
    C1 = (*cache)->M[1];
    C2 = (*cache)->M[2];
  }


  emms();

  C = C2;
  C_prev = C1;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }
    
    for(x=-1;x<X-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      
      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);
      
      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;
      
      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4); 
      
      pxor_r2r (mm7,mm7); 

      if (x>=0) {
        pCd = &(((MM_U_16 *)C->p[0].m)[x<<d_shft]);
        pC1d = &(((MM_U_16 *)C->p[0].m)[(x+1)<<d_shft]);
        pQd  = &(((MM_U_16 *)Q->p[0].m)[x<<d_shft]);
        pQWd  = &(((MM_U_16 *)Q->p[0].m)[(x+W)<<d_shft]);
        if (y>=0) {
          pPWd = &(((MM_U_16 *)P[y&w_mod]->p[0].m)[(x+W)<<d_shft]);
        }
        pI1  = &(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF]);
        pI2  = &(((MM_U_8 *)I2->p[y+W].m)[x+W]);
      }

      
      for (d=0;d<D-3;d+=4) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {      
          // Equivalent C function
          //C[d][0]=0; 
          //for(i=0;i<=W;i+=4) {
          //  Q[d][i] = 0;
          // window down image
          //  for (j=0;j<W;j++) {
          //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
          //  Q[d][i] += P[j][d][i];
          //  } 
          //  if (i<W)
          //    C[d][0] += Q[d][i];
          //}
    
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2D(I1, I2, i+D_OFF,j,i+d,j);
              //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
              //printf("P(%d,%d,%d):",i,j,d);
              //PRINTREG16(mm1,"P");
        
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[0].m)[(i<<d_shft)+d]);
            //printf("Q(%d,%d):",i,d);
            // PRINTREG16(mm4,"Q");
    
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          
          // compute for i=W
          // reg 4 = Q
          pxor_r2r (mm4,mm4);  
          // window down image
          for (j=0;j<W;j++) {
            // lbw in 0
            I1_I2D(I1, I2, W+D_OFF,j,W+d,j);
            
            movq_r2m (mm1,((MM_U_16 *)P[j]->p[0].m)[(W<<d_shft)+d]);
            // Add curr P to Q
            paddw_r2r (mm1,mm4);
          }
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[0].m)[(W<<d_shft)+d]);
         
    
        }
        else if (x<0) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i++) {
          // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
          // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
          // P[y%W][d][i] = P_temp;
          // if (i<W)
          //   C[d][0] += Q[d][i];
          //}
          
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            movq_m2r (((MM_U_16 *)Q->p[0].m)[(i<<d_shft)+d],mm4);  
            // Sub prev P from Q
            psubw_m2r (((MM_U_16 *)P[y&w_mod]->p[0].m)[(i<<d_shft)+d],mm4);
            // Add curr P to Q
            // lbw in 0
            I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[0].m)[(i<<d_shft)+d]);
            // Store P
            movq_r2m (mm1,((MM_U_16 *)P[y&w_mod]->p[0].m)[(i<<d_shft)+d]);
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C[d][0]
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          //PRINTREG16(mm5,"C");
          
          
          // Do for i=W
          // reg 4 = Q
          movq_m2r (((MM_U_16 *)Q->p[0].m)[(W<<d_shft)+d],mm4);  
          // Sub prev P from Q
          psubw_m2r (((MM_U_16 *)P[y&w_mod]->p[0].m)[(W<<d_shft)+d],mm4);
          // Add curr P to Q
          // lbw in 0
          I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
          //PRINTREG16(mm1,"mmx1");
          // P_temp is reg 1
          paddw_r2r (mm1,mm4);
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[0].m)[(W<<d_shft)+d]);
          // Store P
          movq_r2m (mm1,((MM_U_16 *)P[y&w_mod]->p[0].m)[(W<<d_shft)+d]);
        }
        else {
          if (y<0) {
            // Equivalent C function
            //Q[x+W][d] = 0;
            //for (j=0;j<W;j++) {
            //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
            //  Q[x+W][d] += P[x+W][j][d];
            //}
            
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
              //PRINTREG16(mm1,"mmx1");
              movq_r2m (mm1,((MM_U_16 *)P[j]->p[0].m)[((x+W)<<d_shft)+d]);
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[0].m)[((x+W)<<d_shft)+d]);
            
          }  
          else {
            // Equivalent C function
            // P is [N][W][D] the W dimension is a wrap around buffer.
            //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
            //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
            //P[x+W][y%W][d] = P_temp;
            
            // reg 4 = Q
            //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
            movq_m2r (*pQWd,mm4);

            // subtract prev P
            //psubw_m2r (((MM_U_16 *)P[y%W]->p[x+W].m)[d],mm4);
            psubw_m2r (*pPWd,mm4);

            // Add curr P to Q
            // lbw in 0
            //I1_I2D(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
            I1_I2Dptr(pI1, pI2);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            
            // Store Q
            //movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            movq_r2m (mm4,*pQWd);

            // Store P
            //movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[x+W].m)[d]);
            movq_r2m (mm1,*pPWd);

          }
          // Equivalent C function
          //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
          
          // reg 5 = C
          //movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);  
          movq_m2r (*pCd,mm5);

          
          // Sub curr Q[x][d] from C
          //psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
          psubw_m2r (*pQd,mm5);

          //PRINTREG16(mm5,"C-P");
          
          // Add Q[x+W][d] to C
          //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
          //movq_m2r (*pQWd,mm4);
          //paddw_r2r (mm4,mm5);
          paddw_m2r (*pQWd,mm5);
          //PRINTREG16(mm5,"C-P+P");
          
          // Store C[x+1][d]
          //movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
          movq_r2m (mm5,*pC1d);
         
           
          pPWd += 4;
          pQd += 4;
          pQWd += 4;
          pCd += 4;
          pC1d += 4;
          pI2 += 4;
          
        }


        // ---------------------------------------------
        // find minimum C Left->Right for C[x+1][d]      
        // ---------------------------------------------
        // first create & store 0x0001000100010001 in 6
        // for future reference
        pxor_r2r(mm6,mm6);
        pcmpeqw_r2r(mm1,mm1);
        psubw_r2r(mm1,mm6);
        

        // reg 5 = C 
        //movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
        movq_r2r(mm5,mm2);
        
        //reg 4 = minimum C
        movq_m2r(min_lr,mm4);
        movq_r2r(mm4,mm0);
        
        //fill 1 with 0x80008000s (sign bit)
        movq_r2r(mm6,mm1);
        psllw_i2r(15,mm1);

        paddw_r2r(mm1,mm0);
        paddw_r2r(mm1,mm2);
        //PRINTREG16(mm0,"min_lr");
        //PRINTREG16(mm2,"C");
        
        // reg 0 becomes mask
        pcmpgtw_r2r(mm2,mm0);
        movq_r2r(mm0,mm1);
        
        
        // reg 0 is C values smaller than minimum
        movq_r2r(mm5,mm2);
        pand_r2r(mm0,mm2);
        
        // reg 4 is preserved minimum
        pandn_r2r(mm4,mm1);
        
        // reg 1 is new minimum
        por_r2r(mm2,mm1);
        movq_r2m(mm1,min_lr);
        
        // reg 4 is d values for minimum
        movq_m2r(min_d_lr,mm4);
        // reg 1 is preserved d values
        movq_r2r(mm0,mm5);
        pandn_r2r(mm4,mm5);
        
        // reg 3 = current d value
        // increase current d values by 4
        movq_m2r(curr_d_lr,mm3);
        movq_r2r(mm3,mm2);
        

        // reg 0 = current d value for updated minimum
        pand_r2r(mm0,mm2);
        
        // reg 1 = new d's for minimum values
        por_r2r(mm5,mm2);
        movq_r2m(mm2,min_d_lr);

        // increase current d's
        // fill with 0x0004000400040004 
        psllw_i2r(2,mm6);
        paddw_r2r(mm6,mm3);
  
        //paddw_m2r(plus_four,mm3);
        movq_r2m(mm3,curr_d_lr);        
  
      }// for d
      
      // first create & store 0x0001000100010001 in 6
      // for future reference
      //pxor_r2r(mm6,mm6);
      //pcmpeqw_r2r(mm0,mm0);
      //psubw_r2r(mm0,mm6);
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]
      // --------------------------------------------
      
      //movq_m2r(min_lr,mm1);
      //movq_m2r(min_d_lr,mm2);

      //fill 0 with 0x80008000s (sign bit)
      //movq_r2r(mm6,mm0);
      psllw_i2r(13,mm6);
      //PRINTREG16(mm6,"0x80008000");

      // remove sign
      paddw_r2r(mm6,mm1);

      movq_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);

      // once
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // twice
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);


      // three times
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);


      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);
 
      //movq_m2r(min_d_lr,mm0);

      movq_r2m(mm3,min_lr);
      movq_r2m(mm4,min_d_lr);
      //PRINTREG16(mm1,"min_lr");
      //PRINTREG16(mm3,"overall_min_lr");
      //PRINTREG16(mm0,"min_d_lr");
      //PRINTREG16(mm4,"overall_min_d_lr");
      
      overall_min_lr = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = (overall_min_d_lr <<2);
      
      
#if 0

                               
      if (!(y<0)) {
  //d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1]-0x40)>>2;
  d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1])>>2;
  //d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1])>>3;
  //d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1]);
  min_rl = ((MM_U_16 *)C_prev->p[x+1].m)[d_rl];

  //printf("min&d_rl:%u,%u\n",min_rl,d_rl);
  idx_rl = x+1+d_rl;
  for (d=0;d<D;d++) {     
    if ((idx_rl>-1)&&(idx_rl<X)&&(min_rl > ((MM_U_16 *)C_prev->p[idx_rl].m)[d])) {
      //printf("better&d_rl:%u,%d\n",((MM_U_16 *)C_prev->p[idx_rl].m)[d],d);
      ((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1] = 0x00;
      //((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1] = ((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1-1];
     // break;
    }
    idx_rl--;
        }
      }
#endif
      
    }// for x
  }// for y

  emms();
  return 0;
}


//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  const int W = 16;
  //const int W = 8;


  int Y,X;

  register int i, j;
  register int d,x,y;
  
  int w_2 = (W/2);
  int d_w_2_1 = D_OFF+w_2+1;

  unsigned short overall_min_lr;


  int overall_min_d_lr;

  unsigned char *pI1=NULL, *pI2=NULL;
  unsigned short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;

  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C1;
  MMXMatrix *C2;
  MMXMatrix *C;
  MMXMatrix *C_prev;
  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  
  X = I1->cols;
  Y = I1->rows;

  if (*cache==0) {
    i=0;
    *cache = MMXMatricesCacheAlloc(3+W,0,0);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C1 = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C2 = MMXMatrixAlloc(MMT_U_16, D, X);
    P = (*cache)->M + 3;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, D, X);

  }
  else { 
    P = (*cache)->M + 3;
    Q = (*cache)->M[0]; 
    C1 = (*cache)->M[1]; 
    C2 = (*cache)->M[2]; 
  }


  emms();

  C = C2;
  C_prev = C1;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }
    
    for(x=-1;x<X-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      
      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);
      
      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;
      
      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4); 
      
      pxor_r2r (mm7,mm7); 

      if (x>=0) {
        pCd = &(((MM_U_16 *)C->p[x].m)[0]);
        pC1d = &(((MM_U_16 *)C->p[x+1].m)[0]);
        pQd  = &(((MM_U_16 *)Q->p[x].m)[0]);
        pQWd  = &(((MM_U_16 *)Q->p[x+W].m)[0]);
        if (y>=0) {
          pPWd = &(((MM_U_16 *)P[y%W]->p[x+W].m)[0]);
        }
        pI1  = &(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF]);
        pI2  = &(((MM_U_8 *)I2->p[y+W].m)[x+W]);
      }

      
      for (d=0;d<D-3;d+=4) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {      
          // Equivalent C function
          //C[d][0]=0; 
          //for(i=0;i<=W;i+=4) {
          //  Q[d][i] = 0;
          // window down image
          //  for (j=0;j<W;j++) {
          //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
          //  Q[d][i] += P[j][d][i];
          //  } 
          //  if (i<W)
          //    C[d][0] += Q[d][i];
          //}
    
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2D(I1, I2, i+D_OFF,j,i+d,j);
              //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
              //printf("P(%d,%d,%d):",i,j,d);
              //PRINTREG16(mm1,"P");
        
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
            //printf("Q(%d,%d):",i,d);
            // PRINTREG16(mm4,"Q");
    
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          
          // compute for i=W
          // reg 4 = Q
          pxor_r2r (mm4,mm4);  
          // window down image
          for (j=0;j<W;j++) {
            // lbw in 0
            I1_I2D(I1, I2, W+D_OFF,j,W+d,j);
            
            movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
            // Add curr P to Q
            paddw_r2r (mm1,mm4);
          }
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
         
    
        }
        else if (x<0) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i++) {
          // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
          // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
          // P[y%W][d][i] = P_temp;
          // if (i<W)
          //   C[d][0] += Q[d][i];
          //}
          
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            movq_m2r (((MM_U_16 *)Q->p[i].m)[d],mm4);  
            // Sub prev P from Q
            psubw_m2r (((MM_U_16 *)P[y%W]->p[i].m)[d],mm4);
            // Add curr P to Q
            // lbw in 0
            I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
            // Store P
            movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[i].m)[d]);
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C[d][0]
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          //PRINTREG16(mm5,"C");
          
          
          // Do for i=W
          // reg 4 = Q
          movq_m2r (((MM_U_16 *)Q->p[W].m)[d],mm4);  
          // Sub prev P from Q
          psubw_m2r (((MM_U_16 *)P[y%W]->p[W].m)[d],mm4);
          // Add curr P to Q
          // lbw in 0
          I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
          //PRINTREG16(mm1,"mmx1");
          // P_temp is reg 1
          paddw_r2r (mm1,mm4);
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
          // Store P
          movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[W].m)[d]);
        }
        else {
          if (y<0) {
            // Equivalent C function
            //Q[x+W][d] = 0;
            //for (j=0;j<W;j++) {
            //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
            //  Q[x+W][d] += P[x+W][j][d];
            //}
            
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
              //PRINTREG16(mm1,"mmx1");
              movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            
          }  
          else {
            // Equivalent C function
            // P is [N][W][D] the W dimension is a wrap around buffer.
            //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
            //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
            //P[x+W][y%W][d] = P_temp;
            
            // reg 4 = Q
            //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
            movq_m2r (*pQWd,mm4);

            // subtract prev P
            //psubw_m2r (((MM_U_16 *)P[y%W]->p[x+W].m)[d],mm4);
            psubw_m2r (*pPWd,mm4);

            // Add curr P to Q
            // lbw in 0
            //I1_I2D(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
            I1_I2Dptr(pI1, pI2);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            
            // Store Q
            //movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            movq_r2m (mm4,*pQWd);

            // Store P
            //movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[x+W].m)[d]);
            movq_r2m (mm1,*pPWd);

          }
          // Equivalent C function
          //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
          
          // reg 5 = C
          //movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);  
          movq_m2r (*pCd,mm5);

          
          // Sub curr Q[x][d] from C
          //psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
          psubw_m2r (*pQd,mm5);

          //PRINTREG16(mm5,"C-P");
          
          // Add Q[x+W][d] to C
          //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
          //movq_m2r (*pQWd,mm4);
          //paddw_r2r (mm4,mm5);
          paddw_m2r (*pQWd,mm5);
          //PRINTREG16(mm5,"C-P+P");
          
          // Store C[x+1][d]
          //movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
          movq_r2m (mm5,*pC1d);
         
           
          pPWd += 4;
          pQd += 4;
          pQWd += 4;
          pCd += 4;
          pC1d += 4;
          pI2 += 4;
          
        }


        // ---------------------------------------------
        // find minimum C Left->Right for C[x+1][d]      
        // ---------------------------------------------
        // first create & store 0x0001000100010001 in 6
        // for future reference
        pxor_r2r(mm6,mm6);
        pcmpeqw_r2r(mm1,mm1);
        psubw_r2r(mm1,mm6);
        

        // reg 5 = C 
        //movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
        movq_r2r(mm5,mm2);
        
        //reg 4 = minimum C
        movq_m2r(min_lr,mm4);
        movq_r2r(mm4,mm0);
        
        //fill 1 with 0x80008000s (sign bit)
        movq_r2r(mm6,mm1);
        psllw_i2r(15,mm1);

        paddw_r2r(mm1,mm0);
        paddw_r2r(mm1,mm2);
        //PRINTREG16(mm0,"min_lr");
        //PRINTREG16(mm2,"C");
        
        // reg 0 becomes mask
        pcmpgtw_r2r(mm2,mm0);
        movq_r2r(mm0,mm1);
        
        
        // reg 0 is C values smaller than minimum
        movq_r2r(mm5,mm2);
        pand_r2r(mm0,mm2);
        
        // reg 4 is preserved minimum
        pandn_r2r(mm4,mm1);
        
        // reg 1 is new minimum
        por_r2r(mm2,mm1);
        movq_r2m(mm1,min_lr);
        
        // reg 4 is d values for minimum
        movq_m2r(min_d_lr,mm4);
        // reg 1 is preserved d values
        movq_r2r(mm0,mm5);
        pandn_r2r(mm4,mm5);
        
        // reg 3 = current d value
        // increase current d values by 4
        movq_m2r(curr_d_lr,mm3);
        movq_r2r(mm3,mm2);
        

        // reg 0 = current d value for updated minimum
        pand_r2r(mm0,mm2);
        
        // reg 1 = new d's for minimum values
        por_r2r(mm5,mm2);
        movq_r2m(mm2,min_d_lr);

        // increase current d's
        // fill with 0x0004000400040004 
        psllw_i2r(2,mm6);
        paddw_r2r(mm6,mm3);
  
        //paddw_m2r(plus_four,mm3);
        movq_r2m(mm3,curr_d_lr);        
  
      }// for d
      
      // first create & store 0x0001000100010001 in 6
      // for future reference
      //pxor_r2r(mm6,mm6);
      //pcmpeqw_r2r(mm0,mm0);
      //psubw_r2r(mm0,mm6);
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]
      // --------------------------------------------
      
      //movq_m2r(min_lr,mm1);
      //movq_m2r(min_d_lr,mm2);

      //fill 0 with 0x80008000s (sign bit)
      //movq_r2r(mm6,mm0);
      psllw_i2r(13,mm6);
      //PRINTREG16(mm6,"0x80008000");

      // remove sign
      paddw_r2r(mm6,mm1);

      movq_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);

      // once
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // twice
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);


      // three times
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);


      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);
 
      //movq_m2r(min_d_lr,mm0);

      movq_r2m(mm3,min_lr);
      movq_r2m(mm4,min_d_lr);
      //PRINTREG16(mm1,"min_lr");
      //PRINTREG16(mm3,"overall_min_lr");
      //PRINTREG16(mm0,"min_d_lr");
      //PRINTREG16(mm4,"overall_min_d_lr");
      
      overall_min_lr = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = (overall_min_d_lr <<2);
      
      
#if 0

      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]      
      // ---------------------------------------------
            

      overall_min_lr = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      if (overall_min_lr > min_lr.uw[1]) {
        overall_min_lr = min_lr.uw[1];
        overall_min_d_lr = min_d_lr.uw[1];
      }
      if (overall_min_lr > min_lr.uw[2]) {
        overall_min_lr = min_lr.uw[2];
        overall_min_d_lr = min_d_lr.uw[2];
      }
      if (overall_min_lr > min_lr.uw[3]) {
        overall_min_lr = min_lr.uw[3];
        overall_min_d_lr = min_d_lr.uw[3];
      }
      //((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = 0x40+(overall_min_d_lr <<2);
      //((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = (overall_min_d_lr <<3);
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = (overall_min_d_lr <<2);
      //((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = overall_min_d_lr;
      // ---------------------------------------------
      // find overall minimum C Right->Left for C[x][d]      
      // check for consistency
      // ---------------------------------------------
                               
      if (!(y<0)) {
  //d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1]-0x40)>>2;
  d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1])>>2;
  //d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1])>>3;
  //d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1]);
  min_rl = ((MM_U_16 *)C_prev->p[x+1].m)[d_rl];

  //printf("min&d_rl:%u,%u\n",min_rl,d_rl);
  idx_rl = x+1+d_rl;
  for (d=0;d<D;d++) {     
    if ((idx_rl>-1)&&(idx_rl<X)&&(min_rl > ((MM_U_16 *)C_prev->p[idx_rl].m)[d])) {
      //printf("better&d_rl:%u,%d\n",((MM_U_16 *)C_prev->p[idx_rl].m)[d],d);
      ((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1] = 0x00;
      //((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1] = ((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1-1];
     // break;
    }
    idx_rl--;
        }
      }
#endif
      
    }// for x
  }// for y

  emms();
  return 0;
}









//#define MIN_THRESH 4096

// - for parallel cameras. offset introduced to give synthetic vergence
//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD4(const MMXMatrix *I1,const  MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  //const int W = 16;
  //const int W = 12;
  //const int W = 8;
  //const int V = 4;
  const int W = 8;
  const int V = 4;

  //const int W = 4;

  //const int shift = (D>32)?2:3;
  const int shift = (D<=8)?5:(D<=16)?4:(D<=32)?3:2;

  int Y,X;

  register int i, j;
  register int d,x,y;
  
  int w_2 = (W/2);
  int v_2 = (V/2);
  //int d_w_2_1 = D_OFF+w_2+1;

  unsigned short overall_min_lr;

  int idx_rl; 
  unsigned short min_rl;

  int overall_min_d_lr;
  int min_d_rl;

  int subpix_d,subpix_denom;

  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C;
  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  unsigned char *pI1=NULL, *pI2=NULL;
  unsigned short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;
  
  X = I1->cols;
  Y = I1->rows;

  if (*cache==0) {
    i = 0;
    *cache = MMXMatricesCacheAlloc(2+V,0,0);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C = MMXMatrixAlloc(MMT_U_16, D, X);
    P = (*cache)->M + 2;
    for (i=0;i<V;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, D, X);
    
  }
  else { 
    P = (*cache)->M + 2;
    Q = (*cache)->M[0]; 
    C = (*cache)->M[1]; 
  }
  
  emms();




  for(y=-1;y<Y-V-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);

    
    for(x=-1;x<X-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      
      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);
      
      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;
      
      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4); 
      
      pxor_r2r (mm7,mm7); 

      if (!(x<0)) {
        pCd = &(((MM_U_16 *)C->p[x].m)[0]);
        pC1d = &(((MM_U_16 *)C->p[x+1].m)[0]);
        pQd  = &(((MM_U_16 *)Q->p[x].m)[0]);
        pQWd  = &(((MM_U_16 *)Q->p[x+W].m)[0]);
        if (!(y<0)) {
          pPWd = &(((MM_U_16 *)P[y%V]->p[x+W].m)[0]);
        }
        pI1  = &(((MM_U_8 *)I1->p[y+V].m)[x+W+D_OFF]);
        pI2  = &(((MM_U_8 *)I2->p[y+V].m)[x+W]);
      }

      
      for (d=0;d<D-3;d+=4) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {      
          // Equivalent C function
          //C[d][0]=0; 
          //for(i=0;i<=W;i+=4) {
          //  Q[d][i] = 0;
          // window down image
          //  for (j=0;j<V;j++) {
          //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
          //  Q[d][i] += P[j][d][i];
          //  } 
          //  if (i<W)
          //    C[d][0] += Q[d][i];
          //}
    
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<V;j++) {
              // lbw in 0
              I1_I2D(I1, I2, i+D_OFF,j,i+d,j);
              //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
              //printf("P(%d,%d,%d):",i,j,d);
              //PRINTREG16(mm1,"P");
        
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
            //printf("Q(%d,%d):",i,d);
            // PRINTREG16(mm4,"Q");
    
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          
          // compute for i=W
          // reg 4 = Q
          pxor_r2r (mm4,mm4);  
          // window down image
          for (j=0;j<V;j++) {
            // lbw in 0
            I1_I2D(I1, I2, W+D_OFF,j,W+d,j);
            
            movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
            // Add curr P to Q
            paddw_r2r (mm1,mm4);
          }
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
         
        }
        else if (x<0) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i++) {
          // P_temp = I1_I2(I1, I2, i+D_OFF,y+V,i+d,y+V);
          // Q[d][i] = Q[d][i] - P[y%V][d][i] + P_temp ;
          // P[y%V][d][i] = P_temp;
          // if (i<W)
          //   C[d][0] += Q[d][i];
          //}
          
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            movq_m2r (((MM_U_16 *)Q->p[i].m)[d],mm4);  
            // Sub prev P from Q
            psubw_m2r (((MM_U_16 *)P[y%V]->p[i].m)[d],mm4);
            // Add curr P to Q
            // lbw in 0
            I1_I2D(I1, I2, i+D_OFF,y+V,i+d,y+V);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
            // Store P
            movq_r2m (mm1,((MM_U_16 *)P[y%V]->p[i].m)[d]);
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C[d][0]
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          //PRINTREG16(mm5,"C");
          
          
          // Do for i=W
          // reg 4 = Q
          movq_m2r (((MM_U_16 *)Q->p[W].m)[d],mm4);  
          // Sub prev P from Q
          psubw_m2r (((MM_U_16 *)P[y%V]->p[W].m)[d],mm4);
          // Add curr P to Q
          // lbw in 0
          I1_I2D(I1, I2, W+D_OFF,y+V,W+d,y+V);
          //PRINTREG16(mm1,"mmx1");
          // P_temp is reg 1
          paddw_r2r (mm1,mm4);
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
          // Store P
          movq_r2m (mm1,((MM_U_16 *)P[y%V]->p[W].m)[d]);
        }
        else {
          if (y<0) {
            // Equivalent C function
            //Q[x+W][d] = 0;
            //for (j=0;j<V;j++) {
            //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
            //  Q[x+W][d] += P[x+W][j][d];
            //}
            
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<V;j++) {
              // lbw in 0
              I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
              //PRINTREG16(mm1,"mmx1");
              movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            
          }  
          else {
            // Equivalent C function
            // P is [N][V][D] the V dimension is a wrap around buffer.
            //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+V,x+W+d,y+V);
            //Q[x+W][d] = Q[x+W][d] - P[x+W][y%V][d] + P_temp ;
            //P[x+W][y%V][d] = P_temp;
            
            // reg 4 = Q
            //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
            movq_m2r (*pQWd,mm4);

            // subtract prev P
            //psubw_m2r (((MM_U_16 *)P[y%V]->p[x+W].m)[d],mm4);
            psubw_m2r (*pPWd,mm4);

            // Add curr P to Q
            // lbw in 0
            //I1_I2D(I1, I2,x+W+D_OFF,y+V,x+W+d,y+V);
            I1_I2Dptr(pI1, pI2);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            
            // Store Q
            //movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            movq_r2m (mm4,*pQWd);

            // Store P
            //movq_r2m (mm1,((MM_U_16 *)P[y%V]->p[x+W].m)[d]);
            movq_r2m (mm1,*pPWd);

          }
          // Equivalent C function
          //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
          
          // reg 5 = C
          //movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);  
          movq_m2r (*pCd,mm5);

          
          // Sub curr Q[x][d] from C
          //psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
          psubw_m2r (*pQd,mm5);

          //PRINTREG16(mm5,"C-P");
          
          // Add Q[x+W][d] to C
          //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
          //movq_m2r (*pQWd,mm4);
          //paddw_r2r (mm4,mm5);
          paddw_m2r (*pQWd,mm5);
          //PRINTREG16(mm5,"C-P+P");
          
          // Store C[x+1][d]
          //movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
          movq_r2m (mm5,*pC1d);
         
           
          pPWd += 4;
          pQd += 4;
          pQWd += 4;
          pCd += 4;
          pC1d += 4;
          pI2 += 4;
          
        }

        // ---------------------------------------------
        // find minimum C Left->Right for C[x+1][d]      
        // ---------------------------------------------
        // first create & store 0x0001000100010001 in 6
        // for future reference
        pxor_r2r(mm6,mm6);
        pcmpeqw_r2r(mm1,mm1);
        psubw_r2r(mm1,mm6);
        

        // reg 5 = C 
        //movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
        movq_r2r(mm5,mm2);
        
        //reg 4 = minimum C
        movq_m2r(min_lr,mm4);
        movq_r2r(mm4,mm0);
        
        //fill 1 with 0x80008000s (sign bit)
        movq_r2r(mm6,mm1);
        psllw_i2r(15,mm1);

        paddw_r2r(mm1,mm0);
        paddw_r2r(mm1,mm2);
        //PRINTREG16(mm0,"min_lr");
        //PRINTREG16(mm2,"C");
        
        // reg 0 becomes mask
        pcmpgtw_r2r(mm2,mm0);
        movq_r2r(mm0,mm1);
        
        //PRINTREG16(mm2,"mask");
        
        // reg 0 is C values smaller than minimum
        movq_r2r(mm5,mm2);
        pand_r2r(mm0,mm2);
        
        // reg 4 is preserved minimum
        pandn_r2r(mm4,mm1);
        
        // reg 1 is new minimum
        por_r2r(mm2,mm1);
        movq_r2m(mm1,min_lr);
        //PRINTREG16(mm6,"new_min_lr");
        
        // reg 4 is d values for minimum
        movq_m2r(min_d_lr,mm4);
        // reg 1 is preserved d values
        movq_r2r(mm0,mm5);
        pandn_r2r(mm4,mm5);
        
        // reg 3 = current d value
        // increase current d values by 4
        movq_m2r(curr_d_lr,mm3);
        movq_r2r(mm3,mm2);
        

        // reg 0 = current d value for updated minimum
        pand_r2r(mm0,mm2);
        
        // reg 1 = new d's for minimum values
        por_r2r(mm5,mm2);
        movq_r2m(mm2,min_d_lr);
        //PRINTREG16(mm1,"new_min_d_lr");

        // increase current d's
        // fill with 0x0004000400040004 
        psllw_i2r(2,mm6);
        paddw_r2r(mm6,mm3);
  
        //paddw_m2r(plus_four,mm3);
        movq_r2m(mm3,curr_d_lr);
  
        // ---------------------------------------------
        // find minimum C Right->Left for C[x][d]      
        // ---------------------------------------------
        
  
      }// for d
      
      // first create & store 0x0001000100010001 in 6
      // for future reference
      //pxor_r2r(mm6,mm6);
      //pcmpeqw_r2r(mm0,mm0);
      //psubw_r2r(mm0,mm6);
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]
      // --------------------------------------------
      
      //movq_m2r(min_lr,mm1);
      //movq_m2r(min_d_lr,mm2);

      //fill 0 with 0x80008000s (sign bit)
      //movq_r2r(mm6,mm0);
      psllw_i2r(13,mm6);
      //PRINTREG16(mm6,"0x80008000");

      // remove sign
      paddw_r2r(mm6,mm1);

      movq_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);

      // once
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // twice
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);


      // three times
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);


      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);
 
      //movq_m2r(min_d_lr,mm0);

      //movq_r2m(mm3,min_lr);
      movq_r2m(mm4,min_d_lr);
      //PRINTREG16(mm1,"min_lr");
      //PRINTREG16(mm3,"overall_min_lr");
      //PRINTREG16(mm0,"min_d_lr");
      //PRINTREG16(mm4,"overall_min_d_lr");
      
      overall_min_d_lr = min_d_lr.uw[0];
      overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr];

      ((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = overall_min_d_lr;
    }      

    for(x=-1;x<X-W-D-1;x++) {

      overall_min_d_lr = ((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1];
      overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr];      
      // ---------------------------------------------
      // find overall minimum C Right->Left for C[x][d]      
      // check for consistency
      // ---------------------------------------------                                     

      min_d_rl = overall_min_d_lr;
      min_rl = overall_min_lr;
      
      //printf("min&d_rl:%u,%u\n",min_rl,d_rl);
      idx_rl = x+1+min_d_rl;
      for (d=0;d<D;d++) {
        if ((idx_rl>-1)&&(idx_rl<X)&&(min_rl >= ((MM_U_16 *)C->p[idx_rl].m)[d])) {
          min_rl = ((MM_U_16 *)C->p[idx_rl].m)[d];
          min_d_rl = d;
          //break;
        } // if
        idx_rl--;
      } // for
      if (abs(min_d_rl - overall_min_d_lr)>0) {
        //((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = 0x00;
        //((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = 0x00;
        overall_min_d_lr=0x00;
      }
      

      // ---------------------------------------------
      // subpixel interpolation      
      // weighted average of best match and each neighbour
      // ---------------------------------------------
      if ((overall_min_d_lr>0)&&(overall_min_d_lr<D-1)) {
      
        subpix_denom = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1] - overall_min_lr - overall_min_lr +((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1];
        subpix_d = (((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1]- ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1])<<shift;

        //fprintf(stderr,"%d %d %d ",overall_min_d_lr,subpix_d,subpix_denom);
        if (subpix_denom) {
          subpix_d /= (subpix_denom);
          subpix_d += (overall_min_d_lr<<shift);
        }
        else
          subpix_d = 0;
        //fprintf(stderr," %d  %d\n ",subpix_d,subpix_d/8);
        
        ((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = subpix_d;
      
      }
      else 
        ((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = 0x00;


     }// for x
  }// for y
    
  emms();
  return 0;
}




#ifndef USE_SAD_LEANNE
int MMXRecursive_Depthmap_SAD4_leanne(const MMXMatrix *I1,const  MMXMatrix *I2, 
		MMXMatrix  *Out, const int D, 
		const int D_OFF, const int W, const int lr_threshold, 
		MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
	MMXMatrixAssert(I1);
	MMXMatrixAssert(I2);
	MMXMatrixAssert(Out);
	//const int W = 16;
	//const int W = 12;
	//const int W = 8;
	//const int V = 4;


	// parameter 
	//const int W = 8;
	const int V = 4;

	//const int W = 4;

	//const int shift = (D>32)?2:3;
	const int shift = (D<=8)?5:(D<=16)?4:(D<=32)?3:2;

	int Y,X;

	register int i, j;
	register int d,x,y;

	int w_2 = (W/2);
	int v_2 = (V/2);
	//int d_w_2_1 = D_OFF+w_2+1;

	unsigned short overall_min_lr;

	int idx_rl; 
	unsigned short min_rl;

	int overall_min_d_lr;
	int min_d_rl;

	int subpix_d,subpix_denom;

	MMXMatrix **P;
	MMXMatrix *Q;
	MMXMatrix *C;
	mmx_t min_lr;
	mmx_t curr_d_lr;
	mmx_t min_d_lr;

	unsigned char *pI1=NULL, *pI2=NULL;
	unsigned short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;

	X = I1->cols;
	Y = I1->rows;

	if (*cache==0) {
          i = 0;
          *cache = MMXMatricesCacheAlloc(2+V,0,0);
          (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
          (*cache)->M[i++] = C = MMXMatrixAlloc(MMT_U_16, D, X);
          P = (*cache)->M + 2;
          for (i=0;i<V;i++)
            P[i] = MMXMatrixAlloc(MMT_U_16, D, X);
	}
	else { 
		P = (*cache)->M + 2;
		Q = (*cache)->M[0]; 
		C = (*cache)->M[1]; 
	}

	emms();



	MMXMatrixAssert(Out);

	for(y=-1;y<Y-V-1;y++){
		//fprintf(stderr,"depthmap: y:%d \n",y);


		for(x=-1;x<X-W-D-1;x++) {
			//fprintf(stderr,"depthmap: x:%d \n",x);


			// fill min with 111111111111111s (max number)
			pcmpeqw_r2r(mm0,mm0);
			movq_r2m(mm0,min_lr);

			curr_d_lr.ud[0] = 0x00010000;
			curr_d_lr.ud[1] = 0x00030002;

			// clear registers
			//pxor_r2r (mm3,mm3);
			// pxor_r2r (mm4,mm4); 

			pxor_r2r (mm7,mm7); 

			if (!(x<0)) {
				pCd = &(((MM_U_16 *)C->p[x].m)[0]);
				pC1d = &(((MM_U_16 *)C->p[x+1].m)[0]);
				pQd  = &(((MM_U_16 *)Q->p[x].m)[0]);
				pQWd  = &(((MM_U_16 *)Q->p[x+W].m)[0]);
				if (!(y<0)) {
					pPWd = &(((MM_U_16 *)P[y%V]->p[x+W].m)[0]);
				}
				pI1  = &(((MM_U_8 *)I1->p[y+V].m)[x+W+D_OFF]);
				pI2  = &(((MM_U_8 *)I2->p[y+V].m)[x+W]);
			}


			for (d=0;d<D-3;d+=4) {
				//fprintf(stderr,"depthmap: d:%d \n",d);
				if ((x<0)&&(y<0)) {      
					// Equivalent C function
					//C[d][0]=0; 
					//for(i=0;i<=W;i+=4) {
					//  Q[d][i] = 0;
					// window down image
					//  for (j=0;j<V;j++) {
					//  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
					//  Q[d][i] += P[j][d][i];
					//  } 
					//  if (i<W)
					//    C[d][0] += Q[d][i];
					//}

					// reg 5 = C
					pxor_r2r (mm5,mm5);
					for(i=0;i<W;i++) {
						// reg 4 = Q
						pxor_r2r (mm4,mm4);

						// window down image
						for (j=0;j<V;j++) {
							// lbw in 0
							I1_I2D(I1, I2, i+D_OFF,j,i+d,j);
							//printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
							//printf("P(%d,%d,%d):",i,j,d);
							//PRINTREG16(mm1,"P");

							// Add curr P to Q
							paddw_r2r (mm1,mm4);
						}
						// Store Q
						movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
						//printf("Q(%d,%d):",i,d);
						// PRINTREG16(mm4,"Q");

						// Add curr Q to C
						paddw_r2r (mm4,mm5);
					}
					// Store C
					movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);

					// compute for i=W
					// reg 4 = Q
					pxor_r2r (mm4,mm4);  
					// window down image
					for (j=0;j<V;j++) {
						// lbw in 0
						I1_I2D(I1, I2, W+D_OFF,j,W+d,j);

						movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
						// Add curr P to Q
						paddw_r2r (mm1,mm4);
					}
					// Store Q
					movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);

				}
				else if (x<0) {
					// Equivalent C function
					//C[d][0]=0;
					//for(i=0;i<=W;i++) {
					// P_temp = I1_I2(I1, I2, i+D_OFF,y+V,i+d,y+V);
					// Q[d][i] = Q[d][i] - P[y%V][d][i] + P_temp ;
					// P[y%V][d][i] = P_temp;
					// if (i<W)
					//   C[d][0] += Q[d][i];
					//}

					// reg 5 = C
					pxor_r2r (mm5,mm5);
					for(i=0;i<W;i++) {
						// reg 4 = Q
						movq_m2r (((MM_U_16 *)Q->p[i].m)[d],mm4);  
						// Sub prev P from Q
						psubw_m2r (((MM_U_16 *)P[y%V]->p[i].m)[d],mm4);
						// Add curr P to Q
						// lbw in 0
						I1_I2D(I1, I2, i+D_OFF,y+V,i+d,y+V);
						//PRINTREG16(mm1,"mmx1");
						// P_temp is reg 1
						paddw_r2r (mm1,mm4);
						// Store Q
						movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
						// Store P
						movq_r2m (mm1,((MM_U_16 *)P[y%V]->p[i].m)[d]);
						// Add curr Q to C
						paddw_r2r (mm4,mm5);
					}
					// Store C[d][0]
					movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
					//PRINTREG16(mm5,"C");


					// Do for i=W
					// reg 4 = Q
					movq_m2r (((MM_U_16 *)Q->p[W].m)[d],mm4);  
					// Sub prev P from Q
					psubw_m2r (((MM_U_16 *)P[y%V]->p[W].m)[d],mm4);
					// Add curr P to Q
					// lbw in 0
					I1_I2D(I1, I2, W+D_OFF,y+V,W+d,y+V);
					//PRINTREG16(mm1,"mmx1");
					// P_temp is reg 1
					paddw_r2r (mm1,mm4);
					// Store Q
					movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
					// Store P
					movq_r2m (mm1,((MM_U_16 *)P[y%V]->p[W].m)[d]);
				}
				else {
					if (y<0) {
						// Equivalent C function
						//Q[x+W][d] = 0;
						//for (j=0;j<V;j++) {
						//  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
						//  Q[x+W][d] += P[x+W][j][d];
						//}

						// reg 4 = Q
						pxor_r2r (mm4,mm4);

						// window down image
						for (j=0;j<V;j++) {
							// lbw in 0
							I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
							//PRINTREG16(mm1,"mmx1");
							movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
							// Add curr P to Q
							paddw_r2r (mm1,mm4);
						}
						// Store Q
						movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);

					}  
					else {
						// Equivalent C function
						// P is [N][V][D] the V dimension is a wrap around buffer.
						//P_temp = I1_I2(I1, I2, x+W+D_OFF,y+V,x+W+d,y+V);
						//Q[x+W][d] = Q[x+W][d] - P[x+W][y%V][d] + P_temp ;
						//P[x+W][y%V][d] = P_temp;

						// reg 4 = Q
						//movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
						movq_m2r (*pQWd,mm4);

						// subtract prev P
						//psubw_m2r (((MM_U_16 *)P[y%V]->p[x+W].m)[d],mm4);
						psubw_m2r (*pPWd,mm4);

						// Add curr P to Q
						// lbw in 0
						//I1_I2D(I1, I2,x+W+D_OFF,y+V,x+W+d,y+V);
						I1_I2Dptr(pI1, pI2);
						//PRINTREG16(mm1,"mmx1");
						// P_temp is reg 1
						paddw_r2r (mm1,mm4);

						// Store Q
						//movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
						movq_r2m (mm4,*pQWd);

						// Store P
						//movq_r2m (mm1,((MM_U_16 *)P[y%V]->p[x+W].m)[d]);
						movq_r2m (mm1,*pPWd);

					}
					// Equivalent C function
					//C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];

					// reg 5 = C
					//movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);  
					movq_m2r (*pCd,mm5);


					// Sub curr Q[x][d] from C
					//psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
					psubw_m2r (*pQd,mm5);

					//PRINTREG16(mm5,"C-P");

					// Add Q[x+W][d] to C
					//movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
					//movq_m2r (*pQWd,mm4);
					//paddw_r2r (mm4,mm5);
					paddw_m2r (*pQWd,mm5);
					//PRINTREG16(mm5,"C-P+P");

					// Store C[x+1][d]
					//movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
					movq_r2m (mm5,*pC1d);


					pPWd += 4;
					pQd += 4;
					pQWd += 4;
					pCd += 4;
					pC1d += 4;
					pI2 += 4;

				}

				// ---------------------------------------------
				// find minimum C Left->Right for C[x+1][d]      
				// ---------------------------------------------
				// first create & store 0x0001000100010001 in 6
				// for future reference
				pxor_r2r(mm6,mm6);
				pcmpeqw_r2r(mm1,mm1);
				psubw_r2r(mm1,mm6);


				// reg 5 = C 
				//movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
				movq_r2r(mm5,mm2);

				//reg 4 = minimum C
				movq_m2r(min_lr,mm4);
				movq_r2r(mm4,mm0);

				//fill 1 with 0x80008000s (sign bit)
				movq_r2r(mm6,mm1);
				psllw_i2r(15,mm1);

				paddw_r2r(mm1,mm0);
				paddw_r2r(mm1,mm2);
				//PRINTREG16(mm0,"min_lr");
				//PRINTREG16(mm2,"C");

				// reg 0 becomes mask
				pcmpgtw_r2r(mm2,mm0);
				movq_r2r(mm0,mm1);

				//PRINTREG16(mm2,"mask");

				// reg 0 is C values smaller than minimum
				movq_r2r(mm5,mm2);
				pand_r2r(mm0,mm2);

				// reg 4 is preserved minimum
				pandn_r2r(mm4,mm1);

				// reg 1 is new minimum
				por_r2r(mm2,mm1);
				movq_r2m(mm1,min_lr);
				//PRINTREG16(mm6,"new_min_lr");

				// reg 4 is d values for minimum
				movq_m2r(min_d_lr,mm4);
				// reg 1 is preserved d values
				movq_r2r(mm0,mm5);
				pandn_r2r(mm4,mm5);

				// reg 3 = current d value
				// increase current d values by 4
				movq_m2r(curr_d_lr,mm3);
				movq_r2r(mm3,mm2);


				// reg 0 = current d value for updated minimum
				pand_r2r(mm0,mm2);

				// reg 1 = new d's for minimum values
				por_r2r(mm5,mm2);
				movq_r2m(mm2,min_d_lr);
				//PRINTREG16(mm1,"new_min_d_lr");

				// increase current d's
				// fill with 0x0004000400040004 
				psllw_i2r(2,mm6);
				paddw_r2r(mm6,mm3);

				//paddw_m2r(plus_four,mm3);
				movq_r2m(mm3,curr_d_lr);

				// ---------------------------------------------
				// find minimum C Right->Left for C[x][d]      
				// ---------------------------------------------


			}// for d

			// first create & store 0x0001000100010001 in 6
			// for future reference
			//pxor_r2r(mm6,mm6);
			//pcmpeqw_r2r(mm0,mm0);
			//psubw_r2r(mm0,mm6);
			// ---------------------------------------------
			// find overall minimum C Left->Right for C[x+1][d]
			// --------------------------------------------

			//movq_m2r(min_lr,mm1);
			//movq_m2r(min_d_lr,mm2);

			//fill 0 with 0x80008000s (sign bit)
			//movq_r2r(mm6,mm0);
			psllw_i2r(13,mm6);
			//PRINTREG16(mm6,"0x80008000");

			// remove sign
			paddw_r2r(mm6,mm1);

			movq_r2r(mm1,mm3);
			movq_r2r(mm2,mm4);

			// once
			psrlq_i2r(16,mm3);
			psrlq_i2r(16,mm4);
			pminsw_r2r(mm1,mm3);

			movq_r2r(mm1,mm0);
			pcmpgtw_r2r(mm3,mm0);

			pand_r2r(mm0,mm4);
			pandn_r2r(mm2,mm0);
			por_r2r(mm0,mm4);

			// twice
			psrlq_i2r(16,mm3);
			psrlq_i2r(16,mm4);
			pminsw_r2r(mm1,mm3);

			movq_r2r(mm1,mm0);
			pcmpgtw_r2r(mm3,mm0);

			pand_r2r(mm0,mm4);
			pandn_r2r(mm2,mm0);
			por_r2r(mm0,mm4);


			// three times
			psrlq_i2r(16,mm3);
			psrlq_i2r(16,mm4);
			pminsw_r2r(mm1,mm3);


			movq_r2r(mm1,mm0);
			pcmpgtw_r2r(mm3,mm0);

			pand_r2r(mm0,mm4);
			pandn_r2r(mm2,mm0);
			por_r2r(mm0,mm4);

			//movq_m2r(min_d_lr,mm0);

			//movq_r2m(mm3,min_lr);
			movq_r2m(mm4,min_d_lr);
			//PRINTREG16(mm1,"min_lr");
			//PRINTREG16(mm3,"overall_min_lr");
			//PRINTREG16(mm0,"min_d_lr");
			//PRINTREG16(mm4,"overall_min_d_lr");

			overall_min_d_lr = min_d_lr.uw[0];
			overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr];

			((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = overall_min_d_lr;
		}      

		for(x=-1;x<X-W-D-1;x++) {

			overall_min_d_lr = ((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1];
			overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr];      
			// ---------------------------------------------
			// find overall minimum C Right->Left for C[x][d]      
			// check for consistency
			// ---------------------------------------------                                     

			min_d_rl = overall_min_d_lr;
			min_rl = overall_min_lr;

			//printf("min&d_rl:%u,%u\n",min_rl,d_rl);
			idx_rl = x+1+min_d_rl;
			for (d=0;d<D;d++) {
				if ((idx_rl>-1)&&(idx_rl<X)&&(min_rl >= ((MM_U_16 *)C->p[idx_rl].m)[d])) {
					min_rl = ((MM_U_16 *)C->p[idx_rl].m)[d];
					min_d_rl = d;
					//break;
				} // if
				idx_rl--;
			} // for
			if (abs(min_d_rl - overall_min_d_lr)>0) {
				//((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = 0x00;
				//((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = 0x00;
				overall_min_d_lr=0x00;
			}


			// ---------------------------------------------
			// subpixel interpolation      
			// weighted average of best match and each neighbour
			// ---------------------------------------------
			if ((overall_min_d_lr>0)&&(overall_min_d_lr<D-1)) {

				subpix_denom = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1] - overall_min_lr - overall_min_lr +((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1];
				subpix_d = (((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1]- ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1])<<shift;

				//fprintf(stderr,"%d %d %d ",overall_min_d_lr,subpix_d,subpix_denom);
				if (subpix_denom) {
					subpix_d /= (subpix_denom);
					subpix_d += (overall_min_d_lr<<shift);
				}
				else
					subpix_d = 0;
				//fprintf(stderr," %d  %d\n ",subpix_d,subpix_d/8);

				((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = subpix_d;

			}
			else 
				((MM_U_8 *)Out->p[y+v_2+1].m)[x+D_OFF+w_2+1] = 0x00;


		}// for x
	}// for y

	emms();
	MMXMatrixAssert(I1);
	MMXMatrixAssert(I2);
	MMXMatrixAssert(Out);
	return 0;
}

#else
// - for parallel cameras. offset introduced to give synthetic vergence
//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD4_leanne(const MMXMatrix *I1,const  MMXMatrix *I2,
				      MMXMatrix  *Out, const int D, 
				      const int D_OFF,  const int W, 
				      const int lr_threshold, 
				      MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
	//const int W = 16;
	//const int W = 8;
	//const int W = 4;

	const int shift = (D>32)?2:3;

	int Y,X;

	register int i, j;
	register int d,x,y;

	int w_2 = (W/2);
	//int d_w_2_1 = D_OFF+w_2+1;

	unsigned short overall_min_lr;

	int idx_rl; 
	unsigned short min_rl;

	int overall_min_d_lr;
	int d_rl; 
	int min_d_rl=0;

	int subpix_d,subpix_denom;

	MMXMatrix **P;
	MMXMatrix *Q;
	MMXMatrix *C1;
	MMXMatrix *C2;
	MMXMatrix *C;
	MMXMatrix *C_prev;
	mmx_t min_lr;
	mmx_t curr_d_lr;
	mmx_t min_d_lr = 0;

	unsigned char *pI1=NULL, *pI2=NULL;
	short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;

	X = I1->cols;
	Y = I1->rows;

	if (*cache==0) {
		unsigned int i = 0;
		*cache = MMXMatricesCacheAlloc(3+W,0,0);
		(*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
		(*cache)->M[i++] = C1 = MMXMatrixAlloc(MMT_U_16, D, X);
		(*cache)->M[i++] = C2 = MMXMatrixAlloc(MMT_U_16, D, X);
		P = (*cache)->M + 3;
		for (i=0;i<W;i++)
			P[i] = MMXMatrixAlloc(MMT_U_16, D, X);

	} else { 
		P = (*cache)->M + 3;
		Q = (*cache)->M[0]; 
		C1 = (*cache)->M[1];
		C2 = (*cache)->M[2];
	}

	emms();


	C = C2;
	C_prev = C1;


	for(y=-1;y<Y-W-1;y++){
		//fprintf(stderr,"depthmap: y:%d \n",y);
		if (C==C1) {
			C = C2;
			C_prev = C1;
		}
		else {
			C = C1;
			C_prev = C2;
		}

		/**********************************************
		  I have modified the following line from this:
		  for(x=-1;x<X-W-D-1;x++)  
		 **********************************************/
		for(x=-1;x<X-W-1;x++) {
			//fprintf(stderr,"depthmap: x:%d \n",x);


			// fill min with 111111111111111s (max number)
			pcmpeqw_r2r(mm0,mm0);
			movq_r2m(mm0,min_lr);

			curr_d_lr.ud[0] = 0x00010000;
			curr_d_lr.ud[1] = 0x00030002;

			// clear registers
			//pxor_r2r (mm3,mm3);
			// pxor_r2r (mm4,mm4); 

			pxor_r2r (mm7,mm7); 

			if (!(x<0)) {
				pCd = &(((MM_U_16 *)C->p[x].m)[0]);
				pC1d = &(((MM_U_16 *)C->p[x+1].m)[0]);
				pQd  = &(((MM_U_16 *)Q->p[x].m)[0]);
				pQWd  = &(((MM_U_16 *)Q->p[x+W].m)[0]);
				if (!(y<0)) {
					pPWd = &(((MM_U_16 *)P[y%W]->p[x+W].m)[0]);
				}
				pI1  = &(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF]);
				pI2  = &(((MM_U_8 *)I2->p[y+W].m)[x+W]);
			}


			for (d=0;d<D-3;d+=4) {
				//fprintf(stderr,"depthmap: d:%d \n",d);
				if ((x<0)&&(y<0)) {      
					// Equivalent C function
					//C[d][0]=0; 
					//for(i=0;i<=W;i+=4) {
					//  Q[d][i] = 0;
					// window down image
					//  for (j=0;j<W;j++) {
					//  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
					//  Q[d][i] += P[j][d][i];
					//  } 
					//  if (i<W)
					//    C[d][0] += Q[d][i];
					//}

					// reg 5 = C
					pxor_r2r (mm5,mm5);
					for(i=0;i<W;i++) {
						// reg 4 = Q
						pxor_r2r (mm4,mm4);

						// window down image
						for (j=0;j<W;j++) {
							// lbw in 0
							I1_I2D(I1, I2, i+D_OFF,j,i+d,j);
							//printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
							//printf("P(%d,%d,%d):",i,j,d);
							//PRINTREG16(mm1,"P");

							// Add curr P to Q
							paddw_r2r (mm1,mm4);
						}
						// Store Q
						movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
						//printf("Q(%d,%d):",i,d);
						// PRINTREG16(mm4,"Q");

						// Add curr Q to C
						paddw_r2r (mm4,mm5);
					}
					// Store C
					movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);

					// compute for i=W
					// reg 4 = Q
					pxor_r2r (mm4,mm4);  
					// window down image
					for (j=0;j<W;j++) {
						// lbw in 0
						I1_I2D(I1, I2, W+D_OFF,j,W+d,j);

						movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
						// Add curr P to Q
						paddw_r2r (mm1,mm4);
					}
					// Store Q
					movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);

				}
				else if (x<0) {
					// Equivalent C function
					//C[d][0]=0;
					//for(i=0;i<=W;i++) {
					// P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
					// Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
					// P[y%W][d][i] = P_temp;
					// if (i<W)
					//   C[d][0] += Q[d][i];
					//}

					// reg 5 = C
					pxor_r2r (mm5,mm5);
					for(i=0;i<W;i++) {
						// reg 4 = Q
						movq_m2r (((MM_U_16 *)Q->p[i].m)[d],mm4);  
						// Sub prev P from Q
						psubw_m2r (((MM_U_16 *)P[y%W]->p[i].m)[d],mm4);
						// Add curr P to Q
						// lbw in 0
						I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
						//PRINTREG16(mm1,"mmx1");
						// P_temp is reg 1
						paddw_r2r (mm1,mm4);
						// Store Q
						movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
						// Store P
						movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[i].m)[d]);
						// Add curr Q to C
						paddw_r2r (mm4,mm5);
					}
					// Store C[d][0]
					movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
					//PRINTREG16(mm5,"C");


					// Do for i=W
					// reg 4 = Q
					movq_m2r (((MM_U_16 *)Q->p[W].m)[d],mm4);  
					// Sub prev P from Q
					psubw_m2r (((MM_U_16 *)P[y%W]->p[W].m)[d],mm4);
					// Add curr P to Q
					// lbw in 0
					I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
					//PRINTREG16(mm1,"mmx1");
					// P_temp is reg 1
					paddw_r2r (mm1,mm4);
					// Store Q
					movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
					// Store P
					movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[W].m)[d]);
				}
				else {
					if (y<0) {
						// Equivalent C function
						//Q[x+W][d] = 0;
						//for (j=0;j<W;j++) {
						//  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
						//  Q[x+W][d] += P[x+W][j][d];
						//}

						// reg 4 = Q
						pxor_r2r (mm4,mm4);

						// window down image
						for (j=0;j<W;j++) {
							// lbw in 0
							I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
							//PRINTREG16(mm1,"mmx1");
							movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
							// Add curr P to Q
							paddw_r2r (mm1,mm4);
						}
						// Store Q
						movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);

					}  
					else {
						// Equivalent C function
						// P is [N][W][D] the W dimension is a wrap around buffer.
						//P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
						//Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
						//P[x+W][y%W][d] = P_temp;

						// reg 4 = Q
						//movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
						movq_m2r (*pQWd,mm4);

						// subtract prev P
						//psubw_m2r (((MM_U_16 *)P[y%W]->p[x+W].m)[d],mm4);
						psubw_m2r (*pPWd,mm4);

						// Add curr P to Q
						// lbw in 0
						//I1_I2D(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
						I1_I2Dptr(pI1, pI2);
						//PRINTREG16(mm1,"mmx1");
						// P_temp is reg 1
						paddw_r2r (mm1,mm4);

						// Store Q
						//movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
						movq_r2m (mm4,*pQWd);

						// Store P
						//movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[x+W].m)[d]);
						movq_r2m (mm1,*pPWd);

					}
					// Equivalent C function
					//C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];

					// reg 5 = C
					//movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);  
					movq_m2r (*pCd,mm5);


					// Sub curr Q[x][d] from C
					//psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
					psubw_m2r (*pQd,mm5);

					//PRINTREG16(mm5,"C-P");

					// Add Q[x+W][d] to C
					//movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
					//movq_m2r (*pQWd,mm4);
					//paddw_r2r (mm4,mm5);
					paddw_m2r (*pQWd,mm5);
					//PRINTREG16(mm5,"C-P+P");

					// Store C[x+1][d]
					//movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
					movq_r2m (mm5,*pC1d);


					pPWd += 4;
					pQd += 4;
					pQWd += 4;
					pCd += 4;
					pC1d += 4;
					pI2 += 4;

				}

				// ---------------------------------------------
				// find minimum C Left->Right for C[x+1][d]      
				// ---------------------------------------------
				// first create & store 0x0001000100010001 in 6
				// for future reference
				pxor_r2r(mm6,mm6);
				pcmpeqw_r2r(mm1,mm1);
				psubw_r2r(mm1,mm6);


				// reg 5 = C 
				//movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
				movq_r2r(mm5,mm2);

				//reg 4 = minimum C
				movq_m2r(min_lr,mm4);
				movq_r2r(mm4,mm0);

				//fill 1 with 0x80008000s (sign bit)
				movq_r2r(mm6,mm1);
				psllw_i2r(15,mm1);

				paddw_r2r(mm1,mm0);
				paddw_r2r(mm1,mm2);
				//PRINTREG16(mm0,"min_lr");
				//PRINTREG16(mm2,"C");

				// reg 0 becomes mask
				pcmpgtw_r2r(mm2,mm0);
				movq_r2r(mm0,mm1);

				//PRINTREG16(mm2,"mask");

				// reg 0 is C values smaller than minimum
				movq_r2r(mm5,mm2);
				pand_r2r(mm0,mm2);

				// reg 4 is preserved minimum
				pandn_r2r(mm4,mm1);

				// reg 1 is new minimum
				por_r2r(mm2,mm1);
				movq_r2m(mm1,min_lr);
				//PRINTREG16(mm6,"new_min_lr");

				// reg 4 is d values for minimum
				movq_m2r(min_d_lr,mm4);
				// reg 1 is preserved d values
				movq_r2r(mm0,mm5);
				pandn_r2r(mm4,mm5);

				// reg 3 = current d value
				// increase current d values by 4
				movq_m2r(curr_d_lr,mm3);
				movq_r2r(mm3,mm2);


				// reg 0 = current d value for updated minimum
				pand_r2r(mm0,mm2);

				// reg 1 = new d's for minimum values
				por_r2r(mm5,mm2);
				movq_r2m(mm2,min_d_lr);
				//PRINTREG16(mm1,"new_min_d_lr");

				// increase current d's
				// fill with 0x0004000400040004 
				psllw_i2r(2,mm6);
				paddw_r2r(mm6,mm3);

				//paddw_m2r(plus_four,mm3);
				movq_r2m(mm3,curr_d_lr);

				// ---------------------------------------------
				// find minimum C Right->Left for C[x][d]      
				// ---------------------------------------------


			}// for d

			// first create & store 0x0001000100010001 in 6
			// for future reference
			//pxor_r2r(mm6,mm6);
			//pcmpeqw_r2r(mm0,mm0);
			//psubw_r2r(mm0,mm6);
			// ---------------------------------------------
			// find overall minimum C Left->Right for C[x+1][d]
			// --------------------------------------------

			//movq_m2r(min_lr,mm1);
			//movq_m2r(min_d_lr,mm2);

			//fill 0 with 0x80008000s (sign bit)
			//movq_r2r(mm6,mm0);
			psllw_i2r(13,mm6);
			//PRINTREG16(mm6,"0x80008000");

			// remove sign
			paddw_r2r(mm6,mm1);

			movq_r2r(mm1,mm3);
			movq_r2r(mm2,mm4);

			// once
			psrlq_i2r(16,mm3);
			psrlq_i2r(16,mm4);
			pminsw_r2r(mm1,mm3);

			movq_r2r(mm1,mm0);
			pcmpgtw_r2r(mm3,mm0);

			pand_r2r(mm0,mm4);
			pandn_r2r(mm2,mm0);
			por_r2r(mm0,mm4);

			// twice
			psrlq_i2r(16,mm3);
			psrlq_i2r(16,mm4);
			pminsw_r2r(mm1,mm3);

			movq_r2r(mm1,mm0);
			pcmpgtw_r2r(mm3,mm0);

			pand_r2r(mm0,mm4);
			pandn_r2r(mm2,mm0);
			por_r2r(mm0,mm4);


			// three times
			psrlq_i2r(16,mm3);
			psrlq_i2r(16,mm4);
			pminsw_r2r(mm1,mm3);


			movq_r2r(mm1,mm0);
			pcmpgtw_r2r(mm3,mm0);

			pand_r2r(mm0,mm4);
			pandn_r2r(mm2,mm0);
			por_r2r(mm0,mm4);

			//movq_m2r(min_d_lr,mm0);

			//movq_r2m(mm3,min_lr);
			movq_r2m(mm4,min_d_lr);
			//PRINTREG16(mm1,"min_lr");
			//PRINTREG16(mm3,"overall_min_lr");
			//PRINTREG16(mm0,"min_d_lr");
			//PRINTREG16(mm4,"overall_min_d_lr");

			overall_min_d_lr = min_d_lr.uw[0];
			overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr];

			((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = overall_min_d_lr<<shift;

			// ---------------------------------------------
			// subpixel interpolation      
			// weighted average of best match and each neighbour
			// ---------------------------------------------
			//emms();
			if ((overall_min_d_lr>0)&&(overall_min_d_lr<D)) {

				subpix_denom = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1] - overall_min_lr - overall_min_lr +((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1];
				subpix_d = (((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1]- ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1])<<shift;

				//fprintf(stderr,"%d %d %d ",overall_min_d_lr,subpix_d,subpix_denom);
				if (subpix_denom) {
					subpix_d /= (subpix_denom);
					subpix_d += (overall_min_d_lr<<shift);
				}
				else
					subpix_d = 0;
				//fprintf(stderr," %d  %d\n ",subpix_d,subpix_d/8);

				((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = subpix_d;
			}
			else
				((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = (overall_min_d_lr<<shift);

			// ---------------------------------------------
			// find overall minimum C Right->Left for C[x][d]      
			// check for consistency
			// ---------------------------------------------                                     
			if (!(y<0)) {
				d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+D_OFF+w_2+1])>>shift;
				min_rl = ((MM_U_16 *)C_prev->p[x+1].m)[d_rl];

				//printf("min&d_rl:%u,%u\n",min_rl,d_rl);
				idx_rl = x+1+d_rl;
				for (d=0;d<D;d++) {
					if ((idx_rl>-1)&&(idx_rl<X)&&(min_rl >= ((MM_U_16 *)C_prev->p[idx_rl].m)[d])) {
						min_rl = ((MM_U_16 *)C_prev->p[idx_rl].m)[d];
						min_d_rl = d;
						//printf("better&d_rl:%u,%d\n",((MM_U_16 *)C_prev->p[idx_rl].m)[d],d);
						//((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1] = 0x00;
						//break;
					} // if
					idx_rl--;
				} // for
				/* Have changed the following line from this:
				   if (abs(min_d_rl - d_rl)>2)  
				 */
				if (abs(min_d_rl - d_rl)>lr_threshold) {
					((MM_U_8 *)Out->p[y+w_2].m)[x+D_OFF+w_2+1] = 0x00;
				}
				//else {
				//((MM_U_8 *)Out->p[y+w_2].m)[x+D_OFF+w_2+1] = 0xff - ((MM_U_8 *)Out->p[y+w_2].m)[x+D_OFF+w_2+1];

				//printf("better d_rl:%d %d\n",min_d_rl, d_rl);
				//}
			} // if (!y

		}// for x
	}// for y

	emms();
	return 0;
}
#endif


// - for parallel cameras. offset introduced to give synthetic vergence
//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD4point1(const MMXMatrix *I1,const  MMXMatrix *I2,
				      MMXMatrix  *Out, const int D, 
				      const int D_OFF,  const int W, 
				      const int lr_threshold, 
				      MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
    //const int W = 16;
    //const int W = 8;
    //const int W = 4;

  const int shift = (D>32)?2:3;

  int Y,X;

  register int i, j;
  register int d,x,y;
  
  int w_2 = (W/2);
  //int d_w_2_1 = D_OFF+w_2+1;

  unsigned short overall_min_lr;

  int idx_rl; 
  unsigned short min_rl;

  int overall_min_d_lr;
  int d_rl; 
  int min_d_rl=0;

  int subpix_d,subpix_denom;

  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C1;
  MMXMatrix *C2;
  MMXMatrix *C;
  MMXMatrix *C_prev;
  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  unsigned char *pI1=NULL, *pI2=NULL;
  unsigned short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;
  
  X = I1->cols;
  Y = I1->rows;

  if (*cache==0) {
    i=0;
    *cache = MMXMatricesCacheAlloc(3+W,0,0);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C1 = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C2 = MMXMatrixAlloc(MMT_U_16, D, X);
    P = (*cache)->M + 3;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, D, X);
  }
  else { 
    P = (*cache)->M+3;
    Q = (*cache)->M[0];
    C1 = (*cache)->M[1];
    C2 = (*cache)->M[2];
  }

  emms();


  C = C2;
  C_prev = C1;


  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }
    
    /**********************************************
       I have modified the following line from this:
       for(x=-1;x<X-W-D-1;x++) { 
    **********************************************/
    for(x=-1;x<X-W-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      
      
      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);
      
      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;
      
      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4); 
      
      pxor_r2r (mm7,mm7); 

      if (!(x<0)) {
        pCd = &(((MM_U_16 *)C->p[x].m)[0]);
        pC1d = &(((MM_U_16 *)C->p[x+1].m)[0]);
        pQd  = &(((MM_U_16 *)Q->p[x].m)[0]);
        pQWd  = &(((MM_U_16 *)Q->p[x+W].m)[0]);
        if (!(y<0)) {
          pPWd = &(((MM_U_16 *)P[y%W]->p[x+W].m)[0]);
        }
        pI1  = &(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF]);
        pI2  = &(((MM_U_8 *)I2->p[y+W].m)[x+W]);
      }

      
      for (d=0;d<D-3;d+=4) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {      
          // Equivalent C function
          //C[d][0]=0; 
          //for(i=0;i<=W;i+=4) {
          //  Q[d][i] = 0;
          // window down image
          //  for (j=0;j<W;j++) {
          //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
          //  Q[d][i] += P[j][d][i];
          //  } 
          //  if (i<W)
          //    C[d][0] += Q[d][i];
          //}
    
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2D(I1, I2, i+D_OFF,j,i+d,j);
              //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
              //printf("P(%d,%d,%d):",i,j,d);
              //PRINTREG16(mm1,"P");
        
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
            //printf("Q(%d,%d):",i,d);
            // PRINTREG16(mm4,"Q");
    
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          
          // compute for i=W
          // reg 4 = Q
          pxor_r2r (mm4,mm4);  
          // window down image
          for (j=0;j<W;j++) {
            // lbw in 0
            I1_I2D(I1, I2, W+D_OFF,j,W+d,j);
            
            movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
            // Add curr P to Q
            paddw_r2r (mm1,mm4);
          }
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
         
        }
        else if (x<0) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i++) {
          // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
          // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
          // P[y%W][d][i] = P_temp;
          // if (i<W)
          //   C[d][0] += Q[d][i];
          //}
          
          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            movq_m2r (((MM_U_16 *)Q->p[i].m)[d],mm4);  
            // Sub prev P from Q
            psubw_m2r (((MM_U_16 *)P[y%W]->p[i].m)[d],mm4);
            // Add curr P to Q
            // lbw in 0
            I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
            // Store P
            movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[i].m)[d]);
            // Add curr Q to C
            paddw_r2r (mm4,mm5);
          }
          // Store C[d][0]
          movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
          //PRINTREG16(mm5,"C");
          
          
          // Do for i=W
          // reg 4 = Q
          movq_m2r (((MM_U_16 *)Q->p[W].m)[d],mm4);  
          // Sub prev P from Q
          psubw_m2r (((MM_U_16 *)P[y%W]->p[W].m)[d],mm4);
          // Add curr P to Q
          // lbw in 0
          I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
          //PRINTREG16(mm1,"mmx1");
          // P_temp is reg 1
          paddw_r2r (mm1,mm4);
          // Store Q
          movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
          // Store P
          movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[W].m)[d]);
        }
        else {
          if (y<0) {
            // Equivalent C function
            //Q[x+W][d] = 0;
            //for (j=0;j<W;j++) {
            //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
            //  Q[x+W][d] += P[x+W][j][d];
            //}
            
            // reg 4 = Q
            pxor_r2r (mm4,mm4);
            
            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
              //PRINTREG16(mm1,"mmx1");
              movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            
          }  
          else {
            // Equivalent C function
            // P is [N][W][D] the W dimension is a wrap around buffer.
            //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
            //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
            //P[x+W][y%W][d] = P_temp;
            
            // reg 4 = Q
            //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
            movq_m2r (*pQWd,mm4);

            // subtract prev P
            //psubw_m2r (((MM_U_16 *)P[y%W]->p[x+W].m)[d],mm4);
            psubw_m2r (*pPWd,mm4);

            // Add curr P to Q
            // lbw in 0
            //I1_I2D(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
            I1_I2Dptr(pI1, pI2);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddw_r2r (mm1,mm4);
            
            // Store Q
            //movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
            movq_r2m (mm4,*pQWd);

            // Store P
            //movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[x+W].m)[d]);
            movq_r2m (mm1,*pPWd);

          }
          // Equivalent C function
          //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
          
          // reg 5 = C
          //movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);  
          movq_m2r (*pCd,mm5);

          
          // Sub curr Q[x][d] from C
          //psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
          psubw_m2r (*pQd,mm5);

          //PRINTREG16(mm5,"C-P");
          
          // Add Q[x+W][d] to C
          //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
          //movq_m2r (*pQWd,mm4);
          //paddw_r2r (mm4,mm5);
          paddw_m2r (*pQWd,mm5);
          //PRINTREG16(mm5,"C-P+P");
          
          // Store C[x+1][d]
          //movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
          movq_r2m (mm5,*pC1d);
         
           
          pPWd += 4;
          pQd += 4;
          pQWd += 4;
          pCd += 4;
          pC1d += 4;
          pI2 += 4;
          
        }

        // ---------------------------------------------
        // find minimum C Left->Right for C[x+1][d]      
        // ---------------------------------------------
        // first create & store 0x0001000100010001 in 6
        // for future reference
        pxor_r2r(mm6,mm6);
        pcmpeqw_r2r(mm1,mm1);
        psubw_r2r(mm1,mm6);
        

        // reg 5 = C 
        //movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
        movq_r2r(mm5,mm2);
        
        //reg 4 = minimum C
        movq_m2r(min_lr,mm4);
        movq_r2r(mm4,mm0);
        
        //fill 1 with 0x80008000s (sign bit)
        movq_r2r(mm6,mm1);
        psllw_i2r(15,mm1);

        paddw_r2r(mm1,mm0);
        paddw_r2r(mm1,mm2);
        //PRINTREG16(mm0,"min_lr");
        //PRINTREG16(mm2,"C");
        
        // reg 0 becomes mask
        pcmpgtw_r2r(mm2,mm0);
        movq_r2r(mm0,mm1);
        
        //PRINTREG16(mm2,"mask");
        
        // reg 0 is C values smaller than minimum
        movq_r2r(mm5,mm2);
        pand_r2r(mm0,mm2);
        
        // reg 4 is preserved minimum
        pandn_r2r(mm4,mm1);
        
        // reg 1 is new minimum
        por_r2r(mm2,mm1);
        movq_r2m(mm1,min_lr);
        //PRINTREG16(mm6,"new_min_lr");
        
        // reg 4 is d values for minimum
        movq_m2r(min_d_lr,mm4);
        // reg 1 is preserved d values
        movq_r2r(mm0,mm5);
        pandn_r2r(mm4,mm5);
        
        // reg 3 = current d value
        // increase current d values by 4
        movq_m2r(curr_d_lr,mm3);
        movq_r2r(mm3,mm2);
        

        // reg 0 = current d value for updated minimum
        pand_r2r(mm0,mm2);
        
        // reg 1 = new d's for minimum values
        por_r2r(mm5,mm2);
        movq_r2m(mm2,min_d_lr);
        //PRINTREG16(mm1,"new_min_d_lr");

        // increase current d's
        // fill with 0x0004000400040004 
        psllw_i2r(2,mm6);
        paddw_r2r(mm6,mm3);
  
        //paddw_m2r(plus_four,mm3);
        movq_r2m(mm3,curr_d_lr);
  
        // ---------------------------------------------
        // find minimum C Right->Left for C[x][d]      
        // ---------------------------------------------
        
  
      }// for d
      
      // first create & store 0x0001000100010001 in 6
      // for future reference
      //pxor_r2r(mm6,mm6);
      //pcmpeqw_r2r(mm0,mm0);
      //psubw_r2r(mm0,mm6);
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]
      // --------------------------------------------
      
      //movq_m2r(min_lr,mm1);
      //movq_m2r(min_d_lr,mm2);

      //fill 0 with 0x80008000s (sign bit)
      //movq_r2r(mm6,mm0);
      psllw_i2r(13,mm6);
      //PRINTREG16(mm6,"0x80008000");

      // remove sign
      paddw_r2r(mm6,mm1);

      movq_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);

      // once
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // twice
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);

      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);


      // three times
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);


      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);
 
      //movq_m2r(min_d_lr,mm0);

      //movq_r2m(mm3,min_lr);
      movq_r2m(mm4,min_d_lr);
      //PRINTREG16(mm1,"min_lr");
      //PRINTREG16(mm3,"overall_min_lr");
      //PRINTREG16(mm0,"min_d_lr");
      //PRINTREG16(mm4,"overall_min_d_lr");
      
      overall_min_d_lr = min_d_lr.uw[0];
      overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr];

      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = overall_min_d_lr<<shift;

      // ---------------------------------------------
      // subpixel interpolation      
      // weighted average of best match and each neighbour
      // ---------------------------------------------
      //emms();
      if ((overall_min_d_lr>0)&&(overall_min_d_lr<D)) {

        subpix_denom = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1] - overall_min_lr - overall_min_lr +((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1];
        subpix_d = (((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1]- ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1])<<shift;

        //fprintf(stderr,"%d %d %d ",overall_min_d_lr,subpix_d,subpix_denom);
        if (subpix_denom) {
          subpix_d /= (subpix_denom);
          subpix_d += (overall_min_d_lr<<shift);
        }
        else
          subpix_d = 0;
        //fprintf(stderr," %d  %d\n ",subpix_d,subpix_d/8);

        ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = subpix_d;
     }
     else
        ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = (overall_min_d_lr<<shift);

      // ---------------------------------------------
      // find overall minimum C Right->Left for C[x][d]      
      // check for consistency
      // ---------------------------------------------                                     
      if (!(y<0)) {
        d_rl = (((MM_U_8 *)Out->p[y+w_2].m)[x+D_OFF+w_2+1])>>shift;
        min_rl = ((MM_U_16 *)C_prev->p[x+1].m)[d_rl];

        //printf("min&d_rl:%u,%u\n",min_rl,d_rl);
        idx_rl = x+1+d_rl;
        for (d=0;d<D;d++) {
          if ((idx_rl>-1)&&(idx_rl<X)&&(min_rl >= ((MM_U_16 *)C_prev->p[idx_rl].m)[d])) {
            min_rl = ((MM_U_16 *)C_prev->p[idx_rl].m)[d];
            min_d_rl = d;
            //printf("better&d_rl:%u,%d\n",((MM_U_16 *)C_prev->p[idx_rl].m)[d],d);
            //((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1] = 0x00;
            //break;
          } // if
          idx_rl--;
          } // for
	/* Have changed the following line from this:
	   if (abs(min_d_rl - d_rl)>2) { 
	*/
          if (abs(min_d_rl - d_rl)>lr_threshold) {
                  ((MM_U_8 *)Out->p[y+w_2].m)[x+D_OFF+w_2+1] = 0x00;
                }
                //else {
                //((MM_U_8 *)Out->p[y+w_2].m)[x+D_OFF+w_2+1] = 0xff - ((MM_U_8 *)Out->p[y+w_2].m)[x+D_OFF+w_2+1];
                
                //printf("better d_rl:%d %d\n",min_d_rl, d_rl);
                //}
       } // if (!y

     }// for x
  }// for y
    
  emms();
  return 0;
}


//#define SUBPIX_TOL 2020
#define SUBPIX_TOL 2040
//#define SUBPIX_TOL 1400
//#define SUBPIX_TOL 1800

// - for parallel cameras. offset introduced to give synthetic vergence
// - linear subpix interp
//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD5(const MMXMatrix *I1,const  MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  //const int W = 16;
  const int W = 8;
  //const int W = 4;
  //const int W = 12;
  //const int W = 2;
  //const int W = 4;

  const int w_2 = (W/2);
  const int shift = (D<=8)?5:(D<=16)?4:(D<=32)?3:2;
  const int X = I1->cols;
  const int Y = I1->rows;

  register int i, j;
  register int d,x,y;

  //int d_w_2_1 = D_OFF+w_2+1;

  int overall_min_lr, overall_min_lrm1,overall_min_lrp1;
  unsigned short min_rl;

  int idx_rl;
  int min_d_rl=0;
   int subpix_d;


  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C;

  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  unsigned short *O,*pO;
  unsigned char *pI1=NULL, *pI2=NULL;
  unsigned short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;

  if (*cache==0) {
    i = 0;
    *cache = MMXMatricesCacheAlloc(2+W,0,1);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->G[0] = O = Alloc(sizeof(unsigned short) *X);
    P = (*cache)->M + 2;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, D, X);
  }
  else {
    P = (*cache)->M + 2;
    Q = (*cache)->M[0]; 
    C = (*cache)->M[1]; 
    O = (unsigned short *) (*cache)->G[0];
  }

  emms();




  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
     pO = O;

    for(x=-1;x<X-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);


      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;

      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4);

      pxor_r2r (mm7,mm7);

      if (!(x<0)) {
        pCd = &(((MM_U_16 *)C->p[x].m)[0]);
        pC1d = &(((MM_U_16 *)C->p[x+1].m)[0]);
        pQd  = &(((MM_U_16 *)Q->p[x].m)[0]);
        pQWd  = &(((MM_U_16 *)Q->p[x+W].m)[0]);
        if (!(y<0)) {
          pPWd = &(((MM_U_16 *)P[y%W]->p[x+W].m)[0]);
        }
        pI1  = &(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF]);
        pI2  = &(((MM_U_8 *)I2->p[y+W].m)[x+W]);
      }


      for (d=0;d<D-3;d+=4) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i+=4) {
                  //  Q[d][i] = 0;
                  // window down image
                  //  for (j=0;j<W;j++) {
            //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
            //  Q[d][i] += P[j][d][i];
            //  }
            //  if (i<W)
            //    C[d][0] += Q[d][i];
                  //}

                  // reg 5 = C
	  pxor_r2r (mm5,mm5);
	  for(i=0;i<W;i++) {
	    // reg 4 = Q
	    pxor_r2r (mm4,mm4);
	    
	    // window down image
	    for (j=0;j<W;j++) {
	      // lbw in 0
	      I1_I2D(I1, I2, i+D_OFF,j,i+d,j);
	      //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
	      //printf("P(%d,%d,%d):",i,j,d);
	      //PRINTREG16(mm1,"P");
	      
	      // Add curr P to Q
	      paddw_r2r (mm1,mm4);
	    }
	    // Store Q
	    movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
	    //printf("Q(%d,%d):",i,d);
	    // PRINTREG16(mm4,"Q");
	    
	    // Add curr Q to C
	    paddw_r2r (mm4,mm5);
	  }
	  // Store C
	  movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
	  
	  // compute for i=W
	  // reg 4 = Q
	  pxor_r2r (mm4,mm4);
	  // window down image
	  for (j=0;j<W;j++) {
	    // lbw in 0
	    I1_I2D(I1, I2, W+D_OFF,j,W+d,j);
	    
	    movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
	    // Add curr P to Q
	    paddw_r2r (mm1,mm4);
	  }
	  // Store Q
	  movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);                     
	}
	else if (x<0) {
	  // Equivalent C function
	  //C[d][0]=0;
	  //for(i=0;i<=W;i++) {
	  // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
	  // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
	  // P[y%W][d][i] = P_temp;
	  // if (i<W)
	  //   C[d][0] += Q[d][i];
	  //}
                
	  // reg 5 = C
	  pxor_r2r (mm5,mm5);
	  for(i=0;i<W;i++) {
	    // reg 4 = Q
	    movq_m2r (((MM_U_16 *)Q->p[i].m)[d],mm4);
	    // Sub prev P from Q
	    psubw_m2r (((MM_U_16 *)P[y%W]->p[i].m)[d],mm4);
	    // Add curr P to Q
	    // lbw in 0
	    I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
	    //PRINTREG16(mm1,"mmx1");
	    // P_temp is reg 1
	    paddw_r2r (mm1,mm4);
	    // Store Q
	    movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
	    // Store P
	    movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[i].m)[d]);
	    // Add curr Q to C
	    paddw_r2r (mm4,mm5);
	  }
	  // Store C[d][0]
	  movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
	  //PRINTREG16(mm5,"C");
	  
	  // Do for i=W
	  // reg 4 = Q
	  movq_m2r (((MM_U_16 *)Q->p[W].m)[d],mm4);
	  // Sub prev P from Q
	  psubw_m2r (((MM_U_16 *)P[y%W]->p[W].m)[d],mm4);
	  // Add curr P to Q
	  // lbw in 0
	  I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
	  //PRINTREG16(mm1,"mmx1");
	  // P_temp is reg 1
	  paddw_r2r (mm1,mm4);
	  // Store Q
	  movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
	  // Store P
	  movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[W].m)[d]);
	}
	else {
	  if (y<0) {
	    // Equivalent C function
	    //Q[x+W][d] = 0;
	    //for (j=0;j<W;j++) {
	    //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
	    //  Q[x+W][d] += P[x+W][j][d];
	    //}
                        
	    // reg 4 = Q
	    pxor_r2r (mm4,mm4);
	    
	    // window down image
	    for (j=0;j<W;j++) {
	      // lbw in 0
	      I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
	      //PRINTREG16(mm1,"mmx1");
	      movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
	      // Add curr P to Q
	      paddw_r2r (mm1,mm4);
	    }
	    // Store Q
	    movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
	    
	  }
	  else {
	    // Equivalent C function
	    // P is [N][W][D] the W dimension is a wrap around buffer.
	    //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
	    //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
	    //P[x+W][y%W][d] = P_temp;
	    
	    // reg 4 = Q
	    //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
	    movq_m2r (*pQWd,mm4);
	    
	    // subtract prev P
	    //psubw_m2r (((MM_U_16 *)P[y%W]->p[x+W].m)[d],mm4);
	    psubw_m2r (*pPWd,mm4);
	    
	    // Add curr P to Q
	    // lbw in 0
	    //I1_I2D(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
	    I1_I2Dptr(pI1, pI2);
	    //PRINTREG16(mm1,"mmx1");
	    // P_temp is reg 1
	    paddw_r2r (mm1,mm4);
	    
	    // Store Q
	    //movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
	    movq_r2m (mm4,*pQWd);
	    
	    // Store P
	    //movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[x+W].m)[d]);
	    movq_r2m (mm1,*pPWd);

	  }
	  // Equivalent C function
	  //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
	  
	  // reg 5 = C
	  //movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);
	  movq_m2r (*pCd,mm5);
	  
                        
	  // Sub curr Q[x][d] from C
	  //psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
	  psubw_m2r (*pQd,mm5);
	  
	  //PRINTREG16(mm5,"C-P");
	  
	  // Add Q[x+W][d] to C
	  //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
	  //movq_m2r (*pQWd,mm4);
	  //paddw_r2r (mm4,mm5);
	  paddw_m2r (*pQWd,mm5);
	  //PRINTREG16(mm5,"C-P+P");
                        
	  // Store C[x+1][d]
	  //movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
	  movq_r2m (mm5,*pC1d);
                        
                        
	  pPWd += 4;
	  pQd += 4;
	  pQWd += 4;
	  pCd += 4;
	  pC1d += 4;
	  pI2 += 4;
                        
	}
	
	// ---------------------------------------------
	// find minimum C Left->Right for C[x+1][d]
	// ---------------------------------------------
	// first create & store 0x0001000100010001 in 6
	// for future reference
	pxor_r2r(mm6,mm6);
	pcmpeqw_r2r(mm1,mm1);
	psubw_r2r(mm1,mm6);
	
	
	// reg 5 = C
	//movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
	movq_r2r(mm5,mm2);
	
	//reg 4 = minimum C
	movq_m2r(min_lr,mm4);
	movq_r2r(mm4,mm0);
	
	//fill 1 with 0x80008000s (sign bit)
	movq_r2r(mm6,mm1);
	psllw_i2r(15,mm1);
	
	paddw_r2r(mm1,mm0);
	paddw_r2r(mm1,mm2);
	//PRINTREG16(mm0,"min_lr");
	//PRINTREG16(mm2,"C");

	// reg 0 becomes mask
	pcmpgtw_r2r(mm2,mm0);
	movq_r2r(mm0,mm1);
                
	//PRINTREG16(mm2,"mask");
        
	// reg 0 is C values smaller than minimum
	movq_r2r(mm5,mm2);
	pand_r2r(mm0,mm2);
	
	// reg 4 is preserved minimum
	pandn_r2r(mm4,mm1);
	
	// reg 1 is new minimum
	por_r2r(mm2,mm1);
	movq_r2m(mm1,min_lr);
	//PRINTREG16(mm6,"new_min_lr");
	
	// reg 4 is d values for minimum
	movq_m2r(min_d_lr,mm4);
	// reg 1 is preserved d values
	movq_r2r(mm0,mm5);
	pandn_r2r(mm4,mm5);
                
	// reg 3 = current d value
	// increase current d values by 4
	movq_m2r(curr_d_lr,mm3);
	movq_r2r(mm3,mm2);
		

	// reg 0 = current d value for updated minimum
	pand_r2r(mm0,mm2);
        
	// reg 1 = new d's for minimum values
	por_r2r(mm5,mm2);
	movq_r2m(mm2,min_d_lr);
	//PRINTREG16(mm1,"new_min_d_lr");
	
	// increase current d's
	// fill with 0x0004000400040004
	psllw_i2r(2,mm6);
	paddw_r2r(mm6,mm3);
	
	//paddw_m2r(plus_four,mm3);
	movq_r2m(mm3,curr_d_lr);
      }// for d
      
      // first create & store 0x0001000100010001 in 6
      // for future reference
      //pxor_r2r(mm6,mm6);
      //pcmpeqw_r2r(mm0,mm0);
      //psubw_r2r(mm0,mm6);
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]
      // --------------------------------------------
      
      //movq_m2r(min_lr,mm1);
      //movq_m2r(min_d_lr,mm2);
      
      //fill 0 with 0x80008000s (sign bit)
      //movq_r2r(mm6,mm0);
      psllw_i2r(13,mm6);
      //PRINTREG16(mm6,"0x80008000");
      
      // remove sign
      paddw_r2r(mm6,mm1);
      
      movq_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);
      
      // once
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);
      
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);
	    
      // twice
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);
      
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);

      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);
      

      // three times
      psrlq_i2r(16,mm3);
      psrlq_i2r(16,mm4);
      pminsw_r2r(mm1,mm3);
      
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);
      
      //movq_m2r(min_d_lr,mm0);

      movq_r2m(mm4,min_d_lr);

      //            overall_min_d_lr = min_d_lr.uw[0];
      //O[x+1] = min_d_lr.uw[0];
      *pO = min_d_lr.uw[0];
      pO++;
      //overall_min_lr = min_lr.uw[0]; // wrong because of sign eradication


      //((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = overall_min_d_lr;
      //((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = min_d_lr.uw[0];
    } // for x
    
    pO=O;
    for(x=-1;x<X-W-D-1;x++) {

//           overall_min_d_lr = ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1];
            // ---------------------------------------------
            // subpixel interpolation
            // weighted average of best match and each neighbour
            // ---------------------------------------------
// if adjacent disparities correlation result that close to the minimum we have an unreliable match
// 0.49 * shift factor
//#define SUBPIX_TOL 2040
//#define SUBPIX_TOL 2000
//#define SUBPIX_TOL 1900
//#define SUBPIX_TOL 1800
//#define SUBPIX_TOL 1600
//#define SUBPIX_TOL 1400
//            overall_min_d_lr = O[x+1];
      if ((*pO)&&(*pO<D-1)) {
        overall_min_lrm1 = ((MM_U_16 *)C->p[x+1].m)[*pO-1];
        overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[*pO];
        overall_min_lrp1 = ((MM_U_16 *)C->p[x+1].m)[*pO+1];
        
        if ((overall_min_lr<<12)<(SUBPIX_TOL*(overall_min_lrp1 + overall_min_lrm1))) {
          if (overall_min_lrm1<overall_min_lrp1) {
            subpix_d = -(overall_min_lr<<shift)/(1+overall_min_lrm1+ overall_min_lr);
          }
          else {
            subpix_d = (overall_min_lr<<shift)/(1+overall_min_lrp1 + overall_min_lr);
          }
          //subpix_d = subpix_d>>shift;
          subpix_d += *pO<<shift;
          //subpix_d = (subpix_d>>shift) + (overall_min_d_lr<<shift);
          ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = subpix_d;
        }
        else {
          subpix_d = 0;
          *pO = 0x0000;
        }
      }
      else {
        subpix_d = *pO<<shift;
        ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = subpix_d;
      } // if ((overall

      pO++;
    } // for x

    pO=O;
    for(x=-1;x<X-W-D-1;x++) {
      // ---------------------------------------------
      // find overall minimum C Right->Left for C[x][d]
      // check for consistency
      // ---------------------------------------------
      if (!(y<0)&&*pO) {
        min_rl = ((MM_U_16 *)C->p[x+1].m)[*pO];
        idx_rl = x+1+*pO;
        for (d=0;d<D;d++) {
          if ((idx_rl>-1)&&(idx_rl<X)&&((MM_U_16 *)C->p[idx_rl].m)[d]&&!(min_rl < ((MM_U_16 *)C->p[idx_rl].m)[d])) {
            min_rl = ((MM_U_16 *)C->p[idx_rl].m)[d];
            min_d_rl = d;
            //break;
          } // if
          idx_rl--;
        } // for
        if (abs(min_d_rl - *pO)>0) {
          //((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = 0x00;
          //*pO = 0x0000;
        }
        
      } // if (!y
      
      pO++;
    }// for x
    
  }// for y
  emms();
  return 0;
}


// - for parallel cameras. offset introduced to give synthetic vergence
// - linear subpix interp
//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD32(const MMXMatrix *I1,const  MMXMatrix *I2, MMXMatrix  *Out,MMXMatricesCachePtr* vcache)
{
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  const int W = 16;
  //const int W = 8;
  //const int W = 4;

  const int w_2 = (W/2);
  const int X = I1->cols;
  const int Y = I1->rows;

  register int i, j;
  register int d,x,y;



  int overall_min_lr, overall_min_lrm1,overall_min_lrp1;
  int idx_rl;
  int min_d_rl=0;
   int subpix_d;
    int ymodW;

  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C;

  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  unsigned short *O,*pO;
  unsigned char *pI1=NULL, *pI2=NULL;
  unsigned short *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;
  unsigned short min_rl;

  if (*cache==0) {
    i = 0;
    *cache = MMXMatricesCacheAlloc(2+W,0,1);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, 32, X);
    (*cache)->M[i++] = C = MMXMatrixAlloc(MMT_U_16, 32, X);
    (*cache)->G[0] = O = Alloc(sizeof(unsigned short) *X);
    P = (*cache)->M + 2;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, 32, X);
  }
  else {
    P = (*cache)->M + 2;
    Q = (*cache)->M[0];
    C = (*cache)->M[1];
    O = (unsigned short *)(*cache)->G[0];
  }

  emms();




  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
     pO = O;
     ymodW= y%W;

    for(x=-1;x<X-W-32-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);


      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;

      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4);

      pxor_r2r (mm7,mm7);

      if (x>=0) {
        pCd = &(((MM_U_16 *)C->p[x].m)[0]);
        pC1d = &(((MM_U_16 *)C->p[x+1].m)[0]);
        pQd  = &(((MM_U_16 *)Q->p[x].m)[0]);
        pQWd  = &(((MM_U_16 *)Q->p[x+W].m)[0]);
        if (y>=0) {
          pPWd = &(((MM_U_16 *)P[ymodW]->p[x+W].m)[0]);
        }
        pI1  = &(((MM_U_8 *)I1->p[y+W].m)[x+W]);
        pI2  = &(((MM_U_8 *)I2->p[y+W].m)[x+W]);
      }


      for (d=0;d<32-3;d+=4) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i+=4) {
                  //  Q[d][i] = 0;
                  // window down image
                  //  for (j=0;j<W;j++) {
            //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
            //  Q[d][i] += P[j][d][i];
            //  }
            //  if (i<W)
            //    C[d][0] += Q[d][i];
                  //}

                  // reg 5 = C
                  pxor_r2r (mm5,mm5);
                  for(i=0;i<W;i++) {
                    // reg 4 = Q
                    pxor_r2r (mm4,mm4);
                        
                    // window down image
                    for (j=0;j<W;j++) {
                      // lbw in 0
                      I1_I2D(I1, I2, i,j,i+d,j);
                      //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
                      //printf("P(%d,%d,%d):",i,j,d);
                      //PRINTREG16(mm1,"P");

                      // Add curr P to Q
                      paddw_r2r (mm1,mm4);
                    }
                    // Store Q
                    movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
                    //printf("Q(%d,%d):",i,d);
                    // PRINTREG16(mm4,"Q");

                    // Add curr Q to C
                    paddw_r2r (mm4,mm5);
                  }
                  // Store C
                  movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
                
                  // compute for i=W
                  // reg 4 = Q
                  pxor_r2r (mm4,mm4);
                  // window down image
                  for (j=0;j<W;j++) {
                    // lbw in 0
                    I1_I2D(I1, I2, W,j,W+d,j);
                
                    movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
                    // Add curr P to Q
                    paddw_r2r (mm1,mm4);
                  }
                  // Store Q
                  movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);                     
                }
                else if (x<0) {
                  // Equivalent C function
                  //C[d][0]=0;
                  //for(i=0;i<=W;i++) {
                  // P_temp = I1_I2(I1, I2, i,y+W,i+d,y+W);
                  // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
                  // P[y%W][d][i] = P_temp;
                  // if (i<W)
                  //   C[d][0] += Q[d][i];
                  //}
                
                  // reg 5 = C
                  pxor_r2r (mm5,mm5);
                  for(i=0;i<W;i++) {
                    // reg 4 = Q
                    movq_m2r (((MM_U_16 *)Q->p[i].m)[d],mm4);
                    // Sub prev P from Q
                    psubw_m2r (((MM_U_16 *)P[ymodW]->p[i].m)[d],mm4);
                    // Add curr P to Q
                    // lbw in 0
                    I1_I2D(I1, I2, i,y+W,i+d,y+W);
                    //PRINTREG16(mm1,"mmx1");
                    // P_temp is reg 1
                    paddw_r2r (mm1,mm4);
                    // Store Q
                    movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
                    // Store P
                    movq_r2m (mm1,((MM_U_16 *)P[ymodW]->p[i].m)[d]);
                    // Add curr Q to C
                    paddw_r2r (mm4,mm5);
                  }
                  // Store C[d][0]
                  movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
                  //PRINTREG16(mm5,"C");
                
                  // Do for i=W
                  // reg 4 = Q
                  movq_m2r (((MM_U_16 *)Q->p[W].m)[d],mm4);
                  // Sub prev P from Q
                  psubw_m2r (((MM_U_16 *)P[ymodW]->p[W].m)[d],mm4);
                  // Add curr P to Q
                  // lbw in 0
                  I1_I2D(I1, I2, W,y+W,W+d,y+W);
                  //PRINTREG16(mm1,"mmx1");
                  // P_temp is reg 1
                  paddw_r2r (mm1,mm4);
                  // Store Q
                  movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
                  // Store P
                  movq_r2m (mm1,((MM_U_16 *)P[ymodW]->p[W].m)[d]);
                }
                else {
                  if (y<0) {
                    // Equivalent C function
                    //Q[x+W][d] = 0;
                    //for (j=0;j<W;j++) {
                    //  P[x+W][j][d]= I1_I2(I1, I2, x+W,j,x+W+d,j);
                    //  Q[x+W][d] += P[x+W][j][d];
                    //}
                        
                    // reg 4 = Q
                    pxor_r2r (mm4,mm4);
                        
                    // window down image
                    for (j=0;j<W;j++) {
                      // lbw in 0
                      I1_I2D(I1, I2,x+W,j,x+W+d,j);
                      //PRINTREG16(mm1,"mmx1");
                      movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
                      // Add curr P to Q
                      paddw_r2r (mm1,mm4);
                    }
                    // Store Q
                    movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
                                        
                  }
                  else {
                    // Equivalent C function
                    // P is [N][W][D] the W dimension is a wrap around buffer.
                    //P_temp = I1_I2(I1, I2, x+W,y+W,x+W+d,y+W);
                    //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
                    //P[x+W][y%W][d] = P_temp;
                        
                    // reg 4 = Q
                    //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
                    movq_m2r (*pQWd,mm4);

                    // subtract prev P
                    //psubw_m2r (((MM_U_16 *)P[y%W]->p[x+W].m)[d],mm4);
                    psubw_m2r (*pPWd,mm4);

                    // Add curr P to Q
                    // lbw in 0
                    //I1_I2D(I1, I2,x+W,y+W,x+W+d,y+W);
                    I1_I2Dptr(pI1, pI2);
                    //PRINTREG16(mm1,"mmx1");
                    // P_temp is reg 1
                    paddw_r2r (mm1,mm4);
                        
                    // Store Q
                    //movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
                    movq_r2m (mm4,*pQWd);

                    // Store P
                    //movq_r2m (mm1,((MM_U_16 *)P[y%W]->p[x+W].m)[d]);
                    movq_r2m (mm1,*pPWd);

                  }
                  // Equivalent C function
                  //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
                        
                  // reg 5 = C
                  //movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);
                  movq_m2r (*pCd,mm5);

                        
                  // Sub curr Q[x][d] from C
                  //psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);
                  psubw_m2r (*pQd,mm5);

                  //PRINTREG16(mm5,"C-P");
                        
                  // Add Q[x+W][d] to C
                  //movq_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm4);
                  //movq_m2r (*pQWd,mm4);
                  //paddw_r2r (mm4,mm5);
                  paddw_m2r (*pQWd,mm5);
                  //PRINTREG16(mm5,"C-P+P");
                        
                  // Store C[x+1][d]
                  //movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);
                  movq_r2m (mm5,*pC1d);
                        
                        
                  pPWd += 4;
                  pQd += 4;
                  pQWd += 4;
                  pCd += 4;
                  pC1d += 4;
                  pI2 += 4;
                        
                }

                // ---------------------------------------------
                // find minimum C Left->Right for C[x+1][d]
                // ---------------------------------------------
                // first create & store 0x0001000100010001 in 6
                // for future reference
                pxor_r2r(mm6,mm6);
                pcmpeqw_r2r(mm1,mm1);
                psubw_r2r(mm1,mm6);
                                

                // reg 5 = C
                //movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
                movq_r2r(mm5,mm2);
                
                //reg 4 = minimum C
                movq_m2r(min_lr,mm4);
                movq_r2r(mm4,mm0);
                                
                //fill 1 with 0x80008000s (sign bit)
                movq_r2r(mm6,mm1);
                psllw_i2r(15,mm1);

                paddw_r2r(mm1,mm0);
                paddw_r2r(mm1,mm2);
                //PRINTREG16(mm0,"min_lr");
                //PRINTREG16(mm2,"C");

                // reg 0 becomes mask
                pcmpgtw_r2r(mm2,mm0);
                movq_r2r(mm0,mm1);
                
                //PRINTREG16(mm2,"mask");
        
                // reg 0 is C values smaller than minimum
                movq_r2r(mm5,mm2);
                pand_r2r(mm0,mm2);
                
                // reg 4 is preserved minimum
                pandn_r2r(mm4,mm1);
                
                // reg 1 is new minimum
                por_r2r(mm2,mm1);
                movq_r2m(mm1,min_lr);
                //PRINTREG16(mm6,"new_min_lr");
                
                // reg 4 is d values for minimum
                movq_m2r(min_d_lr,mm4);
                // reg 1 is preserved d values
                movq_r2r(mm0,mm5);
                pandn_r2r(mm4,mm5);
                
                // reg 3 = current d value
                // increase current d values by 4
                movq_m2r(curr_d_lr,mm3);
                movq_r2r(mm3,mm2);
        

                // reg 0 = current d value for updated minimum
                pand_r2r(mm0,mm2);
        
                // reg 1 = new d's for minimum values
                por_r2r(mm5,mm2);
                movq_r2m(mm2,min_d_lr);
                //PRINTREG16(mm1,"new_min_d_lr");

                // increase current d's
                // fill with 0x0004000400040004
                psllw_i2r(2,mm6);
                paddw_r2r(mm6,mm3);

                //paddw_m2r(plus_four,mm3);
                movq_r2m(mm3,curr_d_lr);
            }// for d

            // first create & store 0x0001000100010001 in 6
            // for future reference
            //pxor_r2r(mm6,mm6);
            //pcmpeqw_r2r(mm0,mm0);
            //psubw_r2r(mm0,mm6);
            // ---------------------------------------------
            // find overall minimum C Left->Right for C[x+1][d]
            // --------------------------------------------

            //movq_m2r(min_lr,mm1);
            //movq_m2r(min_d_lr,mm2);

            //fill 0 with 0x80008000s (sign bit)
            //movq_r2r(mm6,mm0);
            psllw_i2r(13,mm6);
            //PRINTREG16(mm6,"0x80008000");

            // remove sign
            paddw_r2r(mm6,mm1);

            movq_r2r(mm1,mm3);
            movq_r2r(mm2,mm4);

            // once
            psrlq_i2r(16,mm3);
            psrlq_i2r(16,mm4);
            pminsw_r2r(mm1,mm3);

            movq_r2r(mm1,mm0);
            pcmpgtw_r2r(mm3,mm0);

            pand_r2r(mm0,mm4);
            pandn_r2r(mm2,mm0);
            por_r2r(mm0,mm4);

            // twice
            psrlq_i2r(16,mm3);
            psrlq_i2r(16,mm4);
            pminsw_r2r(mm1,mm3);

            movq_r2r(mm1,mm0);
            pcmpgtw_r2r(mm3,mm0);

            pand_r2r(mm0,mm4);
            pandn_r2r(mm2,mm0);
            por_r2r(mm0,mm4);


            // three times
            psrlq_i2r(16,mm3);
            psrlq_i2r(16,mm4);
            pminsw_r2r(mm1,mm3);

            movq_r2r(mm1,mm0);
            pcmpgtw_r2r(mm3,mm0);

            pand_r2r(mm0,mm4);
            pandn_r2r(mm2,mm0);
            por_r2r(mm0,mm4);

            //movq_m2r(min_d_lr,mm0);

            movq_r2m(mm4,min_d_lr);
            //movq_r2m(mm3,min_lr);
            //PRINTREG16(mm1,"min_lr");
            //PRINTREG16(mm3,"overall_min_lr");
            //PRINTREG16(mm0,"min_d_lr");
            //PRINTREG16(mm4,"overall_min_d_lr");

//            overall_min_d_lr = min_d_lr.uw[0];
//            O[x+1] = min_d_lr.uw[0];
            *pO = min_d_lr.uw[0];
            pO++;
            //overall_min_lr = min_lr.uw[0]; // wrong because of sign eradication


            //((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+1] = overall_min_d_lr;
//            ((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+1] = min_d_lr.uw[0];
          } // for x


          pO=O;
          for(x=-1;x<X-W-32-1;x++) {

//           overall_min_d_lr = ((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+1];
            // ---------------------------------------------
            // subpixel interpolation
            // weighted average of best match and each neighbour
            // ---------------------------------------------
// if adjacent disparities correlation result that close to the minimum we have an unreliable match
// 0.49 * shift factor
	    //#define SUBPIX_TOL 2000
//            overall_min_d_lr = O[x+1];
            if ((*pO)&&(*pO<32)) {
              overall_min_lrm1 = ((MM_U_16 *)C->p[x+1].m)[*pO-1];
              overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[*pO];
              overall_min_lrp1 = ((MM_U_16 *)C->p[x+1].m)[*pO+1];

              if ((overall_min_lr<<12)<(SUBPIX_TOL*(overall_min_lrp1 + overall_min_lrm1))) {
                if (overall_min_lrm1<overall_min_lrp1) {
                  subpix_d = -(overall_min_lr<<3)/(1+overall_min_lrm1+ overall_min_lr);
                }
                else {
                  subpix_d = (overall_min_lr<<3)/(1+overall_min_lrp1 + overall_min_lr);
                }
                subpix_d += (*pO)<<3;
                //subpix_d = (subpix_d>>3) + (overall_min_d_lr<<3);
                ((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+1] = subpix_d;
              }
              else {
                subpix_d = 0;
                *pO = 0x0000;
              }
            }
            else {
              subpix_d = *pO<<3;
              ((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+1] = subpix_d;
            } // if ((overall

            pO++;
          } // for x

          pO=O;
          for(x=-1;x<X-W-32-1;x++) {
            // ---------------------------------------------
            // find overall minimum C Right->Left for C[x][d]
            // check for consistency
            // ---------------------------------------------
            if (!(y<0)&&*pO) {
              min_rl = ((MM_U_16 *)C->p[x+1].m)[*pO];
                  idx_rl = x+1+*pO;
              for (d=0;d<32;d++) {
                  if ((idx_rl>-1)&&(idx_rl<X)&&((MM_U_16 *)C->p[idx_rl].m)[d]&&!(min_rl < ((MM_U_16 *)C->p[idx_rl].m)[d])) {
                    min_rl = ((MM_U_16 *)C->p[idx_rl].m)[d];
                    min_d_rl = d;
                    //break;
                  } // if
                  idx_rl--;
              } // for
                 if (abs(min_d_rl - *pO)>1) {
                ((MM_U_8 *)Out->p[y+w_2+1].m)[x+w_2+1] = 0x00;
                //*pO = 0x0000;
                  }

            } // if (!y
            pO++;
          }// for x

  }// for y
  emms();
  return 0;
}




























#if 0

/*
            if ((overall_min_d_lr>0)&&(overall_min_d_lr<D)) {
              overall_min_lrm1 = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1];
              overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr];
              overall_min_lrp1 = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1];

              if ((overall_min_lr<<12)<(SUBPIX_TOL*(overall_min_lrp1 + overall_min_lrm1))) {
                if (overall_min_lrm1<overall_min_lrp1) {
                  subpix_d = -(overall_min_lr<<shift)/(1+overall_min_lrm1+ overall_min_lr);
                }
                else {
                  subpix_d = (overall_min_lr<<shift)/(1+overall_min_lrp1 + overall_min_lr);
                }
                subpix_d = subpix_d>>shift;
                subpix_d += overall_min_d_lr<<shift;
                //subpix_d = (subpix_d>>shift) + (overall_min_d_lr<<shift);
              }
              else {
                subpix_d = 0;
              }
            }
            else {
              subpix_d = overall_min_d_lr<<shift;
            } // if ((overall
            ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = subpix_d;
          } // for x
*/

      // ---------------------------------------------
      // subpixel interpolation
      // weighted average of best match and each neighbour
      // ---------------------------------------------
      //emms();
      if ((overall_min_d_lr>0)&&(overall_min_d_lr<D)) {
        overall_min_lr = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr];
        overall_min_lrm1 = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr-1];
        overall_min_lrp1 = ((MM_U_16 *)C->p[x+1].m)[overall_min_d_lr+1];
//        if (overall_min_lr<((SUBPIX_TOL*(overall_min_lrm1+overall_min_lrp1))>>8)) {
          if (overall_min_lrm1<overall_min_lrp1) {
            subpix_d = -(overall_min_lr<<12)/(1+overall_min_lrm1+ overall_min_lr);
          }
          else {
            subpix_d = (overall_min_lr<<12)/(1+overall_min_lrp1 + overall_min_lr);
          }
//#define SUBPIX_TOL 127
#define SUBPIX_TOL 2044

           if (abs(subpix_d)<SUBPIX_TOL) {
#ifndef DEPTH64
             subpix_d = subpix_d>>9;
             subpix_d += overall_min_d_lr<<3;
#else
             subpix_d = subpix_d>>10;
             subpix_d += overall_min_d_lr<<2;
#endif
           }
           else {
                  subpix_d = 0;
           }
           ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = subpix_d;

     }
     else {
#ifndef DEPTH64
        ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = overall_min_d_lr<<3;
#else
        ((MM_U_8 *)Out->p[y+w_2+1].m)[x+D_OFF+w_2+1] = overall_min_d_lr<<2;
#endif
     }



int MMXRecursive_Depthmap_SAD6(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCache)
{  
  const short W = 16;

  short Y,X;

  register short i, j;
  register short d,e,x,y;
  
  short w_2 = (W/2);
  short d_w_2_1 = D_OFF+w_2+1;

  unsigned short overall_min_lr;
  unsigned short overall_min_rl;

  short idx_rl; 
  unsigned short min_rl, prev;

  unsigned char overall_min_d_rl;
  unsigned char overall_min_d_lr;
  unsigned char d_rl; 
  unsigned char d_lr; 
  unsigned char min_d_rl;

  int subpix_d,subpix_min,subpix_denom;

  static int firsttime =1;
  static MMXMatrix **P;
  static MMXMatrix *Q;
  static MMXMatrix *C1;
  static MMXMatrix *C2;
  MMXMatrix *C;
  MMXMatrix *C_prev;
  sse_t min_lr;
  sse_t curr_d_lr;
  sse_t min_d_lr;

  
  X = I1->cols;
  Y = I1->rows;


  if (firsttime) {
    Q = MMXMatrixAlloc(MMT_F_32, (D/2), X);
    C1 = MMXMatrixAlloc(MMT_F_32, (D/2), X);
    C2 = MMXMatrixAlloc(MMT_F_32, (D/2), X);
    P = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*W); 
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_F_32, (D/2), X);

    firsttime = 0;
  }

  emms();

  C = C2;
  C_prev = C1;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }

    for(x=-1;x<X-W-D_OFF-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);


      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(xmm0,xmm0);
      movdqu_r2m(xmm0,min_lr);

      curr_d_lr.uw[0] = 0;
      curr_d_lr.uw[1] = 1;
      curr_d_lr.uw[2] = 2;
      curr_d_lr.uw[3] = 3;
      curr_d_lr.uw[4] = 4;
      curr_d_lr.uw[5] = 5;
      curr_d_lr.uw[6] = 6;
      curr_d_lr.uw[7] = 7;

      // clear registers
      //pxor_r2r (xmm3,xmm3);
      // pxor_r2r (xmm4,xmm4); 

      pxor_r2r (xmm7,xmm7); 

      for (d=0;d<D-7;d+=8) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i+=4) {
            //  Q[d][i] = 0;
            // window down image
            //  for (j=0;j<W;j++) {
              //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
              //  Q[d][i] += P[j][d][i];
              //  }
              //  if (i<W)
              //    C[d][0] += Q[d][i];
              //}

              // reg 5 = C
              pxor_r2r (xmm5,xmm5);
              for(i=0;i<W;i++) {
                // reg 4 = Q
                pxor_r2r (xmm4,xmm4);

                // window down image
                for (j=0;j<W;j++) {
                  // lbw in 0
                  I1_I2DXMM(I1, I2, i+D_OFF,j,i+d,j);
                  //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
                  //printf("P(%d,%d,%d):",i,j,d);
                  //PRINTREG16(xmm1,"P");


                  // Add curr P to Q
                  paddw_r2r (xmm1,xmm4);
                }
                // Store Q
                movdqa_r2m (xmm4,((MM_U_16 *)Q->p[i].s)[d]);
                //printf("Q(%d,%d):",i,d);
                // PRINTREG16(xmm4,"Q");

                // Add curr Q to C
                paddw_r2r (xmm4,xmm5);
              }
              // Store C
              movdqa_r2m (xmm5,((MM_U_16 *)C->p[0].s)[d]);

              // compute for i=W
              // reg 4 = Q
              pxor_r2r (xmm4,xmm4);
              // window down image
              for (j=0;j<W;j++) {
                // lbw in 0
                I1_I2DXMM(I1, I2, W+D_OFF,j,W+d,j);

                movdqa_r2m (xmm1,((MM_U_16 *)P[j]->p[W].s)[d]);
                // Add curr P to Q
                paddw_r2r (xmm1,xmm4);
              }
              // Store Q
              movdqa_r2m (xmm4,((MM_U_16 *)Q->p[W].s)[d]);


            }
            else if (x<0) {

              // Equivalent C function
              //C[d][0]=0;
              //for(i=0;i<=W;i++) {
                // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
                // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
                // P[y%W][d][i] = P_temp;
                // if (i<W)
                //   C[d][0] += Q[d][i];
                //}

                // reg 5 = C
                pxor_r2r (xmm5,xmm5);
                for(i=0;i<W;i++) {
                  // reg 4 = Q
                  movdqa_m2r (((MM_U_16 *)Q->p[i].s)[d],xmm4);
                  // Sub prev P from Q
                  psubw_m2r (((MM_U_16 *)P[y%W]->p[i].s)[d],xmm4);
                  // Add curr P to Q
                  // lbw in 0
                  I1_I2DXMM(I1, I2, i+D_OFF,y+W,i+d,y+W);
                  //PRINTREG16(mm1,"mmx1");
                  // P_temp is reg 1
                  paddw_r2r (xmm1,xmm4);
                  // Store Q
                  movdqa_r2m (xmm4,((MM_U_16 *)Q->p[i].s)[d]);
                  // Store P
                  movdqa_r2m (xmm1,((MM_U_16 *)P[y%W]->p[i].s)[d]);
                  // Add curr Q to C
                  paddw_r2r (xmm4,xmm5);
                }
                // Store C[d][0]
                movdqa_r2m (xmm5,((MM_U_16 *)C->p[0].s)[d]);
                //PRINTREG16(xmm5,"C");


                // Do for i=W
                // reg 4 = Q
                movdqa_m2r (((MM_U_16 *)Q->p[W].s)[d],xmm4);
                // Sub prev P from Q
                psubw_m2r (((MM_U_16 *)P[y%W]->p[W].s)[d],xmm4);
                // Add curr P to Q
                // lbw in 0
                I1_I2DXMM(I1, I2, W+D_OFF,y+W,W+d,y+W);
                //PRINTREG16(xmm1,"mmx1");
                // P_temp is reg 1
                paddw_r2r (xmm1,xmm4);
                // Store Q
                movdqa_r2m (xmm4,((MM_U_16 *)Q->p[W].s)[d]);
                // Store P
                movdqa_r2m (xmm1,((MM_U_16 *)P[y%W]->p[W].s)[d]);
              }
              else
              {
                if (y<0) {
                  // Equivalent C function
                  //Q[x+W][d] = 0;
                  //for (j=0;j<W;j++) {
                    //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
                    //  Q[x+W][d] += P[x+W][j][d];
                    //}

                    // reg 4 = Q
                    pxor_r2r (xmm4,xmm4);

                    // window down image
                    for (j=0;j<W;j++) {
                      // lbw in 0
                      I1_I2DXMM(I1, I2,x+W+D_OFF,j,x+W+d,j);
                      //PRINTREG16(xmm1,"mmx1");
                      movdqa_r2m (xmm1,((MM_U_16 *)P[j]->p[x+W].s)[d]);
                      // Add curr P to Q
                      paddw_r2r (xmm1,xmm4);
                    }
                    // Store Q
                    movdqa_r2m (xmm4,((MM_U_16 *)Q->p[x+W].s)[d]);

                  }
                  else
                  {
                    // Equivalent C function
                    // P is [N][W][D] the W dimension is a wrap around buffer.
                    //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
                    //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
                    //P[x+W][y%W][d] = P_temp;

                    // reg 4 = Q
                    movdqa_m2r (((MM_U_16 *)Q->p[x+W].s)[d],xmm4);
                    // subtract prev P
                    psubw_m2r (((MM_U_16 *)P[y%W]->p[x+W].s)[d],xmm4);
                    // Add curr P to Q
                    // lbw in 0
                    I1_I2DXMM(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
                    //PRINTREG16(xmm1,"mmx1");
                    // P_temp is reg 1
                    paddw_r2r (xmm1,xmm4);

                    // Store Q
                    movdqa_r2m (xmm4,((MM_U_16 *)Q->p[x+W].s)[d]);

                    // Store P
                    movdqa_r2m (xmm1,((MM_U_16 *)P[y%W]->p[x+W].s)[d]);

                  }
                  // Equivalent C function
                  //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];

                  // reg 5 = C
                  movdqa_m2r (((MM_U_16 *)C->p[x].s)[d],xmm5);
                  //printf("(%d,%d,%d):",x,y,d);
                  //PRINTREG16(mm4,"Q");
                  //PRINTREG16(xmm5,"C");

                  // Sub curr Q[x][d] from C
                  psubw_m2r (((MM_U_16 *)Q->p[x].s)[d],xmm5);
                  //PRINTREG16(xmm5,"C-P");

                  // Add Q[x+W][d] to C
                  movdqa_m2r (((MM_U_16 *)Q->p[x+W].s)[d],xmm4);
                  paddw_r2r (xmm4,xmm5);
                  //PRINTREG16(xmm5,"C-P+P");

                  // Store C[x+1][d]
                  movdqa_r2m (xmm5,((MM_U_16 *)C->p[x+1].s)[d]);

                }

                // ---------------------------------------------
                // find minimum C Left->Right for C[x+1][d]
                // ---------------------------------------------
                // first create & store 0x0001000100010001 in 6
                // for future reference
                pxor_r2r(xmm6,xmm6);
                pcmpeqw_r2r(xmm1,xmm1);
                psubw_r2r(xmm1,xmm6);


                // reg 5 = C
                //movdqa_m2r (((MM_U_16 *)C->p[x+1].s)[d],xmm5);
                movdqa_r2r(xmm5,xmm0);

                //reg 4 = minimum C
                movdqu_m2r(min_lr,xmm4);
                movdqa_r2r(xmm4,xmm2);

                //fill 1 with 0x80008000s (sign bit)
                movdqa_r2r(xmm6,xmm1);
                psllw_i2r(15,xmm1);

                paddw_r2r(xmm1,xmm2);
                paddw_r2r(xmm1,xmm0);
                //PRINTREG16(xmm2,"unsigned min_lr");
                //PRINTREG16XMM(xmm0,"unsigned C");

                // reg 2 becomes mask
                pcmpgtw_r2r(xmm0,xmm2);
                movdqa_r2r(xmm2,xmm1);

                //PRINTREG16(xmm2,"mask");

                // reg 0 is C values smaller than minimum
                movdqa_r2r(xmm5,xmm0);
                pand_r2r(xmm2,xmm0);

                // reg 4 is preserved minimum
                pandn_r2r(xmm4,xmm1);

                // reg 1 is new minimum
                por_r2r(xmm0,xmm1);
                movdqu_r2m(xmm1,min_lr);
                //PRINTREG16XMM(xmm1,"new_min_lr");

                // reg 4 is d values for minimum
                movdqu_m2r(min_d_lr,xmm4);
                // reg 1 is preserved d values
                movdqa_r2r(xmm2,xmm1);
                pandn_r2r(xmm4,xmm1);

                // reg 3 = current d value
                // increase current d values by 4
                movdqu_m2r(curr_d_lr,xmm3);
                movdqa_r2r(xmm3,xmm0);


                // reg 0 = current d value for updated minimum
                pand_r2r(xmm2,xmm0);

                // reg 1 = new d's for minimum values
                por_r2r(xmm0,xmm1);
                movdqu_r2m(xmm1,min_d_lr);
                //PRINTREG16XMM(xmm1,"new_min_d_lr");

                // increase current d's
                // fill with 0x0008000800080008
                psllw_i2r(3,xmm6);
                paddw_r2r(xmm6,xmm3);
                //PRINTREG16XMM(xmm6,"8s");

                //paddw_m2r(plus_four,xmm3);
                movdqu_r2m(xmm3,curr_d_lr);


                // ---------------------------------------------
                // find minimum C Right->Left for C[x][d]
                // ---------------------------------------------


              }// for d


              // ---------------------------------------------
              // find overall minimum C Left->Right for C[x+1][d]
              // ---------------------------------------------

              overall_min_lr = min_lr.uw[0];
              overall_min_d_lr = min_d_lr.uw[0];
              if (overall_min_lr > min_lr.uw[1]) {
                overall_min_lr = min_lr.uw[1];
                overall_min_d_lr = min_d_lr.uw[1];
              }
              if (overall_min_lr > min_lr.uw[2]) {
                overall_min_lr = min_lr.uw[2];
                overall_min_d_lr = min_d_lr.uw[2];
              }
              if (overall_min_lr > min_lr.uw[3]) {
                overall_min_lr = min_lr.uw[3];
                overall_min_d_lr = min_d_lr.uw[3];
              }
              if (overall_min_lr > min_lr.uw[4]) {
                overall_min_lr = min_lr.uw[4];
                overall_min_d_lr = min_d_lr.uw[4];
              }
              if (overall_min_lr > min_lr.uw[5]) {
                overall_min_lr = min_lr.uw[5];
                overall_min_d_lr = min_d_lr.uw[5];
              }
              if (overall_min_lr > min_lr.uw[6]) {
                overall_min_lr = min_lr.uw[6];
                overall_min_d_lr = min_d_lr.uw[6];
              }
              if (overall_min_lr > min_lr.uw[7]) {
                overall_min_lr = min_lr.uw[7];
                overall_min_d_lr = min_d_lr.uw[7];
              }
              ((MM_U_8 *)Out->p[y+w_2+1].s)[x+D_OFF+w_2+1] = overall_min_d_lr<<3;

              // ---------------------------------------------
              // subpixel interpolation
              // weighted average of best match and each neighbour
              // ---------------------------------------------
              //emms();
              if ((overall_min_d_lr>0)&&(overall_min_d_lr<D)) {
                subpix_denom = ((MM_U_16 *)C->p[x+1].s)[overall_min_d_lr-1];
                subpix_denom += ((MM_U_16 *)C->p[x+1].s)[overall_min_d_lr+1];
                        subpix_d = ((MM_U_16 *)C->p[x+1].s)[overall_min_d_lr-1];
                subpix_d -=  ((MM_U_16 *)C->p[x+1].s)[overall_min_d_lr+1];
                subpix_d = subpix_d <<3;
                //fprintf(stderr,"%d %d %d ",overall_min_d_lr,subpix_d,subpix_denom);
                if (subpix_denom) {
                  subpix_d /= subpix_denom;
                }
                else
                subpix_d = 0;
                //fprintf(stderr," %d  %d\n ",subpix_d,subpix_d/8);
                subpix_d +=  (overall_min_d_lr<<3);

                ((MM_U_8 *)Out->p[y+w_2+1].s)[x+D_OFF+w_2+1] = subpix_d;
              }
              else
              ((MM_U_8 *)Out->p[y+w_2+1].s)[x+D_OFF+w_2+1] = (overall_min_d_lr<<3);

              // ---------------------------------------------
              // find overall minimum C Right->Left for C[x][d]
              // check for consistency
              // ---------------------------------------------


              if (!(y<0)) {
                d_rl = (((MM_U_8 *)Out->p[y+w_2].s)[x+D_OFF+w_2+1])>>3;
                min_rl = ((MM_U_16 *)C_prev->p[x+1].s)[d_rl];

                //printf("min&d_rl:%u,%u\n",min_rl,d_rl);
                idx_rl = x+1+d_rl;
                for (d=0;d<D;d++) {
                  if ((idx_rl>-1)&&(idx_rl<X)&&(min_rl >= ((MM_U_16 *)C_prev->p[idx_rl].s)[d])) {
                    min_rl = ((MM_U_16 *)C_prev->p[idx_rl].s)[d];
                    min_d_rl = d;
                    //printf("better&d_rl:%u,%d\n",((MM_U_16 *)C_prev->p[idx_rl].s)[d],d);
                    //((MM_U_8 *)Out->p[y+w_2].s)[x+d_w_2_1] = 0x00;
                    //break;
                  }
                  idx_rl--;
                }
                if (abs(min_d_rl - d_rl)>2) {
                  //((MM_U_8 *)Out->p[y+w_2].s)[x+D_OFF+w_2+1] = 0x00;
                  ((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1] = prev;

                }
                else {
                  //      ((MM_U_8 *)Out->p[y+w_2].s)[x+D_OFF+w_2+1] = 0xff - ((MM_U_8 *)Out->p[y+w_2].s)[x+D_OFF+w_2+1];
                  //printf("better d_rl:%d %d\n",min_d_rl, d_rl);
                  prev = ((MM_U_8 *)Out->p[y+w_2].m)[x+d_w_2_1];

                }
              }

            }// for x
          }// for y

          emms();
        }

#endif




//----------------------------------------------------------------
//----------------------------------------------------------------
//----------------------------------------------------------------
//----------------------------------------------------------------
int Recursive_Depthmap_SAD_U8(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{
  const int W = 16;


  register int i, j;
  int X,Y, min=0, min_d=0, P_temp;

  register int d,x,y;
  

  unsigned char ***P;
  unsigned char **Q;
  unsigned char **C;

  X = I1->cols;
  Y = I1->rows;
  
  P = (unsigned char ***)Alloc(sizeof(unsigned char **)*X);
  Q = (unsigned char **)Alloc(sizeof(unsigned char *)*X);
  C = (unsigned char **)Alloc(sizeof(unsigned char *)*X);
  for (x=0;x<X;x++) {
    Q[x] = (unsigned char *)Alloc(sizeof(unsigned char)*D);
    C[x] = (unsigned char *)Alloc(sizeof(unsigned char)*D);
    P[x] = (unsigned char **)Alloc(sizeof(unsigned char *)*W);
    for (i=0;i<W;i++) {
      P[x][i] = (unsigned char *)Alloc(sizeof(unsigned char)*D);
    }
  }     

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    
    for(x=-1;x<X-D-1-W;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      for (d=0;d<D;d++) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {
          
          C[0][d]=0;
          // window across image
          for(i=0;i<W;i++) {
            Q[i][d] = 0;
            // window down image
            for (j=0;j<W;j++) {
              P[i][j][d]= abs((((MM_U_8 *)I1->p[j].m)[i+D_OFF]>>4) - (((MM_U_8 *)I2->p[j].m)[i+d]>>4));
              Q[i][d] += P[i][j][d];
              //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
              //printf("P(%d,%d,%d):%d\n",i,j,d,P[i][j][d]);
              
            } 
            //printf("Q(%d,%d):%d",i,d,Q[i][d]);
            
            if (i<W)
              C[0][d] += Q[i][d];
          }
        }
        else if (x<0) {
          
          C[0][d]=0;
          for(i=0;i<W;i++) {
            P_temp = abs((((MM_U_8 *)I1->p[y+W].m)[i+D_OFF]>>4) - (((MM_U_8 *)I2->p[y+W].m)[i+d]>>4));
            
            Q[i][d] = Q[i][d] - P[i][y%W][d] + P_temp ;
            P[i][y%W][d] = P_temp;
            if (i<W)
              C[0][d] += Q[i][d];
          }
        }
        else
          {
            if (y<0) {
              
              Q[x+W][d] = 0;
              for (j=0;j<W;j++) {
                P[x+W][j][d]= abs((((MM_U_8 *)I1->p[j].m)[x+W+D_OFF]>>4) - (((MM_U_8 *)I2->p[j].m)[x+W+d]>>4));
                Q[x+W][d] += P[x+W][j][d];
              }
            }  
            else
              {
                // P is [X][W][D] the W dimension is a wrap around buffer.
                
                P_temp = abs((((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF]>>4) - (((MM_U_8 *)I2->p[y+W].m)[x+W+d]>>4));
                Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
                P[x+W][y%W][d] = P_temp;
              }
            
            C[x+1][d] = C[x][d] - Q[x][d] + Q[x+W][d];
          }
        
        
        //printf("%d,%d,%d: %d\n",x,y,d, C[x+1][d]);
        
        if ((d==0)||(C[x+1][d] < min)) {
          min = C[x+1][d];
          min_d = d;
        }
      }// for d
      ((MM_U_8 *)Out->p[y+1].m)[x+1]= min_d; 
    }// for x
  }// for y
  
  /* Deallocate memory */
  for (x=0;x<X;x++) {
    for (i=0;i<W;i++) {
      free(P[x][i]);
    }
    free(Q[x]);
    free(C[x]);
    free(P[x]);
  } 
  free(P);
  free(Q);
  free(C);
  return 0;
}


//----------------------------------------------------------------
//----------------------------------------------------------------
//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD_U8(const MMXMatrix *I1,const MMXMatrix *I2, MMXMatrix  *Out,const int D,const int D_OFF, MMXMatricesCachePtr* vcache)
{
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  //const short W = 16;
  const short W = 8;

  short Y,X;

  register short i, j;
  register short d,x,y;

  short w_2 = (W/2);
  short d_w_2_1 = D_OFF+w_2+1;




  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C1;
  MMXMatrix *C2;
  MMXMatrix *C;
  MMXMatrix *C_prev;
  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  unsigned char *pI1=NULL, *pI2=NULL;
  unsigned char *pPWd=NULL, *pQd=NULL, *pQWd=NULL, *pCd=NULL, *pC1d=NULL;

  X = I1->cols;
  Y = I1->rows;


  if (*cache==0) {
    i=0;
    *cache = MMXMatricesCacheAlloc(3+W,0,0);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_8, D, X);
    (*cache)->M[i++] = C1 = MMXMatrixAlloc(MMT_U_8, D, X);
    (*cache)->M[i++] = C2 = MMXMatrixAlloc(MMT_U_8, D, X);
    P = (*cache)->M + 3;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_8, D, X);

  }
  else { 

    P = (*cache)->M + 3;
    Q =  (*cache)->M[0];
    C1 = (*cache)->M[1]; 
    C2 = (*cache)->M[2]; 
  }

  emms();

  C = C2;
  C_prev = C1;

  // make mask 0xf0f0f0f0f0f0
  pcmpeqw_r2r(mm7,mm7);
  psllw_i2r(8,mm7);
  psrlw_i2r(12,mm7);
  psllw_i2r(4,mm7);
  packuswb_r2r(mm7,mm7);
  //PRINTREG8(mm7,"0xf0f0f0:");

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }

    for(x=-1;x<X-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);


      // fill min with 111111111111111s (max number)
      pcmpeqb_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      //curr_d_lr.ub[0] = 0;
      //curr_d_lr.ub[1] = 1;
      //curr_d_lr.ub[2] = 2;
      //curr_d_lr.ub[3] = 3;
      //curr_d_lr.ub[4] = 4;
      //curr_d_lr.ub[5] = 5;
      //curr_d_lr.ub[6] = 6;
      //curr_d_lr.ub[7] = 7;
      //movq_m2r(curr_d_lr,mm0);
      //PRINTREG8(mm0,"curr_d_lr1:");

      curr_d_lr.ud[0] = 0x03020100;
      curr_d_lr.ud[1] = 0x07060504;
      //movq_m2r(curr_d_lr,mm0);
      //PRINTREG8(mm0,"curr_d_lr2:");
 
      // clear registers
      //pxor_r2r (mm3,mm3);
      //pxor_r2r (mm4,mm4);
      //pxor_r2r (mm7,mm7);
      
      if (x>=0) {
        pCd = &(((MM_U_8 *)C->p[x].m)[0]);
        pC1d = &(((MM_U_8 *)C->p[x+1].m)[0]);
        pQd  = &(((MM_U_8 *)Q->p[x].m)[0]);
        pQWd  = &(((MM_U_8 *)Q->p[x+W].m)[0]);
        if (y>=0) {
          pPWd = &(((MM_U_8 *)P[y%W]->p[x+W].m)[0]);
        }
        pI1  = &(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF]);
        pI2  = &(((MM_U_8 *)I2->p[y+W].m)[x+W]);
      }
      
      for (d=0;d<D-7;d+=8) {
        //fprintf(stderr,"depthmap: d:%d \n",d);
        if ((x<0)&&(y<0)) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i+=4) {
          //  Q[d][i] = 0;
          // window down image
          //  for (j=0;j<W;j++) {
          //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
          //  Q[d][i] += P[j][d][i];
          //  }
          //  if (i<W)
          //    C[d][0] += Q[d][i];
          //}

          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            pxor_r2r (mm4,mm4);

            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2DU8(I1, I2, i+D_OFF,j,i+d,j);
              //printf("I1_I2(%d,%d)(%d,%d):",i+D_OFF,j,i+d,j);
              //printf("P(%d,%d,%d):",i,j,d);
              //PRINTREG16(mm1,"P");

              // Add curr P to Q
              paddusb_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_8 *)Q->p[i].m)[d]);
            //printf("Q(%d,%d):",i,d);
            // PRINTREG16(mm4,"Q");

            // Add curr Q to C
            paddusb_r2r (mm4,mm5);
          }
          // Store C
          movq_r2m (mm5,((MM_U_8 *)C->p[0].m)[d]);

          // compute for i=W
          // reg 4 = Q
          pxor_r2r (mm4,mm4);
          // window down image
          for (j=0;j<W;j++) {
            // lbw in 0
            I1_I2DU8(I1, I2, W+D_OFF,j,W+d,j);

            movq_r2m (mm1,((MM_U_8 *)P[j]->p[W].m)[d]);
            // Add curr P to Q
            paddusb_r2r (mm1,mm4);
          }
          // Store Q
          movq_r2m (mm4,((MM_U_8 *)Q->p[W].m)[d]);

        }
        else if (x<0) {
          // Equivalent C function
          //C[d][0]=0;
          //for(i=0;i<=W;i++) {
          // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
          // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
          // P[y%W][d][i] = P_temp;
          // if (i<W)
          //   C[d][0] += Q[d][i];
          //}

          // reg 5 = C
          pxor_r2r (mm5,mm5);
          for(i=0;i<W;i++) {
            // reg 4 = Q
            movq_m2r (((MM_U_8 *)Q->p[i].m)[d],mm4);
            // Sub prev P from Q
            psubusb_m2r (((MM_U_8 *)P[y%W]->p[i].m)[d],mm4);
            // Add curr P to Q
            // lbw in 0
            I1_I2DU8(I1, I2, i+D_OFF,y+W,i+d,y+W);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddusb_r2r (mm1,mm4);
            // Store Q
            movq_r2m (mm4,((MM_U_8 *)Q->p[i].m)[d]);
            // Store P
            movq_r2m (mm1,((MM_U_8 *)P[y%W]->p[i].m)[d]);
            // Add curr Q to C
            paddusb_r2r (mm4,mm5);
          }
          // Store C[d][0]
          movq_r2m (mm5,((MM_U_8 *)C->p[0].m)[d]);
          //PRINTREG16(mm5,"C");
          
          // Do for i=W
          // reg 4 = Q
          movq_m2r (((MM_U_8 *)Q->p[W].m)[d],mm4);
          // Sub prev P from Q
          psubusb_m2r (((MM_U_8 *)P[y%W]->p[W].m)[d],mm4);
          // Add curr P to Q
          // lbw in 0
          I1_I2DU8(I1, I2, W+D_OFF,y+W,W+d,y+W);
          //PRINTREG16(mm1,"mmx1");
          // P_temp is reg 1
          paddusb_r2r (mm1,mm4);
          // Store Q
          movq_r2m (mm4,((MM_U_8 *)Q->p[W].m)[d]);
          // Store P
          movq_r2m (mm1,((MM_U_8 *)P[y%W]->p[W].m)[d]);
        }
        else {
          if (y<0) {
            // Equivalent C function
            //Q[x+W][d] = 0;
            //for (j=0;j<W;j++) {
            //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
            //  Q[x+W][d] += P[x+W][j][d];
            //}

            // reg 4 = Q
            pxor_r2r (mm4,mm4);

            // window down image
            for (j=0;j<W;j++) {
              // lbw in 0
              I1_I2DU8(I1, I2,x+W+D_OFF,j,x+W+d,j);
              //PRINTREG16(mm1,"mmx1");
              movq_r2m (mm1,((MM_U_8 *)P[j]->p[x+W].m)[d]);
              // Add curr P to Q
              paddw_r2r (mm1,mm4);
            }
            // Store Q
            movq_r2m (mm4,((MM_U_8 *)Q->p[x+W].m)[d]);
          }
          else {
            // Equivalent C function
            // P is [N][W][D] the W dimension is a wrap around buffer.
            //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
            //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
            //P[x+W][y%W][d] = P_temp;

            // reg 4 = Q
            //movq_m2r (((MM_U_8 *)Q->p[x+W].m)[d],mm4);
            movq_m2r (*pQWd,mm4);
            // subtract prev P
            //psubusb_m2r (((MM_U_8 *)P[y%W]->p[x+W].m)[d],mm4);
            psubusb_m2r (*pPWd,mm4);
            // Add curr P to Q
            // lbw in 0
            //I1_I2DU8(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
            I1_I2DU8ptr(pI1, pI2);
            //PRINTREG16(mm1,"mmx1");
            // P_temp is reg 1
            paddusb_r2r (mm1,mm4);
            // Store Q
            //movq_r2m (mm4,((MM_U_8 *)Q->p[x+W].m)[d]);
            movq_r2m (mm4,*pQWd);

            // Store P
            //movq_r2m (mm1,((MM_U_8 *)P[y%W]->p[x+W].m)[d]);
            movq_r2m (mm1,*pPWd);

          }
          // Equivalent C function
          //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
          
          // reg 5 = C
          //movq_m2r (((MM_U_8 *)C->p[x].m)[d],mm5);
          movq_m2r (*pCd,mm5);
          
          // Sub curr Q[x][d] from C
          //psubusb_m2r (((MM_U_8 *)Q->p[x].m)[d],mm5);
          psubusb_m2r (*pQd,mm5);

          // Add Q[x+W][d] to C
          //movq_m2r (((MM_U_8 *)Q->p[x+W].m)[d],mm4);
          movq_m2r (*pQWd,mm4);
          paddusb_r2r (mm4,mm5);

          // Store C[x+1][d]
          //movq_r2m (mm5,((MM_U_8 *)C->p[x+1].m)[d]);
          movq_r2m (mm5,*pC1d);

          pPWd += 8;
          pQd += 8;
          pQWd += 8;
          pCd += 8;
          pC1d += 8;
          pI2 += 8;
        }

        // ---------------------------------------------
        // find minimum C Left->Right for C[x+1][d]
        // ---------------------------------------------

        // reg 5 = C
        //movq_m2r (((MM_U_8 *)C->p[x+1].m)[d],mm5);
        movq_r2r(mm5,mm1);

        pminub_m2r(min_lr,mm1);

        movq_r2r(mm1,mm3);
        movq_r2m(mm1,min_lr);

        // mm3 becomes mask
        pcmpeqb_r2r(mm5,mm3);

        // reg 2 is preserved d values
        movq_r2r(mm3,mm2);
        pandn_m2r(min_d_lr,mm2);

        // reg 3 = current d value
        // increase current d values by 4
        movq_m2r(curr_d_lr,mm5);


        // reg 3 = current d value for updated minimum
        pand_r2r(mm5,mm3);
        
        // reg 2 = new d's for minimum values
        por_r2r(mm3,mm2);
        movq_r2m(mm2,min_d_lr);

        // increase current d's
        // fill with 0x080808080808080
        // first create & store 0x0101010101010101 in 0
        // for future reference
        pxor_r2r(mm0,mm0);
        pcmpeqb_r2r(mm4,mm4);
        psubb_r2r(mm4,mm0);
        psllw_i2r(3,mm0);
        //PRINTREG8(mm0,"0x08080808:");

        paddusb_r2r(mm0,mm5);

        //paddw_m2r(plus_four,mm5);
        movq_r2m(mm5,curr_d_lr);

        // ---------------------------------------------
        // find minimum C Right->Left for C[x][d]
        // ---------------------------------------------
      }// for d


      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]
      // --------------------------------------------
      
      //movq_m2r(min_lr,mm1);
      //movq_m2r(min_d_lr,mm2);
      movq_r2r(mm1,mm3);
      movq_r2r(mm2,mm4);

      // once
      psrlq_i2r(8,mm3);
      psrlq_i2r(8,mm4);
      pminub_r2r(mm1,mm3);
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // twice
      psrlq_i2r(8,mm3);
      psrlq_i2r(8,mm4);
      pminub_r2r(mm1,mm3);
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // thrice
      psrlq_i2r(8,mm3);
      psrlq_i2r(8,mm4);
      pminub_r2r(mm1,mm3);
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // 4
      psrlq_i2r(8,mm3);
      psrlq_i2r(8,mm4);
      pminub_r2r(mm1,mm3);
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // 5
      psrlq_i2r(8,mm3);
      psrlq_i2r(8,mm4);
      pminub_r2r(mm1,mm3);
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // 6
      psrlq_i2r(8,mm3);
      psrlq_i2r(8,mm4);
      pminub_r2r(mm1,mm3);
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      // 7
      psrlq_i2r(8,mm3);
      psrlq_i2r(8,mm4);
      pminub_r2r(mm1,mm3);
      movq_r2r(mm1,mm0);
      pcmpgtw_r2r(mm3,mm0);
      pand_r2r(mm0,mm4);
      pandn_r2r(mm2,mm0);
      por_r2r(mm0,mm4);

      movq_r2m(mm3,min_lr);
      movq_r2m(mm4,min_d_lr);
      //PRINTREG8(mm3,"min_lr:");
      //PRINTREG8(mm1,"orig_min_lr:");
      //PRINTREG8(mm2,"min_d_lr:");
      
      if ((min_lr.ub[0]>0x00)&&(min_lr.ub[0]<0xff)) {
        ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = min_d_lr.ub[0];
      }
      
    }// for x
  }// for y

  emms();
  return 0;
}


#ifdef JUNK_DRAW
//----------------------------------------------------------------
// Zig zagging across the image
int Recursive_Depthmap_SAD3(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  const short W = 16;
  const short W_BUFF = 512;
  const short E = 0;

  short Y,X;
  register int d,e,x,y;
  register int i, j;
  register int w_2 = W/2;
  register int e_2 = E/2;
  register short xpw,x1pw,x1, XMAX;
  register short ypw,y1pw,y1, yMODW;



  short min, min_d;
  short P_temp;

  short ***P;
  short **Q;
  short **C;

  X = I1->cols;
  Y = I1->rows;
  
  if (*cache==0) {
	  unsigned int i = 0;
	  *cache = MMXMatrixAlloc(0,0,3 + X * (3+W));
    (*cache)->G[i++] = P = (short ***)Alloc(sizeof(short **)*X);
    (*cache)->G[i++] = Q = (short **)Alloc(sizeof(short *)*X);
    (*cache)->G[i++] = C = (short **)Alloc(sizeof(short *)*X);
    for (x=0;x<X;x++) {
      (*cache)->G[i++] = Q[x] = (short *)Alloc(sizeof(short)*D);
      (*cache)->G[i++] = C[x] = (short *)Alloc(sizeof(short)*D);
      (*cache)->G[i++] = P[x] = (short **)Alloc(sizeof(short *)*W);
      for (i=0;i<W;i++) {
        (*cache)->G[i++] = P[x][i] = (short *)Alloc(sizeof(short)*D);
      }
    }     
  }
  else { 
    P = (short ***) (*cache)->G[0];
    Q =  (short **) (*cache)->G[1];
    C = (short **) (*cache)->G[2];
  }
    
  XMAX = X-D-W-1;

  min = 0xffff;
  min_d = 0x0;  
  for (d=0;d<D;d++) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    // window across image
    for(i=0;i<W;i++) {
      Q[i][d] = 0;
      // window down image
      for (j=0;j<W-1;j++) {
  P[i][j][d]= abs(((MM_U_8 *)I1->p[j].m)[i+D_OFF] - ((MM_U_8 *)I2->p[j].m)[i+d]);
  Q[i][d] += P[i][j][d];  
      }
      C[0][d] += Q[i][d];
    }
  }

      
  for(y=-1;y<Y-W-2;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    yMODW = y%W;

    //----- ODD ROWS --------
    min = 0xffff;
    min_d = 0x0;
    for (d=0;d<D;d++) {
      //fprintf(stderr,"depthmap: d:%d \n",d);
      C[0][d]=0;
      for(i=0;i<W;i++) {
  P_temp = abs(((MM_U_8 *)I1->p[y+W].m)[i+D_OFF] - ((MM_U_8 *)I2->p[y+W].m)[i+d]);
  Q[i][d] = Q[i][d] - P[i][yMODW][d] + P_temp ;
  P[i][yMODW][d] = P_temp;
  C[0][d] += Q[i][d];
      }
      if (C[0][d] < min) {
  min = C[0][d];
  min_d = d;
      }
    }// for d
    ((MM_U_8 *)Out->p[y+1].m)[0]= min_d; 
  
    for(x=0;x<XMAX;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      xpw = x+W;
      x1 = x+1;

      min = 0xffff;
      min_d = 0x0;  
      for (d=0;d<D;d++) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if (y<0) {
    Q[xpw][d] = 0;
    for (j=0;j<W;j++) {
      P[xpw][j][d]= abs(((MM_U_8 *)I1->p[j].m)[x+W+D_OFF] - ((MM_U_8 *)I2->p[j].m)[x+W+d]);
      Q[xpw][d] += P[xpw][j][d];
    }
  }  
  else
  {
    // P is [X][W][D] the W dimension is a wrap around buffer.
        
    P_temp = abs(((MM_U_8 *)I1->p[y+W].m)[x+W+D_OFF] - ((MM_U_8 *)I2->p[y+W].m)[x+W+d]);
    Q[xpw][d] = Q[xpw][d] - P[xpw][yMODW][d] + P_temp ;
    P[xpw][yMODW][d] = P_temp;
  } 
  C[x1][d] = C[x][d] - Q[x][d] + Q[xpw][d];
   
  if (C[x1][d] < min) {
    min = C[x1][d];
    min_d = d;
  }
      }// for d
      ((MM_U_8 *)Out->p[y+1].m)[x+1]= min_d; 
    }// for x
    
    //----- EVEN ROWS --------
    y++;
    yMODW = y%W;

    min = 0xffff;
    min_d = 0x0;
    for (d=0;d<D;d++) {
      C[XMAX][d]=0;
      for(i=XMAX+W-1;i>=XMAX;i--) {
  P_temp = abs(((MM_U_8 *)I1->p[y+W].m)[i+D_OFF] - ((MM_U_8 *)I2->p[y+W].m)[i+d]);
  Q[i][d] = Q[i][d] - P[i][yMODW][d] + P_temp ;
  P[i][yMODW][d] = P_temp;
  //if (i>XMAX)
  C[XMAX][d] += Q[i][d];
      }  
      if (C[XMAX][d] < min) {
  min = C[x1][d];
  min_d = d;
      }
    }// for d
    ((MM_U_8 *)Out->p[y+1].m)[XMAX]= min_d; 

    for(x=XMAX;x>0;x--) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      x1pw = x-1+W;
      x1 = x-1;      

      min = 0xffff;
      min_d = 0x0;
      for (d=0;d<D;d++) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  // P is [X][W][D] the W dimension is a wrap around buffer.  
  P_temp = abs(((MM_U_8 *)I1->p[y+W].m)[x-1+D_OFF] - ((MM_U_8 *)I2->p[y+W].m)[x-1+d]);
  Q[x1][d] = Q[x1][d] - P[x1][yMODW][d] + P_temp ;
  P[x1][yMODW][d] = P_temp;  
  C[x1][d] = C[x][d] + Q[x1][d] - Q[x1pw][d];
  
  if (C[x1][d] < min) {
    min = C[x1][d];
    min_d = d;
  }
      }// for d
      ((MM_U_8 *)Out->p[y+1].m)[x1]= min_d; 
    }// for x
  }// for y

  /* Deallocate memory */
  for (x=0;x<X;x++) {
    for (i=0;i<W;i++) {
      free(P[x][i]);
    }
    free(Q[x]);
    free(C[x]);
    free(P[x]);
  } 
  free(P);
  free(Q);
  free(C);


}

//----------------------------------------------------------------
//----------------------------------------------------------------
//----------------------------------------------------------------
// Equivalent C function
// Q(x+1,y) = Q(x,y) - P(x,y) + P(x+win,y)
// assumes new P is in mm1
#define Qinc(Q,x,y,d,P) \
{\
     /* reg 4 = Q*/ \
     movq_m2r (((MM_U_16 *)Q->p[x].m)[d],mm4); \
     /* subtract prev P */\
     psubw_m2r (((MM_U_16 *)P[y]->p[x].m)[d],mm4);\
     /* Add curr P to Q*/\
     paddw_r2r (mm1,mm4);\
     /* Store Q*/ \
     movq_r2m (mm4,((MM_U_16 *)Q->p[x].m)[d]);\
}


// Equivalent C function
//C[d][x+1] = C[d][x] - Q[d][x] + Q[d][x+W];
// assumes new Q is in reg 4
#define Cinc(C,x,d,Q) \
{\
   /* reg 5 = C */ \
   movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);\
   /* Sub curr Q[x][d] from C */ \
   psubw_m2r (((MM_U_16 *)Q->p[x].m)[d],mm5);\
   /* Add Q[x+W][d] to C */\
   /* paddw_m2r (((MM_U_16 *)Q->p[x+W].m)[d],mm5);*/\
   paddw_r2r (mm4,mm5);\
   /* Store C[x+1][d] */\
   movq_r2m (mm5,((MM_U_16 *)C->p[x+1].m)[d]);\
}


inline void find_min(MMXMatrix *C, mmx_t *pcurr_d_lr,
         mmx_t *pmin_lr, mmx_t *pmin_d_lr)
{
  // ---------------------------------------------
  // find minimum C Left->Right for C[x+1][d]      
  // ---------------------------------------------
  // first create & store 0x0001000100010001 in 6
  // for future reference
  pxor_r2r(mm6,mm6);
  pcmpeqw_r2r(mm1,mm1);
  psubw_r2r(mm1,mm6);


  // reg 5 = C 
  //movq_m2r (((MM_U_16 *)C->p[x+1].m)[d],mm5);
        movq_r2r(mm5,mm0);

  //reg 4 = minimum C
  movq_m2r(*pmin_lr,mm4);
  movq_r2r(mm4,mm2);

  //fill 1 with 0x80008000s (sign bit)
  movq_r2r(mm6,mm1);
  psllw_i2r(15,mm1);

  paddw_r2r(mm1,mm2);
  paddw_r2r(mm1,mm0);
  //PRINTREG16(mm2,"min_lr");
  //PRINTREG16(mm0,"C");
        
  // reg 2 becomes mask
        pcmpgtw_r2r(mm0,mm2);
  movq_r2r(mm2,mm1);
  
  //PRINTREG16(mm2,"mask");
  
  // reg 0 is C values smaller than minimum
  movq_r2r(mm5,mm0);
  pand_r2r(mm2,mm0);
    
  // reg 4 is preserved minimum
  pandn_r2r(mm4,mm1);
  
  // reg 1 is new minimum
  por_r2r(mm0,mm1);
  movq_r2m(mm1,*pmin_lr);
  //PRINTREG16(mm6,"new_min_lr");

  // reg 4 is d values for minimum
  movq_m2r(*pmin_d_lr,mm4);
  // reg 1 is preserved d values
        movq_r2r(mm2,mm1);
  pandn_r2r(mm4,mm1);

  // reg 3 = current d value
  // increase current d values by 4
  movq_m2r(*pcurr_d_lr,mm3);
  movq_r2r(mm3,mm0);


  // reg 0 = current d value for updated minimum
  pand_r2r(mm2,mm0);
  
  // reg 1 = new d's for minimum values
  por_r2r(mm0,mm1);
  movq_r2m(mm1,*pmin_d_lr);
  //PRINTREG16(mm1,"new_min_d_lr");

  // increase current d's
  // fill with 0x0004000400040004 
  psllw_i2r(2,mm6);
         paddw_r2r(mm6,mm3);
  
         //paddw_m2r(plus_four,mm3);
  movq_r2m(mm3,*pcurr_d_lr);
}

//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD2(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  const short W = 16;

  short Y,X;

  register short i, j;
  register short d,e,x,y;
  
  short w_2 = (W/2);
  short d_w_2_1 = D_OFF+w_2+1;
  short yMODW, XMAX;

  unsigned short overall_min_lr;
  unsigned short overall_min_rl;

  short idx_rl; 
  unsigned short min_rl;

  unsigned char overall_min_d_rl;
  unsigned char overall_min_d_lr;
  unsigned char d_rl; 
  unsigned char d_lr; 
  unsigned char min_d_rl;
  unsigned char prev;

  int subpix_d,subpix_min,subpix_denom;

  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C1;
  MMXMatrix *C2;
  MMXMatrix *C;
  MMXMatrix *C_prev;
  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  
  X = I1->cols;
  Y = I1->rows;

  if (*cache==0) {
	  unsigned int i = 0;
	  *cache = MMXMatricesCacheAlloc(3+W,0,0);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C1 = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C2 = MMXMatrixAlloc(MMT_U_16, D, X);
    P = (*cache)->M + 3;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, D, X);
  }
  else { 
    P = (*cache)->M + 3;
    Q =  (*cache)->M[0];
    C1 = (*cache)->M[1]; 
    C2 = (*cache)->M[2]; 
  }

  emms();

  C = C2;
  C_prev = C1;
  XMAX = X-D-W-1;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    yMODW = y%W;
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }

    //----- ODD ROWS --------    
    for(x=-1;x<X-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);


      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;

      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4); 
    
      pxor_r2r (mm7,mm7); 

      for (d=0;d<D-3;d+=4) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if ((x<0)&&(y<0)) {  
    
    // Equivalent C function
    //C[d][0]=0; 
    //for(i=0;i<=W;i+=4) {
    //  Q[d][i] = 0;
    // window down image
    //  for (j=0;j<W;j++) {
    //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
    //  Q[d][i] += P[j][d][i];
    //  } 
    //  if (i<W)
    //    C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    pxor_r2r (mm5,mm5);
    for(i=0;i<W;i++) {
      // reg 4 = Q
      pxor_r2r (mm4,mm4);
      
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2D(I1, I2, i+D_OFF,j,i+d,j);        
        // Add curr P to Q
        paddw_r2r (mm1,mm4);
      }
      // Store Q
      movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
      // PRINTREG16(mm4,"Q");
    
      // Add curr Q to C
      paddw_r2r (mm4,mm5);
    }
    // Store C
    movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
   
    // compute for i=W
    // reg 4 = Q
    pxor_r2r (mm4,mm4);  
    // window down image
    for (j=0;j<W;j++) {
      // lbw in 0
      I1_I2D(I1, I2, W+D_OFF,j,W+d,j);

      movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
      // Add curr P to Q
      paddw_r2r (mm1,mm4);
    }
    // Store Q
    movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
    
  }
  else if (x<0) {
    // Equivalent C function
    //C[d][0]=0;
    //for(i=0;i<=W;i++) {
    // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
    // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
    // P[y%W][d][i] = P_temp;
    // if (i<W)
    //   C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    pxor_r2r (mm5,mm5);
    for(i=0;i<W;i++) {
      I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
      // P_temp is mm1
      Qinc(Q,i,yMODW,d,P);
      // Store P
      movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[i].m)[d]);
      // Add curr Q to C
      paddw_r2r (mm4,mm5);
    }
    // Store C[d][0]
    movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
    //PRINTREG16(mm5,"C");

    // Do for i=W
    I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
    // P_temp is mm1
    Qinc(Q,W,yMODW,d,P);
    // Store P
    movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[W].m)[d]);
    
  }
  else
  {
    if (y<0) {
      // Equivalent C function
      //Q[x+W][d] = 0;
      //for (j=0;j<W;j++) {
      //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
      //  Q[x+W][d] += P[x+W][j][d];
      //}
      
      // reg 4 = Q
      pxor_r2r (mm4,mm4);
      
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
        //PRINTREG16(mm1,"mmx1");
        movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
        // Add curr P to Q
        paddw_r2r (mm1,mm4);
      }
      // Store Q
      movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
      
    }  
    else
    {
      
      // Equivalent C function
      //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
      //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
      //P[x+W][y%W][d] = P_temp;

      I1_I2D(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
      // P_temp is mm1

      Qinc(Q,x+W,yMODW,d,P);      
      // Store P
      movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[x+W].m)[d]);
      
    }
    // Equivalent C function
    //C[d][x+1] = C[d][x] - Q[d][x] + Q[d][x+W];
    Cinc(C,x,d,Q);    


  }

  find_min(C, &curr_d_lr, &min_lr, &min_d_lr);
  
      }// for d

          
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]      
      // ---------------------------------------------
            
      overall_min_lr = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      if (overall_min_lr > min_lr.uw[1]) {
  overall_min_lr = min_lr.uw[1];
  overall_min_d_lr = min_d_lr.uw[1];
      }
      if (overall_min_lr > min_lr.uw[2]) {
  overall_min_lr = min_lr.uw[2];
  overall_min_d_lr = min_d_lr.uw[2];
      }
      if (overall_min_lr > min_lr.uw[3]) {
  overall_min_lr = min_lr.uw[3];
  overall_min_d_lr = min_d_lr.uw[3];
      }

#ifndef DEPTH64
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = overall_min_d_lr<<3;
#else
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = overall_min_d_lr<<2;
#endif
            
           
    }// for x

    //----- EVEN ROWS --------
    y++;
    yMODW = y%W;
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }


    for(x=XMAX+1;x>0;x--) {
      //fprintf(stderr,"depthmap: x:%d \n",x);


      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;

      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4); 
    
      pxor_r2r (mm7,mm7); 

      for (d=0;d<D-3;d+=4) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if (x>XMAX) {
    // Equivalent C function
    //C[d][0]=0;
    //for(i=0;i<=W;i++) {
    // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
    // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
    // P[y%W][d][i] = P_temp;
    // if (i<W)
    //   C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    pxor_r2r (mm5,mm5);
    for(i=XMAX+W-1;i>=XMAX;i--) {
      I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
      // P_temp is mm1
      Qinc(Q,i,yMODW,d,P);
      // Store P
      movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[i].m)[d]);
      // Add curr Q to C
      paddw_r2r (mm4,mm5);
    }
    // Store C[d][0]
    movq_r2m (mm5,((MM_U_16 *)C->p[XMAX].m)[d]);
    //PRINTREG16(mm5,"C");

    // Do for i=W
    //I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
    // P_temp is mm1
    //Qinc(Q,W,yMODW,d,P);
    // Store P
    //movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[W].m)[d]);
    
  }
  else
  {
    // Equivalent C function
    //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
    //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
    //P[x+W][y%W][d] = P_temp;
    I1_I2D(I1, I2,x-1+D_OFF,y+W,x-1+d,y+W);
    // P_temp is mm1
    Qinc(Q,x-1,yMODW,d,P);      
    // Store P
    movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[x-1].m)[d]);
   
    // Equivalent C function
    //C[d][x+1] = C[d][x] - Q[d][x] + Q[d][x+W];
    //Cinc(C,x,d,Q);    
    /* reg 5 = C */ 
    movq_m2r (((MM_U_16 *)C->p[x].m)[d],mm5);
    /* Sub curr Q[x][d] from C */ 
    psubw_m2r (((MM_U_16 *)Q->p[x-1+W].m)[d],mm5);
    /* Add Q[x+W][d] to C */
    /* paddw_m2r (((MM_U_16 *)Q->p[x-1].m)[d],mm5);*/
    paddw_r2r (mm4,mm5);
    /* Store C[x+1][d] */
    movq_r2m (mm5,((MM_U_16 *)C->p[x-1].m)[d]);

  }

  find_min(C, &curr_d_lr, &min_lr, &min_d_lr);
  
      }// for d

          
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]      
      // ---------------------------------------------
            
      overall_min_lr = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      if (overall_min_lr > min_lr.uw[1]) {
  overall_min_lr = min_lr.uw[1];
  overall_min_d_lr = min_d_lr.uw[1];
      }
      if (overall_min_lr > min_lr.uw[2]) {
  overall_min_lr = min_lr.uw[2];
  overall_min_d_lr = min_d_lr.uw[2];
      }
      if (overall_min_lr > min_lr.uw[3]) {
  overall_min_lr = min_lr.uw[3];
  overall_min_d_lr = min_d_lr.uw[3];
      }

#ifndef DEPTH64
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1-2] = overall_min_d_lr<<3;
#else
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1-2] = overall_min_d_lr<<2;
#endif
            
           
    }// for x



  }// for y

  emms();
}


//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD2a(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out, const int D, const int D_OFF, MMXMatricesCachePtr* vcache)
{  
	MMXMatricesCache** cache = (MMXMatricesCache**)vcache;
  const short W = 16;


  short Y,X;

  register short i, j;
  register short d,e,x,y;
  
  short w_2 = (W/2);
  short d_w_2_1 = D_OFF+w_2+1;
  short yMODW;

  unsigned short overall_min_lr;
  unsigned short overall_min_rl;

  short idx_rl; 
  unsigned short min_rl;

  unsigned char overall_min_d_rl;
  unsigned char overall_min_d_lr;
  unsigned char d_rl; 
  unsigned char d_lr; 
  unsigned char min_d_rl;
  unsigned char prev;

  int subpix_d,subpix_min,subpix_denom;


  MMXMatrix **P;
  MMXMatrix *Q;
  MMXMatrix *C1;
  MMXMatrix *C2;
  MMXMatrix *C;
  MMXMatrix *C_prev;
  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t min_d_lr;

  
  X = I1->cols;
  Y = I1->rows;

  if (*cache==0) {
	  unsigned int i = 0;
	  *cache = MMXMatricesCacheAlloc(3+W,0,0);
    (*cache)->M[i++] = Q = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C1 = MMXMatrixAlloc(MMT_U_16, D, X);
    (*cache)->M[i++] = C2 = MMXMatrixAlloc(MMT_U_16, D, X);
    P = (*cache)->M+3;
    for (i=0;i<W;i++)
      P[i] = MMXMatrixAlloc(MMT_U_16, D, X);
  }
  else { 
    P = (*cache)->M+3;
    Q = (*cache)->M[0];
    C1 =(*cache)->M[1];
    C2 =(*cache)->M[2];
  }

  emms();

  C = C2;
  C_prev = C1;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    yMODW = y%W;
    if (C==C1) {
      C = C2;
      C_prev = C1;
    }
    else {
      C = C1;
      C_prev = C2;
    }
    
    for(x=-1;x<X-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);


      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      curr_d_lr.ud[0] = 0x00010000;
      curr_d_lr.ud[1] = 0x00030002;

      // clear registers
      //pxor_r2r (mm3,mm3);
      // pxor_r2r (mm4,mm4); 
    
      pxor_r2r (mm7,mm7); 

      for (d=0;d<D-3;d+=4) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if ((x<0)&&(y<0)) {  
    
    // Equivalent C function
    //C[d][0]=0; 
    //for(i=0;i<=W;i+=4) {
    //  Q[d][i] = 0;
    // window down image
    //  for (j=0;j<W;j++) {
    //  P[j][d][i]= I1_I2(I1, I2,i+D_OFF,j,i+d,j);
    //  Q[d][i] += P[j][d][i];
    //  } 
    //  if (i<W)
    //    C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    pxor_r2r (mm5,mm5);
    for(i=0;i<W;i++) {
      // reg 4 = Q
      pxor_r2r (mm4,mm4);
      
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2D(I1, I2, i+D_OFF,j,i+d,j);        
        // Add curr P to Q
        paddw_r2r (mm1,mm4);
      }
      // Store Q
      movq_r2m (mm4,((MM_U_16 *)Q->p[i].m)[d]);
      // PRINTREG16(mm4,"Q");
    
      // Add curr Q to C
      paddw_r2r (mm4,mm5);
    }
    // Store C
    movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
   
    // compute for i=W
    // reg 4 = Q
    pxor_r2r (mm4,mm4);  
    // window down image
    for (j=0;j<W;j++) {
      // lbw in 0
      I1_I2D(I1, I2, W+D_OFF,j,W+d,j);

      movq_r2m (mm1,((MM_U_16 *)P[j]->p[W].m)[d]);
      // Add curr P to Q
      paddw_r2r (mm1,mm4);
    }
    // Store Q
    movq_r2m (mm4,((MM_U_16 *)Q->p[W].m)[d]);
    
  }
  else if (x<0) {
    // Equivalent C function
    //C[d][0]=0;
    //for(i=0;i<=W;i++) {
    // P_temp = I1_I2(I1, I2, i+D_OFF,y+W,i+d,y+W);
    // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
    // P[y%W][d][i] = P_temp;
    // if (i<W)
    //   C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    pxor_r2r (mm5,mm5);
    for(i=0;i<W;i++) {
      I1_I2D(I1, I2, i+D_OFF,y+W,i+d,y+W);
      // P_temp is mm1
      Qinc(Q,i,yMODW,d,P);
      // Store P
      movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[i].m)[d]);
      // Add curr Q to C
      paddw_r2r (mm4,mm5);
    }
    // Store C[d][0]
    movq_r2m (mm5,((MM_U_16 *)C->p[0].m)[d]);
    //PRINTREG16(mm5,"C");

    // Do for i=W
    I1_I2D(I1, I2, W+D_OFF,y+W,W+d,y+W);
    // P_temp is mm1
    Qinc(Q,W,yMODW,d,P);
    // Store P
    movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[W].m)[d]);
    
  }
  else
  {
    if (y<0) {
      // Equivalent C function
      //Q[x+W][d] = 0;
      //for (j=0;j<W;j++) {
      //  P[x+W][j][d]= I1_I2(I1, I2, x+W+D_OFF,j,x+W+d,j);
      //  Q[x+W][d] += P[x+W][j][d];
      //}
      
      // reg 4 = Q
      pxor_r2r (mm4,mm4);
      
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2D(I1, I2,x+W+D_OFF,j,x+W+d,j);
        //PRINTREG16(mm1,"mmx1");
        movq_r2m (mm1,((MM_U_16 *)P[j]->p[x+W].m)[d]);
        // Add curr P to Q
        paddw_r2r (mm1,mm4);
      }
      // Store Q
      movq_r2m (mm4,((MM_U_16 *)Q->p[x+W].m)[d]);
      
    }  
    else {
      
      // Equivalent C function
      //P_temp = I1_I2(I1, I2, x+W+D_OFF,y+W,x+W+d,y+W);
      //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
      //P[x+W][y%W][d] = P_temp;

      I1_I2D(I1, I2,x+W+D_OFF,y+W,x+W+d,y+W);
      // P_temp is mm1

      Qinc(Q,x+W,yMODW,d,P);      
      // Store P
      movq_r2m (mm1,((MM_U_16 *)P[yMODW]->p[x+W].m)[d]);
      
    }
    // Equivalent C function
    //C[d][x+1] = C[d][x] - Q[d][x] + Q[d][x+W];
    Cinc(C,x,d,Q);    
  }

  find_min(C, &curr_d_lr, &min_lr, &min_d_lr);
  
      }// for d

          
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]      
      // ---------------------------------------------
            
      overall_min_lr = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      if (overall_min_lr > min_lr.uw[1]) {
        overall_min_lr = min_lr.uw[1];
        overall_min_d_lr = min_d_lr.uw[1];
      }
      if (overall_min_lr > min_lr.uw[2]) {
        overall_min_lr = min_lr.uw[2];
        overall_min_d_lr = min_d_lr.uw[2];
      }
      if (overall_min_lr > min_lr.uw[3]) {
        overall_min_lr = min_lr.uw[3];
        overall_min_d_lr = min_d_lr.uw[3];
      }

#ifndef DEPTH64
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = overall_min_d_lr<<3;
#else
      ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = overall_min_d_lr<<2;
#endif
            
           
    }// for x
  }// for y

  emms();
}





        //reg 4 = minimum so far
        //movq_m2r(min_lr,mm4);
        //movq_r2r(mm4,mm2);


        //fill 1 with 0x80808080s (sign bit)
        //movq_r2r(mm6,mm1);
        //psllw_i2r(7,mm1);
        //PRINTREG8(mm1,"0x80808080:");
        //paddb_r2r(mm1,mm2);
        //paddb_r2r(mm1,mm0);
        //PRINTREG16(mm2,"min_lr");
        //PRINTREG16(mm0,"C");

        // reg 2 becomes mask
        //pcmpgtb_r2r(mm0,mm2);
        //movq_r2r(mm2,mm1);

        // reg 0 is C values smaller than minimum
        //movq_r2r(mm5,mm0);
        //pand_r2r(mm2,mm0);

        // reg 4 is preserved minimum
        //pandn_r2r(mm4,mm1);

        // reg 1 is new minimum
        //por_r2r(mm0,mm1);
        //movq_r2m(mm1,min_lr);



      /*
      overall_min_lr = min_lr.ub[0];
      overall_min_d_lr = min_d_lr.ub[0];
      if (overall_min_lr > min_lr.ub[1]) {
        overall_min_lr = min_lr.ub[1];
        overall_min_d_lr = min_d_lr.ub[1];
      }
      if (overall_min_lr > min_lr.ub[2]) {
        overall_min_lr = min_lr.ub[2];
        overall_min_d_lr = min_d_lr.ub[2];
      }
      if (overall_min_lr > min_lr.ub[3]) {
        overall_min_lr = min_lr.ub[3];
        overall_min_d_lr = min_d_lr.ub[3];
      }
      if (overall_min_lr > min_lr.ub[4]) {
        overall_min_lr = min_lr.ub[4];
        overall_min_d_lr = min_d_lr.ub[4];
      }
      if (overall_min_lr > min_lr.ub[5]) {
        overall_min_lr = min_lr.ub[5];
        overall_min_d_lr = min_d_lr.ub[5];
      }
      if (overall_min_lr > min_lr.ub[6]) {
        overall_min_lr = min_lr.ub[6];
        overall_min_d_lr = min_d_lr.ub[6];
      }
      if (overall_min_lr > min_lr.ub[7]) {
        overall_min_lr = min_lr.ub[7];
        overall_min_d_lr = min_d_lr.ub[7];
      }
      if ((overall_min_lr>0x00)&&(overall_min_lr<0xff)) {
        ((MM_U_8 *)Out->p[y+w_2+1].m)[x+d_w_2_1] = overall_min_d_lr;
      }
      */

#endif


