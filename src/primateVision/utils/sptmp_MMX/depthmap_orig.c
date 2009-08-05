 #define MAIN

#include "depthmap.h"

#define FAST_SQRT

#ifdef FAST_SQRT
#include "fast_sqrt.h"
#endif

//#define I1_I2(I1,I2,a,b,c,d) abs(((MM_U_8 *)(I1)->p[(b)])[(a)]-((MM_U_8 *)(I2)->p[(d)])[(c)])


// I1_I2:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1+1,y1)-I2(x2+1,y2))
// abs(I1(x1+2,y1)-I2(x2+2,y2))
// abs(I1(x1+3,y1)-I2(x2+3,y2))
#define I1_I2(I1,I2,x1,y1,x2,y2) \
{\
  MOVQ_MR(((MM_U_8 *)I1->p[y1])[x1],0);\
  PSUBUSB_MR(((MM_U_8 *)I2->p[y2])[x2],0);\
  MOVQ_MR(((MM_U_8 *)I2->p[y2])[x2],2);\
  PSUBUSB_MR(((MM_U_8 *)I1->p[y1])[x1], 2);\
  POR_RR(2,0);\
  PUNPCKLBW_RR(7,0);\
}

// I1_I2S
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1+1,y1)-I2(x2+1,y2))
// abs(I1(x1+2,y1)-I2(x2+2,y2))
// abs(I1(x1+3,y1)-I2(x2+3,y2))
#define I1_I2S(I1,I2,x1,y1,x2,y2) \
{\
  MOVQ_MR(((MM_U_8 *)I1->p[y1])[x1],0);\
  PSUBUSB_MR(((MM_U_8 *)I2->p[y2])[x2],0);\
  MOVQ_MR(((MM_U_8 *)I2->p[y2])[x2],1);\
  PSUBUSB_MR(((MM_U_8 *)I1->p[y1])[x1], 1);\
  POR_RR(1,0);\
  MOVQ_RR(0,2);\
  PUNPCKHBW_RR(7,2);\
  MOVQ_RR(2,1);\
  MOVQ_RR(0,3);\
  PSLLQ_IR(8,3);\
  PUNPCKHBW_RR(7,3);\
  PADDW_RR(3,1);\
  MOVQ_RR(0,2);\
  PSLLQ_IR(16,2);\
  PUNPCKHBW_RR(7,2);\
  PADDW_RR(2,1);\
  MOVQ_RR(0,3);\
  PSLLQ_IR(24,3);\
  PUNPCKHBW_RR(7,3);\
  PADDW_RR(3,1);\
}


// I1_I2D:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1,y1)-I2(x2+1,y2))
// abs(I1(x1,y1)-I2(x2+2,y2))
// abs(I1(x1,y1)-I2(x2+3,y2))
#define I1_I2D(I1,I2,x1,y1,x2,y2) \
{\
  MOVQ_MR(((MM_U_8 *)I1->p[y1])[x1],0);\
  PUNPCKLBW_RR(7,0);\
  PUNPCKLWD_RR(0,0);\
  PUNPCKLDQ_RR(0,0);\
  MOVQ_RR(0,1);\
  MOVQ_MR(((MM_U_8 *)I2->p[y2])[x2],2);\
  PUNPCKLBW_RR(7,2);\
  MOVQ_RR(2,3);\
  PSUBUSW_RR(0, 2);\
  PSUBUSW_RR(3, 1);\
  POR_RR(2,1);\
}

// I1_I2MULTISUM
// I1(x1,y1)*I2(x2,y2) +
// I1(x1+1,y1)*I2(x2+1,y2)
#define I1_I2MULTISUM(I1,I2,x1,y1,x2,y2) \
{\
  MOVQ_MR(((MM_U_8 *)I1->p[y1])[x1],2);\
  MOVQ_MR(((MM_U_8 *)I2->p[y2])[x2],3);\
  MOVQ_RR(2,0);\
  MOVQ_RR(3,1);\
  PUNPCKLBW_RR(7,0);\
  PUNPCKLBW_RR(7,1);\
  PMADDWD_RR(1,0);\
  PSRLQ_IR(8,2);\
  PSRLQ_IR(8,3);\
  PUNPCKLBW_RR(7,2);\
  PUNPCKLBW_RR(7,3);\
  PMADDWD_RR(3,2);\
  PUNPCKLDQ_RR(2,0);\
}

// I1_I2MULTISUM2
// I1(x1,y1)*I2(x2,y2) +
// I1(x1+1,y1)*I2(x2+1,y2) +
// I1(x1+2,y1)*I2(x2+2,y2) +
// I1(x1+3,y1)*I2(x2+3,y2)
#define I1_I2MULTISUM2(I1,I2,x1,y1,x2,y2) \
{\
  MOVQ_MR(((MM_U_8 *)I1->p[y1])[x1],0);\
  MOVQ_MR(((MM_U_8 *)I2->p[y2])[x2],1);\
  MOVQ_RR(0,2);\
  MOVQ_RR(1,3);\
  PUNPCKHBW_RR(7,0);\
  PUNPCKHBW_RR(7,1);\
  PSLLQ_IR(8,2);\
  PSLLQ_IR(8,3);\
  PMADDWD_RR(1,0);\
  PUNPCKHBW_RR(7,2);\
  PUNPCKHBW_RR(7,3);\
  PMADDWD_RR(3,2);\
  MOVQ_RR(2,3);\
  PUNPCKHDQ_RR(0,2);\
  PUNPCKLDQ_RR(0,3);\
  PADDD_RR(3,2);\
  MOVQ_RR(2,0);\
}

void image_to_matrix (unsigned char * image, Matrix * matrix, 
          int width, int height, int start_x, int start_y,
          int s_width, int s_height)
{
  int x,y;
  int image_index;
  char c; 
  // fprintf(stderr,"before loop in image to matrix\n");
  /* indeices put this way to be consistent with rest of matrix code */
  for (y = 0; y < s_height; y++) {
    for (x = 0; x < s_width; x++) {
      //   fprintf(stderr,"in itom xy = %d,%d\n",x,y);
      image_index = (start_y + y) * width + x + start_x; 
      ((MM_U_8 *)(matrix->p[y]))[x] = image[image_index];
    
    }
  }
   
  //   debug = 1;
  //   fprintf(stderr,"before print matrix in image to matrix\n");
  //  PrintMatrix(matrix,"search");
  //   c = getchar();
  // debug = 0;
}

void matrix_to_image(Matrix * matrix, unsigned char * image,  
          int width, int height, int start_x, int start_y,
          int s_width, int s_height)
{
  int x,y;
  int image_index;
  char c; 
  // fprintf(stderr,"before loop in image to matrix\n");
  /* indeices put this way to be consistent with rest of matrix code */
  for (y = 0; y < s_height; y++) {
    for (x = 0; x < s_width; x++) {
      //   fprintf(stderr,"in itom xy = %d,%d\n",x,y);
      image_index = (start_y + y) * width + x + start_x; 
      ((MM_U_8 *)image)[image_index] = ((MM_U_8 *)(matrix->p[y]))[x];    
    }
  }
   
  //   debug = 1;
  //   fprintf(stderr,"before print matrix in image to matrix\n");
  //  PrintMatrix(matrix,"search");
  //   c = getchar();
  // debug = 0;
}

void matrix_to_double (Matrix * matrix, double * result, int width, int height)
{
  int x,y;
  
  /* indeices put this way to be consistent with rest of matrix code */
  for (x = 0; x < height; x++) {
    for (y = 0; y < width; y++) {
      result[x*width+y]; ((double *)(matrix->p[x]))[y];
    }
  }

}

void allocMMXmatrices (int t_width, int t_height, int s_width, int s_height)
{
 
  image_matrix = MatrixAlloc(MMT_U_8, s_height, s_width);
  template_matrix = MatrixAlloc(MMT_U_8, t_height, t_width);
  result_matrix = MatrixAlloc(sizeof(MMT_S_32), s_width - t_width + 1, 
            s_height - t_height + 1);
  MMX_is_allocated = 1;
}

void deallocMMXmatrices ()
{
  free(image_matrix);
  free(template_matrix);
  free(result_matrix);
  MMX_is_allocated = 0;
}


/********************************************************************/
/* rank transform                                                   */
/********************************************************************/
void MMXrank_trans (Matrix *I, Matrix  *O)
{
  int Y,X;
  const int W = 8;

  mmxdata ones,result;

  register  int x,y,j,i, w_2, count,curr;

  w_2 = W/2;
  Y = I->rows;
  X = I->cols;
  


  // take care of 0->W/2
  for (y = 0; y < w_2; y++) {
    for (x = 0; x < w_2; x++) {
      count = 0;
      curr = ((MM_U_8 *)I->p[y])[x];
      //printf("rank: source: %d\n",curr);
      for (j = 0; j < W; j++) {
  for (i = 0; i < W; i++) {
    if (((y+j-w_2)>=0)&&((x+i-w_2)>=0)){
      if (((MM_U_8 *)I->p[y+j-w_2])[x+i-w_2]<curr) { 
        count++;
      }
    }
      
  }
      }
      ((MM_U_8 *)O->p[y])[x]=count; 
    }
  }
  // take care of Y-W/2 -> Y
  for (y = (Y-w_2); y < Y; y++) {
    for (x = (X-w_2); x < X; x++) {
      count = 0;
      curr = ((MM_U_8 *)I->p[y])[x];
      for (j = 0; j < W; j++) {
  for (i = 0; i < W; i++) {
    if (((y+j-w_2)<Y)&&((x+i-w_2)<X)) {
      if (((MM_U_8 *)I->p[y+j-w_2])[x+i-w_2]<curr) { 
        count++;
      }
    }
      
  }
      }
      ((MM_U_8 *)O->p[y])[x]=count;
    }
  }
 
  // the rest
  for (y = 0; y < Y; y++) {
    for (x = 0; x < X; x++) {
      count = 0;
     
      curr = ((MM_U_8 *)I->p[y])[x];
      //printf("rank: source: %d\n",curr);
      for (j = 0; j < W; j++) {
  for (i = 0; i < W; i++) {
    if ( ((y+j-w_2)>=0) && ((y+j-w_2)<Y) ) {
      if ( ((x+i-w_2)>=0) && ((x+i-w_2)<X) ) {
        if (((MM_U_8 *)I->p[y+j-w_2])[x+i-w_2]<curr) { 
    count++;
        }
      }
    }
      
  }
      }
      ((MM_U_8 *)O->p[y])[x]=count;
      //printf("rank: dest: %d\n",count);
    
    }
  }
  
  EMMS;
  ones.mmu32[0]=0x01010101;
  ones.mmu32[1]=0x01010101;

  MOVQ_MR(ones,6);      

  PXOR_RR(7,7);

  for (y = 0; y < Y-W; y++) {
    for (x = 0; x < X-W; x++) {
      // I -> I1 I1 I1 I1 I1 I1 I1 I1
      MOVQ_MR(((MM_U_8 *)I->p[y+w_2])[x+w_2],0);      
      PUNPCKLBW_RR(7,0);
      PUNPCKLBW_RR(0,0);
      PUNPCKLWD_RR(0,0);
      PUNPCKLDQ_RR(0,0);

      PXOR_RR(5,5);
      for (j = 0; j < W; j++) {
        // I -> I1 I2 I3 I4 I5 I6 I7 I8
  MOVQ_MR(((MM_U_8 *)I->p[y+j])[x],2);
  // reg 2 becomes mask
        PCMPGTB_RR(0,2);
  PAND_RR(6,2);
  PADDB_RR(2,5);
      }
      MOVQ_RR (5,4);
      PUNPCKLBW_RR (7,5);
      PUNPCKHBW_RR (7,4);
      PADDW_RR (5,4);
      MOVQ_RR (4,3);
      PUNPCKLWD_RR (7,3);
      PUNPCKHWD_RR (7,4);
      PADDD_RR (4,3);  
      MOVQ_RM (3,result);

      ((MM_U_8 *)O->p[y+w_2])[x+w_2]= result.mmu32[0] + result.mmu32[1];          
    }
  }
  EMMS;
}

//-----------------------------------------------------------------------
//----------------------------------------------------------------
int Depthmap_SAD(Matrix *I1, Matrix *I2, Matrix  *O)
{
  // const int N = 128;
  const int Y = 240;
  const int X = 512;

  const int W = 10;
  const int D = 40;
  const int E = 8;


  int i, j;
  int sum, mean, count, best, best_d;

  register int d,e,x,y;

  
  int w_2 = W/2;
  int d_2 = D/2;
  int e_2 = E/2;

  
  for(y=0;y<Y;y++){
    fprintf(stderr,"depthmap: y:%d \n",y);

    for(x=0;x<X;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
  
      if ((y<(e_2+w_2))||(x<(d_2+w_2))||(y>(Y-1-e_2-w_2))||(x>(X-1-d_2-w_2))) {
  ((MM_U_8 *)O->p[y])[x]=0;
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

    sum+= abs(((MM_U_8 *)I1->p[y-w_2+j])[x-w_2+i]-((MM_U_8 *)I2->p[y+e-e_2-w_2+j])[x+d-d_2-w_2+i]);
        }    
      }
      if ((d==0)&&(e==0)||(sum < best)) {
        best = sum;
        best_d = e;
      }
    }
  }
  ((MM_U_8 *)O->p[y])[x]= best_d;
  
      }
    }
  }

}

//----------------------------------------------------------------
int Recursive_Depthmap_SAD(Matrix *I1, Matrix *I2, Matrix  *Out)
{
  const int N = 512;
  const int Y = 240;
  const int X = 512;

  const int W = 16;
  const int D = 40;
  const int E = 0;


  register int i, j;
  int sum, min, min_d;

  register int d,e,x,y;

  
  int w_2 = W/2;
  int d_2 = D/2;
  int e_2 = E/2;

  int P_temp;
  int P[N][W][D];
  int Q[N][D];
  int C[N][D];
    

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    
    for(x=-1;x<N-D-1-W;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      for (d=0;d<D;d++) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    if ((x<0)&&(y<0)) {

      C[0][d]=0;
      // window across image
      for(i=0;i<=W;i++) {
        Q[i][d] = 0;
        // window down image
        for (j=0;j<W;j++) {
    P[i][j][d]= abs(((MM_U_8 *)I1->p[j])[i+d_2] - ((MM_U_8 *)I2->p[j])[i+d]);
    Q[i][d] += P[i][j][d];
    printf("I1_I2(%d,%d)(%d,%d):",i+d_2,j,i+d,j);
    printf("P(%d,%d,%d):%d\n",i,j,d,P[i][j][d]);
       
        } 
        printf("Q(%d,%d):%d",i,d,Q[i][d]);
        
        if (i<W)
    C[0][d] += Q[i][d];
      }
    }
    else if (x<0) {
  
      C[0][d]=0;
      for(i=0;i<=W;i++) {
        P_temp = abs(((MM_U_8 *)I1->p[y+W])[i+d_2] - ((MM_U_8 *)I2->p[y+W])[i+d]);

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
    P[x+W][j][d]= abs(((MM_U_8 *)I1->p[j])[x+W+d_2] - ((MM_U_8 *)I2->p[j])[x+W+d]);
    Q[x+W][d] += P[x+W][j][d];
        }
      }  
      else
      {
        // P is [N][W][D] the W dimension is a wrap around buffer.
        
        P_temp = abs(((MM_U_8 *)I1->p[y+W])[x+W+d_2] - ((MM_U_8 *)I2->p[y+W])[x+W+d]);
        Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
        P[x+W][y%W][d] = P_temp;
      }

      C[x+1][d] = C[x][d] - Q[x][d] + Q[x+W][d];
    }


    printf("%d,%d,%d: %d\n",x,y,d, C[x+1][d]);

          if ((d==0)||(C[x+1][d] < min)) {
      min = C[x+1][d];
      min_d = d;
    }
      }// for d
      ((MM_U_8 *)Out->p[y+1])[x+1]= min_d; 
    }// for x
  }// for y

}

//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD(Matrix *I1, Matrix *I2, Matrix  *Out)
{
  int N,Y,X;

  const int W = 16;
  const int D = 32;
  const int E = 8;


  register short i, j;
  register short d,e,x,y;

  //  unsigned char min_d;
  //unsigned short sum, min;

  short w_2 = W/2;
  short d_2 = D/2;
  int e_2 = E/2;

  Matrix **P;
  Matrix *Q;
  Matrix *C;
  mmxdata sign_offset;
  mmxdata min;
  mmxdata curr_d;
  mmxdata min_d;

  Y=I1->rows;
  N=I1->cols;
  X=I1->cols;
    

  Q = MatrixAlloc(MMT_U_16, D, N);
  C = MatrixAlloc(MMT_U_16, D, N);

  P = (Matrix **) Alloc(sizeof(Matrix *)*W); 
  for (i=0;i<W;i++)
    P[i] = MatrixAlloc(MMT_U_16, D, N);

  sign_offset.mmu32[0] =0x80008000;
  sign_offset.mmu32[1] =0x80008000;

  EMMS;

  // y dc offset to align images
  e = 0;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    
    for(x=-4;x<N-W-D-4;x+=4) {
      //fprintf(stderr,"depthmap: x:%d \n",x);

      // clear registers
      //PXOR_RR (3,3);
      // PXOR_RR (4,4); 
      PXOR_RR (7,7); 
      min.mmu32[0] = 0xffffffff;
      min.mmu32[1] = 0xffffffff;

      for (d=0;d<D;d++) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if ((x<0)&&(y<0)) {
      
    // Equivalent C function
    //C[d][0]=0; 
    //for(i=0;i<=W;i+=4) {
    //  Q[d][i] = 0;
    // window down image
    //  for (j=0;j<W;j++) {
    //  P[j][d][i]= I1_I2(I1, I2,i+d_2,j,i+d,j);
    //  Q[d][i] += P[j][d][i];
    //  } 
    //  if (i<W)
    //    C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    PXOR_RR (5,5);
    for(i=0;i<W;i+=4) {
      // reg 4 = Q
      PXOR_RR (4,4);
      
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2S(I1, I2, i+d_2,j,i+d,j+e);
        //printf("I1_I2(%d,%d)(%d,%d):",i+d_2,j,i+d,j);
        //printf("P(%d,%d,%d):",i,j,d);
        //PRINTREG16(1,"P");

        
        // Add curr P to Q
        PADDW_RR (1,4);
      }
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[d])[i]);
      //printf("Q(%d,%d):",i,d);
      // PRINTREG16(4,"Q");
    
      // Add curr Q to C
      PADDW_RR (4,5);
    }
    // Store C
    MOVQ_RM (5,((MM_U_16 *)C->p[d])[0]);
   
    // compute for i=W
    // reg 4 = Q
    PXOR_RR (4,4);  
    // window down image
    for (j=0;j<W;j++) {
      // lbw in 0
      I1_I2S(I1, I2, W+d_2,j,W+d,j+e);

      MOVQ_RM (1,((MM_U_16 *)P[j]->p[d])[W]);
      // Add curr P to Q
      PADDW_RR (1,4);
    }
    // Store Q
    MOVQ_RM (4,((MM_U_16 *)Q->p[d])[W]);

    
  }
  else if (x<0) {
    
    // Equivalent C function
    //C[d][0]=0;
    //for(i=0;i<=W;i++) {
    // P_temp = I1_I2(I1, I2, i+d_2,y+W,i+d,y+W);
    // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
    // P[y%W][d][i] = P_temp;
    // if (i<W)
    //   C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    PXOR_RR (5,5);
    for(i=0;i<W;i+=4) {
      // reg 4 = Q
      MOVQ_MR (((MM_U_16 *)Q->p[d])[i],4);  
      // Sub prev P from Q
      PSUBW_MR (((MM_U_16 *)P[y%W]->p[d])[i],4);
      // Add curr P to Q
      // lbw in 0
      I1_I2S(I1, I2, i+d_2,y+W,i+d,y+W+e);
      //PRINTREG16(1,"mmx1");
      // P_temp is reg 1
      PADDW_RR (1,4);
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[d])[i]);
      // Store P
      MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[d])[i]);
      // Add curr Q to C
      PADDW_RR (4,5);
    }
    // Store C[d][0]
    MOVQ_RM (5,((MM_U_16 *)C->p[d])[0]);
    //PRINTREG16(5,"C");


    // Do for i=W
    // reg 4 = Q
    MOVQ_MR (((MM_U_16 *)Q->p[d])[W],4);  
      // Sub prev P from Q
    PSUBW_MR (((MM_U_16 *)P[y%W]->p[d])[W],4);
    // Add curr P to Q
    // lbw in 0
    I1_I2S(I1, I2, W+d_2,y+W,W+d,y+W+e);
    //PRINTREG16(1,"mmx1");
    // P_temp is reg 1
    PADDW_RR (1,4);
    // Store Q
    MOVQ_RM (4,((MM_U_16 *)Q->p[d])[W]);
    // Store P
    MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[d])[W]);
  }
  else
  {
    if (y<0) {
      // Equivalent C function
      //Q[x+W][d] = 0;
      //for (j=0;j<W;j++) {
      //  P[x+W][j][d]= I1_I2(I1, I2, x+W+d_2,j,x+W+d,j);
      //  Q[x+W][d] += P[x+W][j][d];
      //}
      
      // reg 4 = Q
      PXOR_RR (4,4);
      
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2S(I1, I2,x+W+d_2,j,x+W+d,j+e);
        //PRINTREG16(1,"mmx1");
        MOVQ_RM (1,((MM_U_16 *)P[j]->p[d])[x+W]);
        // Add curr P to Q
        PADDW_RR (1,4);
      }
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[d])[x+W]);
      
    }  
    else
    {
      // Equivalent C function
      // P is [N][W][D] the W dimension is a wrap around buffer.
      //P_temp = I1_I2(I1, I2, x+W+d_2,y+W,x+W+d,y+W);
      //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
      //P[x+W][y%W][d] = P_temp;
      
      // reg 4 = Q
      MOVQ_MR (((MM_U_16 *)Q->p[d])[x+W],4);
      // subtract prev P
      PSUBW_MR (((MM_U_16 *)P[y%W]->p[d])[x+W],4);
      // Add curr P to Q
      // lbw in 0
      I1_I2S(I1, I2,x+W+d_2,y+W,x+W+d,y+W+e);
      //PRINTREG16(1,"mmx1");
      // P_temp is reg 1
      PADDW_RR (1,4);
        
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[d])[x+W]);
      
      // Store P
      MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[d])[x+W]);

    }
    // Equivalent C function
    //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
    
    // reg 5 = C
    MOVQ_MR (((MM_U_16 *)C->p[d])[x],5);  
    //printf("(%d,%d,%d):",x,y,d);
    //PRINTREG16(4,"Q");
    //PRINTREG16(5,"C");

    // Sub curr Q[x][d] from C
    PSUBW_MR (((MM_U_16 *)Q->p[d])[x],5);
    //PRINTREG16(5,"C-P");
      
    // Add Q[x+W][d] to C
    MOVQ_MR (((MM_U_16 *)Q->p[d])[x+W],4);
    PADDW_RR (4,5);
    //PRINTREG16(5,"C-P+P");
      
    // Store C[x+1][d]
    MOVQ_RM (5,((MM_U_16 *)C->p[d])[x+4]);
    
  }
  
  // Equivalent C function
  //if ((d==0)||(C->p[d][x+4] < min)) {
  //  min = C[x+1][d];
  //  min_d = d;
  //}
  
  curr_d.mmu16[0] = d;
  curr_d.mmu16[1] = d;
  curr_d.mmu16[2] = d;
  curr_d.mmu16[3] = d;
    

  // reg 5 = C 
  //MOVQ_MR(((MM_U_16 *)C->p[d])[x+4],5);
        MOVQ_RR(5,0);


  // reg 6 = minimum C
        MOVQ_MR(min,6);
  MOVQ_RR(6,2);
   MOVQ_MR(sign_offset,1);
  PADDW_RR(1,2);
  PADDW_RR(1,0);
  //PRINTREG16(2,"min");
  //PRINTREG16(0,"C");
        
  // reg 2 becomes mask
        PCMPGTW_RR(0,2);
  MOVQ_RR(2,1);

  //PRINTREG16(2,"mask");
  
  // reg 0 is C values smaller than minimum
  MOVQ_RR(5,0);
  PAND_RR(2,0);
  
  
  // reg 6 is preserved minimum
  PANDN_RR(6,1);
  
  // reg 1 is new minimum
  POR_RR(0,1);
  MOVQ_RM(1,min);
  //PRINTREG16(6,"new_min");


  // reg 6 is d values for minimum
  MOVQ_MR(min_d,6);
  // reg 1 is preserved d values
        MOVQ_RR(2,1)
  PANDN_RR(6,1);

  // reg 0 = current d value
  MOVQ_MR(curr_d,0);
  // reg 0 = current d value for updated minimum
  PAND_RR(2,0);
  
  // reg 6 = new d's for minimum values
  POR_RR(0,1);
  MOVQ_RM(1,min_d);
  //PRINTREG16(1,"new_min_d");

      }// for d
      MOVQ_RR(1,0);
      PACKUSWB_RR(0,1);
      MOVQ_RM(1,((MM_U_8 *)Out->p[y+1])[x+4]);

      //((MM_U_8 *)Out->p[y+1])[x+4] = min_d.mmu16[0];
      //((MM_U_8 *)Out->p[y+1])[x+5] = min_d.mmu16[1];
      //((MM_U_8 *)Out->p[y+1])[x+6] = min_d.mmu16[2];
      //((MM_U_8 *)Out->p[y+1])[x+7] = min_d.mmu16[3];

    }// for x
  }// for y

  EMMS;
}


//----------------------------------------------------------------
int MMXRecursive_Depthmap_SAD2(Matrix *I1, Matrix *I2, Matrix  *Out)
{
  
  const int W = 16;
  const int D = 32;
  const int E = 8;

  int N,Y,X;


  register short i, j;
  register short d,e,x,y;
  
  short w_2 = W/2;
  short d_2 = D/2;
  short e_2 = E/2;

  unsigned short overall_min;
  unsigned char overall_min_d;


  Matrix **P;
  Matrix *Q;
  Matrix *C;
  mmxdata sign_offset;
  mmxdata plus_four;
  mmxdata min;
  mmxdata curr_d;
  mmxdata min_d;
  

  X = I1->cols;
  Y = I1->rows;
  N = I1->cols;


  Q = MatrixAlloc(MMT_U_16, N, D);
  C = MatrixAlloc(MMT_U_16, N, D);

  P = (Matrix **) Alloc(sizeof(Matrix *)*W); 
  for (i=0;i<W;i++)
    P[i] = MatrixAlloc(MMT_U_16, N, D);


  sign_offset.mmu32[0] =0x80008000;
  sign_offset.mmu32[1] =0x80008000;
  plus_four.mmu32[0] =0x00040004;
  plus_four.mmu32[1] =0x00040004;



  EMMS;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    
    for(x=-1;x<N-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);

      min.mmu32[0] = 0xffffffff;
      min.mmu32[1] = 0xffffffff;
      curr_d.mmu16[0] = 0;
      curr_d.mmu16[1] = 1;
      curr_d.mmu16[2] = 2;
      curr_d.mmu16[3] = 3;
   
      // clear registers
      //PXOR_RR (3,3);
      // PXOR_RR (4,4); 
    
      PXOR_RR (7,7); 

      for (d=0;d<D;d+=4) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if ((x<0)&&(y<0)) {      
    // Equivalent C function
    //C[d][0]=0; 
    //for(i=0;i<=W;i+=4) {
    //  Q[d][i] = 0;
    // window down image
    //  for (j=0;j<W;j++) {
    //  P[j][d][i]= I1_I2(I1, I2,i+d_2,j,i+d,j);
    //  Q[d][i] += P[j][d][i];
    //  } 
    //  if (i<W)
    //    C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    PXOR_RR (5,5);
    for(i=0;i<W;i++) {
      // reg 4 = Q
      PXOR_RR (4,4);
      
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2D(I1, I2, i+d_2,j,i+d,j);
        //printf("I1_I2(%d,%d)(%d,%d):",i+d_2,j,i+d,j);
        //printf("P(%d,%d,%d):",i,j,d);
        //PRINTREG16(1,"P");

        
        // Add curr P to Q
        PADDW_RR (1,4);
      }
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[i])[d]);
      //printf("Q(%d,%d):",i,d);
      // PRINTREG16(4,"Q");
    
      // Add curr Q to C
      PADDW_RR (4,5);
    }
    // Store C
    MOVQ_RM (5,((MM_U_16 *)C->p[0])[d]);
   
    // compute for i=W
    // reg 4 = Q
    PXOR_RR (4,4);  
    // window down image
    for (j=0;j<W;j++) {
      // lbw in 0
      I1_I2D(I1, I2, W+d_2,j,W+d,j);

      MOVQ_RM (1,((MM_U_16 *)P[j]->p[W])[d]);
      // Add curr P to Q
      PADDW_RR (1,4);
    }
    // Store Q
    MOVQ_RM (4,((MM_U_16 *)Q->p[W])[d]);

    
  }
  else if (x<0) {
    
    // Equivalent C function
    //C[d][0]=0;
    //for(i=0;i<=W;i++) {
    // P_temp = I1_I2(I1, I2, i+d_2,y+W,i+d,y+W);
    // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
    // P[y%W][d][i] = P_temp;
    // if (i<W)
    //   C[d][0] += Q[d][i];
    //}
    
    // reg 5 = C
    PXOR_RR (5,5);
    for(i=0;i<W;i++) {
      // reg 4 = Q
      MOVQ_MR (((MM_U_16 *)Q->p[i])[d],4);  
      // Sub prev P from Q
      PSUBW_MR (((MM_U_16 *)P[y%W]->p[i])[d],4);
      // Add curr P to Q
      // lbw in 0
      I1_I2D(I1, I2, i+d_2,y+W,i+d,y+W);
      //PRINTREG16(1,"mmx1");
      // P_temp is reg 1
      PADDW_RR (1,4);
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[i])[d]);
      // Store P
      MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[i])[d]);
      // Add curr Q to C
      PADDW_RR (4,5);
    }
    // Store C[d][0]
    MOVQ_RM (5,((MM_U_16 *)C->p[0])[d]);
    //PRINTREG16(5,"C");


    // Do for i=W
    // reg 4 = Q
    MOVQ_MR (((MM_U_16 *)Q->p[W])[d],4);  
      // Sub prev P from Q
    PSUBW_MR (((MM_U_16 *)P[y%W]->p[W])[d],4);
    // Add curr P to Q
    // lbw in 0
    I1_I2D(I1, I2, W+d_2,y+W,W+d,y+W);
    //PRINTREG16(1,"mmx1");
    // P_temp is reg 1
    PADDW_RR (1,4);
    // Store Q
    MOVQ_RM (4,((MM_U_16 *)Q->p[W])[d]);
    // Store P
    MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[W])[d]);
  }
  else
  {
    if (y<0) {
      // Equivalent C function
      //Q[x+W][d] = 0;
      //for (j=0;j<W;j++) {
      //  P[x+W][j][d]= I1_I2(I1, I2, x+W+d_2,j,x+W+d,j);
      //  Q[x+W][d] += P[x+W][j][d];
      //}
      
      // reg 4 = Q
      PXOR_RR (4,4);
      
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2D(I1, I2,x+W+d_2,j,x+W+d,j);
        //PRINTREG16(1,"mmx1");
        MOVQ_RM (1,((MM_U_16 *)P[j]->p[x+W])[d]);
        // Add curr P to Q
        PADDW_RR (1,4);
      }
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[x+W])[d]);
      
    }  
    else
    {
      // Equivalent C function
      // P is [N][W][D] the W dimension is a wrap around buffer.
      //P_temp = I1_I2(I1, I2, x+W+d_2,y+W,x+W+d,y+W);
      //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
      //P[x+W][y%W][d] = P_temp;
      
      // reg 4 = Q
      MOVQ_MR (((MM_U_16 *)Q->p[x+W])[d],4);
      // subtract prev P
      PSUBW_MR (((MM_U_16 *)P[y%W]->p[x+W])[d],4);
      // Add curr P to Q
      // lbw in 0
      I1_I2D(I1, I2,x+W+d_2,y+W,x+W+d,y+W);
      //PRINTREG16(1,"mmx1");
      // P_temp is reg 1
      PADDW_RR (1,4);
        
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[x+W])[d]);
      
      // Store P
      MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[x+W])[d]);

    }
    // Equivalent C function
    //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
    
    // reg 5 = C
    MOVQ_MR (((MM_U_16 *)C->p[x])[d],5);  
    //printf("(%d,%d,%d):",x,y,d);
    //PRINTREG16(4,"Q");
    //PRINTREG16(5,"C");

    // Sub curr Q[x][d] from C
    PSUBW_MR (((MM_U_16 *)Q->p[x])[d],5);
    //PRINTREG16(5,"C-P");
      
    // Add Q[x+W][d] to C
    MOVQ_MR (((MM_U_16 *)Q->p[x+W])[d],4);
    PADDW_RR (4,5);
    //PRINTREG16(5,"C-P+P");
      
    // Store C[x+1][d]
    MOVQ_RM (5,((MM_U_16 *)C->p[x+1])[d]);
    
  }
          
  // reg 5 = C 
        MOVQ_RR(5,0);

  //reg 4 = minimum C
  MOVQ_MR(min,4);
  MOVQ_RR(4,2);
   MOVQ_MR(sign_offset,1);
  PADDW_RR(1,2);
  PADDW_RR(1,0);
  //PRINTREG16(2,"min");
  //PRINTREG16(0,"C");
        
  // reg 2 becomes mask
        PCMPGTW_RR(0,2);
  MOVQ_RR(2,1);
  
  //PRINTREG16(2,"mask");
  
  // reg 0 is C values smaller than minimum
  MOVQ_RR(5,0);
  PAND_RR(2,0);
    
  // reg 4 is preserved minimum
  PANDN_RR(4,1);
  
  // reg 1 is new minimum
  POR_RR(0,1);
  MOVQ_RM(1,min);
  //PRINTREG16(6,"new_min");

  // reg 4 is d values for minimum
  MOVQ_MR(min_d,4);
  // reg 1 is preserved d values
        MOVQ_RR(2,1)
  PANDN_RR(4,1);

  // reg 0 = current d value
  // increase current d values by 4
  MOVQ_MR(curr_d,0);
         PADDW_MR(plus_four,0);
  MOVQ_RM(0,curr_d);
  // reg 0 = current d value for updated minimum
  PAND_RR(2,0);
  
  // reg 1 = new d's for minimum values
  POR_RR(0,1);
  MOVQ_RM(1,min_d);
  //PRINTREG16(1,"new_min_d");

      }// for d
            
      overall_min = min.mmu16[0];
      overall_min_d = min_d.mmu16[0];
      if (overall_min > min_d.mmu16[1]) {
  overall_min = min.mmu16[1];
  overall_min_d = min_d.mmu16[1];
      }
      if (overall_min > min.mmu16[2]) {
  overall_min = min.mmu16[2];
  overall_min_d = min_d.mmu16[2];
      }
      if (overall_min > min.mmu16[3]) {
  overall_min = min.mmu16[3];
  overall_min_d = min_d.mmu16[3];
      }
      
      ((MM_U_8 *)Out->p[y+1])[x+1] = overall_min_d;
      
    }// for x
  }// for y

  EMMS;
}

//----------------------------------------------------------------
int MMXRecursive_Depthmap_FLOW(Matrix *I1, Matrix *I2, Matrix  *Out)
{
  
  const int W = 16;
  const int D = 32;
  const int E = 8;

  int N,Y,X;

  register short i, j;
  register short d,e,x,y;

  unsigned char min_d;
  unsigned short min;
  
  short w_2 = W/2;
  short d_2 = D/2;
  int e_2 = E/2;

  Matrix **P;
  Matrix *Q;
  Matrix *Q1;
  Matrix *Q2;
  Matrix *C;
  

  X = I1->cols;
  Y = I1->rows;
  N = I1->cols;


  Q = MatrixAlloc(MMT_U_16, N, D);
  Q1 = MatrixAlloc(MMT_U_16, N, D);
  Q2 = MatrixAlloc(MMT_U_16, N, D);
  C = MatrixAlloc(MMT_U_16, N, D);

  P = (Matrix **) Alloc(sizeof(Matrix *)*W); 
  for (i=0;i<W;i++)
    P[i] = MatrixAlloc(MMT_U_16, N, D);


  EMMS;

  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    
    for(x=-1;x<N-W-D-1;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);

      // clear registers
      //PXOR_RR (3,3);
      // PXOR_RR (4,4); 
      PXOR_RR (7,7); 
      for (e=0;e<E;e+=4) {

  for (d=0;d<D;d+=4) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    if ((x<0)&&(y<0)) {      
      // Equivalent C function
      //C[d][0]=0; 
      //for(i=0;i<=W;i+=4) {
      //  Q[d][i] = 0;
      // window down image
      //  for (j=0;j<W;j++) {
      //  P[j][d][i]= I1_I2(I1, I2,i+d_2,j,i+d,j);
      //  Q[d][i] += P[j][d][i];
      //  } 
      //  if (i<W)
      //    C[d][0] += Q[d][i];
      //}
      
      // reg 5 = C
      PXOR_RR (5,5);
      for(i=0;i<W;i++) {
        // reg 4 = Q
        PXOR_RR (4,4);
        
        // window down image
        for (j=0;j<W;j++) {
    // lbw in 0
    I1_I2D(I1, I2, i+d_2,j,i+d,j);
    //printf("I1_I2(%d,%d)(%d,%d):",i+d_2,j,i+d,j);
    //printf("P(%d,%d,%d):",i,j,d);
    //PRINTREG16(1,"P");
    
    // Add curr P to Q
    PADDW_RR (1,4);
        }
        // Store Q
        MOVQ_RM (4,((MM_U_16 *)Q->p[i])[d]);
        //printf("Q(%d,%d):",i,d);
        // PRINTREG16(4,"Q");
    
        // Add curr Q to C
        PADDW_RR (4,5);
      }
      // Store C
      MOVQ_RM (5,((MM_U_16 *)C->p[0])[d]);
      
      // compute for i=W
      // reg 4 = Q
      PXOR_RR (4,4);  
      // window down image
      for (j=0;j<W;j++) {
        // lbw in 0
        I1_I2D(I1, I2, W+d_2,j,W+d,j);
        
        MOVQ_RM (1,((MM_U_16 *)P[j]->p[W])[d]);
        // Add curr P to Q
        PADDW_RR (1,4);
      }
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[W])[d]);
    
    }
    else if (x<0) {
    
      // Equivalent C function
      //C[d][0]=0;
      //for(i=0;i<=W;i++) {
      // P_temp = I1_I2(I1, I2, i+d_2,y+W,i+d,y+W);
      // Q[d][i] = Q[d][i] - P[y%W][d][i] + P_temp ;
      // P[y%W][d][i] = P_temp;
      // if (i<W)
      //   C[d][0] += Q[d][i];
      //}
      
      // reg 5 = C
      PXOR_RR (5,5);
      for(i=0;i<W;i++) {
        // reg 4 = Q
        MOVQ_MR (((MM_U_16 *)Q->p[i])[d],4);  
        // Sub prev P from Q
        PSUBW_MR (((MM_U_16 *)P[y%W]->p[i])[d],4);
        // Add curr P to Q
        // lbw in 0
        I1_I2D(I1, I2, i+d_2,y+W,i+d,y+W);
        //PRINTREG16(1,"mmx1");
        // P_temp is reg 1
        PADDW_RR (1,4);
        // Store Q
        MOVQ_RM (4,((MM_U_16 *)Q->p[i])[d]);
        // Store P
        MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[i])[d]);
        // Add curr Q to C
        PADDW_RR (4,5);
      }
      // Store C[d][0]
      MOVQ_RM (5,((MM_U_16 *)C->p[0])[d]);
      //PRINTREG16(5,"C");
      
      
      // Do for i=W
      // reg 4 = Q
      MOVQ_MR (((MM_U_16 *)Q->p[W])[d],4);  
      // Sub prev P from Q
      PSUBW_MR (((MM_U_16 *)P[y%W]->p[W])[d],4);
      // Add curr P to Q
      // lbw in 0
      I1_I2D(I1, I2, W+d_2,y+W,W+d,y+W);
      //PRINTREG16(1,"mmx1");
      // P_temp is reg 1
      PADDW_RR (1,4);
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[W])[d]);
      // Store P
      MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[W])[d]);
    }
    else
      {
        if (y<0) {
    // Equivalent C function
    //Q[x+W][d] = 0;
    //for (j=0;j<W;j++) {
    //  P[x+W][j][d]= I1_I2(I1, I2, x+W+d_2,j,x+W+d,j);
    //  Q[x+W][d] += P[x+W][j][d];
    //}
    
    // reg 4 = Q
    PXOR_RR (4,4);
    
    // window down image
    for (j=0;j<W;j++) {
      // lbw in 0
      I1_I2D(I1, I2,x+W+d_2,j,x+W+d,j);
      //PRINTREG16(1,"mmx1");
      MOVQ_RM (1,((MM_U_16 *)P[j]->p[x+W])[d]);
      // Add curr P to Q
      PADDW_RR (1,4);
    }
    // Store Q
    MOVQ_RM (4,((MM_U_16 *)Q->p[x+W])[d]);
    
        }  
        else
    {
      // Equivalent C function
      // P is [N][W][D] the W dimension is a wrap around buffer.
      //P_temp = I1_I2(I1, I2, x+W+d_2,y+W,x+W+d,y+W);
      //Q[x+W][d] = Q[x+W][d] - P[x+W][y%W][d] + P_temp ;
      //P[x+W][y%W][d] = P_temp;
      
      // reg 4 = Q
      MOVQ_MR (((MM_U_16 *)Q->p[x+W])[d],4);
      // subtract prev P
      PSUBW_MR (((MM_U_16 *)P[y%W]->p[x+W])[d],4);
      // Add curr P to Q
      // lbw in 0
      I1_I2D(I1, I2,x+W+d_2,y+W,x+W+d,y+W);
      //PRINTREG16(1,"mmx1");
      // P_temp is reg 1
      PADDW_RR (1,4);
      
      // Store Q
      MOVQ_RM (4,((MM_U_16 *)Q->p[x+W])[d]);
      
      // Store P
      MOVQ_RM (1,((MM_U_16 *)P[y%W]->p[x+W])[d]);
      
    }
        // Equivalent C function
        //C[d][x+4] = C[d][x] - Q[d][x] + Q[d][x+W];
        
        // reg 5 = C
        MOVQ_MR (((MM_U_16 *)C->p[x])[d],5);  
        //printf("(%d,%d,%d):",x,y,d);
        //PRINTREG16(4,"Q");
        //PRINTREG16(5,"C");
        
        // Sub curr Q[x][d] from C
        PSUBW_MR (((MM_U_16 *)Q->p[x])[d],5);
        //PRINTREG16(5,"C-P");
        
        // Add Q[x+W][d] to C
        MOVQ_MR (((MM_U_16 *)Q->p[x+W])[d],4);
        PADDW_RR (4,5);
        //PRINTREG16(5,"C-P+P");
        
        // Store C[x+1][d]
        MOVQ_RM (5,((MM_U_16 *)C->p[x+1])[d]);
        
      }
    
    // Equivalent C function
    //if ((d==0)||(C->p[d][x+1] < min)) {
    //  min = C[x+1][d];
    //  min_d = d;
    //}
    
    if ((d==0)||(((MM_U_16 *)C->p[x+1])[d] < min)) {
      min = ((MM_U_16 *)C->p[x+1])[d];
      min_d = d;
    }
    if (((MM_U_16 *)C->p[x+1])[d+1] < min) {
      min = ((MM_U_16 *)C->p[x+1])[d+1];
      min_d = d+1;
    }
    if (((MM_U_16 *)C->p[x+1])[d+2] < min) {
      min = ((MM_U_16 *)C->p[x+1])[d+2];
      min_d = d+2;
    }
    if (((MM_U_16 *)C->p[x+1])[d+3] < min) {
      min = ((MM_U_16 *)C->p[x+1])[d+3];
      min_d = d+3;
    }
    
    
  }// for d
  ((MM_U_8 *)Out->p[y+1])[x+1] = min_d;
      } // for e
      }// for x
    }// for y
    
  EMMS;
}


//----------------------------------------------------------------
int Depthmap_NCC(Matrix *I1, Matrix *I2, Matrix  *O)
{
  // const int N = 128;

  const int W = 16;
  const int D = 32;
  const int E = 8;

  int Y,X;
  register int d,e,x,y,i,j;

  double result, best=0.0, num, tmp;
  double I1_bar, I1_hat, I1_tilda;
  double I2_bar, I2_hat, I2_tilda;
  
  int w_2 = W/2;
  int d_2 = D/2;
  int e_2 = E/2;

  int best_e=0, best_d=0;
  
  for(y=0;y<Y;y++){
    fprintf(stderr,"depthmap: y:%.0f% \n",(float)100*y/Y);

    for(x=0;x<X;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
  
      if ((y<(e_2+w_2))||(x<(d_2+w_2))||(y>(Y-1-e_2-w_2))||(x>(X-1-d_2-w_2))) {
  ((MM_U_8 *)O->p[y])[x]=0;
        best_e=0; 
        best_d=0;
        best=0.0;

      }
      else
      {

  //fprintf(stderr,"depthmap: d:%d \n",d);
  I1_bar =0.0;
  for(j=0;j<W;j++) {
    //fprintf(stderr,"depthmap: j:%d \n",j);
    for(i=0;i<W;i++) {
      //fprintf(stderr,"depthmap: i:%d \n",i);
      I1_bar += ((MM_U_8 *)I1->p[y-w_2+j])[x-w_2+i];
            //fprintf(stderr,"depthmap_NCC: %f \n",I1_bar);
      
    }    
  }
  I1_bar /= (W*W);
  
  for (d=0;d<D;d++) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    for (e=0;e<E;e++) {
      //fprintf(stderr,"depthmap: d:%d \n",d);
      I2_bar =0.0;
      for(j=0;j<W;j++) {
        //fprintf(stderr,"depthmap: j:%d \n",j);
        for(i=0;i<W;i++) {
    //fprintf(stderr,"depthmap: i:%d \n",i);
    I2_bar += ((MM_U_8 *)I2->p[y+e-e_2-w_2+j])[x+d-d_2-w_2+i];
    //  fprintf(stderr,"depthmap_NCC: %f \n",I2_bar);

        }    
      }
      I2_bar /= (W*W);
      //fprintf(stderr,"depthmap_NCC: %f %f\n",I1_bar,I2_bar);

      I1_hat =0.0;
      I1_tilda =0.0;
      I2_hat =0.0;
      I2_tilda =0.0;
      num = 0.0;
      for(j=0;j<W;j++) {
        //fprintf(stderr,"depthmap: j:%d \n",j);
        for(i=0;i<W;i++) {
    //fprintf(stderr,"depthmap: i:%d \n",i);
    I1_hat = - I1_bar + ((MM_U_8 *)I1->p[y-w_2+j])[x-w_2+i];
    I1_tilda += I1_hat * I1_hat; 

    I2_hat = - I2_bar + ((MM_U_8 *)I2->p[y+e-e_2-w_2+j])[x+d-d_2-w_2+i];
    I2_tilda += I2_hat * I2_hat;

    num += I1_hat * I2_hat;
        }    
      }
      result = num * num / I1_tilda*I2_tilda; 
      //      result = num / sqrt(I1_tilda*I2_tilda); 
      //printf("depthmap_NCC: %f %f %f %f \n",num, I1_tilda,I2_tilda,result);

      if ((d==0)&&(e==0)||(fabs(result) > best)) {
        best = fabs(result);
        ((MM_U_8 *)O->p[y])[x]= d;
        best_e = e;
        best_d = d;
      } // if
    } // e
  } // d
      } // if
      //printf("%d %d %d %d %f\n",x,y,best_d,best_e,best);
    } // x
  } // y
  
}


//----------------------------------------------------------------
int Recursive_Depthmap_NCC(Matrix *I1, Matrix *I2, Matrix  *Out)
{
  
  const int W = 16;
  const int D = 32;
  const int E = 8;

  int Y,X;

  register int d,e,x,y;
  register int i, j, k;
  
  int w_2 = W/2;
  int d_2 = D/2;
  int e_2 = E/2;

  int P[X][W][D];
  int Q[X][D];
  int N[X][D];

  int R[X][W];
  int S[X];
  int M[X];

  double C, max;

  X = I1->cols;
  Y = I1->rows;

  // compute M for x =0 y=0
  // M[0] -> M[D]
  for (k=0;k<=D;k++){
    //fprintf(stderr,"depthmap: k:%d \n",k);
    M[k]=0;
    // window across image
    for(i=k;i<k+W;i++) {
      S[i]=0;
      // window down image
      for (j=0;j<W;j++) {
  R[i][j]= ((MM_U_8 *)I2->p[j])[i] * ((MM_U_8 *)I2->p[j])[i];
  S[i] += R[i][j];
      } 
      M[k] += S[i];
    }
  }

  // compute N for x=0 & y=0
  for (d=0;d<D;d++) {
    fprintf(stderr,"depthmap: d:%d \n",d);
    N[0][d]=0;
    // window across image
    for(i=0;i<=W;i++) {
      Q[i][d] = 0;
      // window down image
      for (j=0;j<W;j++) {
  P[i][j][d]= ((MM_U_8 *)I1->p[j])[i+d_2] * ((MM_U_8 *)I2->p[j])[i+d];
  Q[i][d] += P[i][j][d];       
      } 
      if (i<W)
  N[0][d] += Q[i][d];
    }
  }

  //--- main Y loop
  for(y=-1;y<Y-W-1;y++){
    fprintf(stdout,"depthmap: y:%d \n",y);

    if (y>=0) {
      // compute M for x=0
      for (k=0;k<=D;k++){
  fprintf(stderr,"depthmap: k:%d \n",k);
  M[k]=0;
  // window across image
  for(i=k;i<k+W;i++) {
    S[i] -= R[i][y%W];
    R[i][y%W] = ((MM_U_8 *)I2->p[y+W])[i] * ((MM_U_8 *)I2->p[y+W])[i];
    S[i] += R[i][y%W];
  
    M[k] += S[i];
  }
      }
    }

    //--- main x loop
    for(x=-1;x<X-W-1-D;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);

      // compute M for y =0 
      if (y<0) {
  S[x+D+W] = 0;
  for (j=0;j<W;j++) {
    R[x+D+W][j]= ((MM_U_8 *)I2->p[j])[x+D+W] * ((MM_U_8 *)I2->p[j])[x+D+W];
    S[x+D+W] += R[x+D+W][j];
  }
      }  
      else
      {
  // P is [X][W] the W dimension is a wrap around buffer.    
  S[x+D+W] -= R[x+D+W][y%W];
  R[x+D+W][y%W] = ((MM_U_8 *)I2->p[y+W])[x+D+W] * ((MM_U_8 *)I2->p[y+W])[x+D+W];
  S[x+D+W] += R[x+D+W][y%W];
      }
  
      M[x+D+1] = M[x+D] - S[x+D] + S[x+D+W];
     
      //--- main D loop
      for (d=0;d<D;d++) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if (x<0) {
    if (y>=0) {
      N[0][d]=0;
      for(i=0;i<W;i++) {
        Q[i][d] -= P[i][y%W][d];
        P[i][y%W][d] = ((MM_U_8 *)I1->p[y+W])[i+d_2] * ((MM_U_8 *)I2->p[y+W])[i+d];
        Q[i][d] += P[i][y%W][d];
  
        N[0][d] += Q[i][d];
      }
      Q[W][d] -= P[W][y%W][d];
      P[W][y%W][d] = ((MM_U_8 *)I1->p[y+W])[W+d_2] * ((MM_U_8 *)I2->p[y+W])[W+d];
      Q[W][d] += P[W][y%W][d];
    }
  }
  else
  {
    if (y<0) {
      Q[x+W][d] = 0;
      for (j=0;j<W;j++) {
        P[x+W][j][d]= ((MM_U_8 *)I1->p[j])[x+W+d_2] * ((MM_U_8 *)I2->p[j])[x+W+d];
        Q[x+W][d] += P[x+W][j][d];
      }
    }  
    else
    {
      // P is [X][W][D] the W dimension is a wrap around buffer.
      Q[x+W][d] -= P[x+W][y%W][d];
      P[x+W][y%W][d] = ((MM_U_8 *)I1->p[y+W])[x+W+d_2] * ((MM_U_8 *)I2->p[y+W])[x+W+d];
      Q[x+W][d] += P[x+W][y%W][d];
    }
    N[x+1][d] = N[x][d] - Q[x][d] + Q[x+W][d];
  }

  C = (double) N[x+1][d]/sqrt(M[x+d+1]);

  //printf("%d,%d,%d: %f\n",x,y,d, C);

  if ((d==0)||(C > max)) {
    max = C;
    ((MM_U_8 *)Out->p[y+1])[x+1] = d;
  }
      } // for d
    }// for x
  }// for y

}

//----------------------------------------------------------------
int MMXRecursive_Depthmap_NCC(Matrix *I1, Matrix *I2, Matrix  *Out)
{

  static int allocated = 0;
  int Y;
  int X;

  const int W = 16;
  const int D = 32;
  const int E = 0;

  short w_2 = W/2;
  short d_2 = D/2;
  short e_2 = E/2;


  register int i, j, k;
  register int d,e,x,y;

  register int y_W, x_W, yMODW;

#ifdef FAST_SQRT
  unsigned long sqrt2,sqrt3;
#endif

  mmxdata curr, curr_d, max, max_d;

  static  Matrix **P, *Q, *N;
  static Matrix *R;
  static MM_U_32 *S, *M, *MINV;

  X = I1->cols;
  Y = I1->rows;
  

  if ( !allocated) {
    // Numerator matrix
    P = (Matrix **) Alloc(sizeof(Matrix *)*W); 
    for (i=0;i<W;i++)
      P[i] = MatrixAlloc(MMT_U_32, D, X);
    Q = MatrixAlloc(MMT_U_32, D, X);
    N = MatrixAlloc(MMT_U_32, D, X);

    // Denominator matrix
    S = (MM_U_32 *) Alloc(sizeof(MM_U_32)*X); 
    M = (MM_U_32 *) Alloc(sizeof(MM_U_32)*X); 
    R = MatrixAlloc(MMT_U_32, W, X);
    MINV = (MM_U_32 *) Alloc(sizeof(MM_U_32)*(X)); 
    allocated = 1;
  }

  EMMS;
  // clear reg 7
  PXOR_RR (7,7);
 
  // compute M for x=0 y=0
  // M[0] -> M[D]
  // S[0] -> S[D+W-2]
  for (k=0;k<=D;k+=2){
    //fprintf(stderr,"depthmap: k:%d \n",k);
    //M[k]=0;
    //// window across image
    //for(i=k;i<k+W;i++) {
    //  S[i]=0;
    //  // window down image
    //  for (j=0;j<W;j++) {
    //    R[i][j]= ((MM_U_8 *)I2->p[j])[i] * ((MM_U_8 *)I2->p[j])[i];
    //    S[i] += R[i][j];
    //  } 
    //  M[k] += S[i];
    //}

    // reg 6 = M
    PXOR_RR (6,6);
    for(i=k;i<k+W;i+=2) {
      // reg 5 = S
      PXOR_RR (5,5);
      for(j=0;j<W;j++) {
  // calc R
  I1_I2MULTISUM(I2, I2, i,j,i,j);
  // Store new R
  MOVQ_RM (0,((MM_U_32 *)R->p[j])[i]);
  // Add R to S
  PADDD_RR (0,5);
      }
      // Store new S
      MOVQ_RM (5,((MM_U_32 *)S)[i]);
      // Add curr S to M
      PADDD_RR (5,6);
    }
    // Store M[0]
    MOVQ_RM (6,((MM_U_32 *)M)[k]);    

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
  // N->p[0][0]  -> N->p[D-1][0]
  for (d=0;d<D;d++) {
    //fprintf(stderr,"depthmap: d:%d \n",d);
    //N[0][d]=0;
    //// window across image
    //for(i=0;i<=W;i++) {
    //  Q[i][d] = 0;
    //  // window down image
    // for (j=0;j<W;j++) {
    //   P[i][j][d]= ((MM_U_8 *)I1->p[j])[i+d_2] * ((MM_U_8 *)I2->p[j])[i+d];
    //   Q[i][d] += P[i][j][d];       
    // } 
    // if (i<W)
    //   N[0][d] += Q[i][d];
    //}

    // reg 6 = N
    PXOR_RR (6,6);
    for(i=0;i<W;i+=2) {
      // reg 5 = Q
      PXOR_RR (5,5);
      for (j=0;j<W;j++) {
  // calc new P   
  I1_I2MULTISUM(I1, I2, i+d_2,j,i+d,j);
  // Store P
  MOVQ_RM (0,((MM_U_32 *)P[j]->p[d])[i]);
  // Add P to Q
  PADDD_RR (0,5);
      }
      // Store Q
      MOVQ_RM (5,((MM_U_32 *)Q->p[d])[i]);
      // Add curr Q to N
      PADDD_RR (5,6);
    }
    // Store N[d][0]
    MOVQ_RM (6,((MM_U_32 *)N->p[d])[0]);
    
    // Do for i=W
    // reg 5 = Q
    PXOR_RR (5,5);
    for (j=0;j<W;j++) {
      // calc new P  
      I1_I2MULTISUM(I1, I2, W+d_2,j,W+d,j);
      // Store P
      MOVQ_RM (0,((MM_U_32 *)P[j]->p[d])[W]);
      // Add P to Q
      PADDD_RR (0,5);
    }
    // Store Q
    MOVQ_RM (5,((MM_U_32 *)Q->p[d])[W]);
  } // for d

  //--- Main Y loop ---
  for(y=-1;y<Y-W-1;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    y_W = y+W;
    yMODW = y%W;


    // compute M for x=0    
    if (y>=0) {
      for (k=0;k<=D;k+=2){
  //fprintf(stderr,"depthmap: k:%d \n",k);
  // M[k]=0;
  // window across image
  //for(i=k;i<k+W;i++) {
  //  S[i] -= R[i][y%W];
  //  R[i][y%W] = ((MM_U_8 *)I2->p[y+W])[i] * ((MM_U_8 *)I2->p[y+W])[i];
  //  S[i] += R[i][y%W];  
  //  M[k] += S[i];
  //}
  // reg 6&6 = M
  PXOR_RR (6,6);
  for(i=k;i<k+W;i+=2) {
    // reg 5 & 4 = S
    MOVQ_MR (((MM_U_32 *)S)[i],5);  
    // Sub prev R from S
    PSUBD_MR (((MM_U_32 *)R->p[yMODW])[i],5);
    // calc new R
    I1_I2MULTISUM(I2, I2, i,y_W,i,y_W);
    // Store R
    MOVQ_RM (0,((MM_U_32 *)R->p[yMODW])[i]);
    // Add R to S
    PADDD_RR (0,5);
    // Store S
    MOVQ_RM (5,((MM_U_32 *)S)[i]);
    // Add curr S to M
    PADDD_RR (5,6);
  } // for i
  // Store M[0]
  MOVQ_RM (6,((MM_U_32 *)M)[k]);
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
    for(x=-2;x<X-W-D-2;x+=2) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      x_W= x+W;
   
      // compute M for y=0 
      if (y<0) {
  // Equivalent C function
  //S[x+D+W] = 0;
  //for (j=0;j<W;j++) {
  //  R[x+D+W][j]= ((MM_U_8 *)I2->p[j])[x+D+W] * ((MM_U_8 *)I2->p[j])[x+D+W];
  //  S[x+D+W] += R[x+D+W][j];
  //}
  // reg 5 = S
  PXOR_RR (5,5);
  // window down image
  for (j=0;j<W;j++) {
    //fprintf(stderr,"depthmap: j:%d\n",j);
    // Calc P
    I1_I2MULTISUM(I2, I2,x_W+D,j,x_W+D,j);
    // Store R
    MOVQ_RM (0,((MM_U_32 *)R->p[j])[x_W+D]);
    // Add curr R to S
    PADDD_RR (0,5);
  }
  // Store S
  MOVQ_RM (5,((MM_U_32 *)S)[x_W+D]);

      }  
      else
      {
  // Equivalent C function
  // P is [X][W] the W dimension is a wrap around buffer.    
  //S[x+D+W] -= R[x+D+W][y%W];
  //R[x+D+W][y%W] = ((MM_U_8 *)I2->p[y+W])[x+D+W] * ((MM_U_8 *)I2->p[y+W])[x+D+W];
  //S[x+D+W] += R[x+D+W][y%W];
  
  // reg 5 = S
  MOVQ_MR (((MM_U_32 *)S)[x_W+D],5);
  // subtract prev R
  PSUBD_MR (((MM_U_32 *)R->p[yMODW])[x_W+D],5);
  // Add curr R to S
  I1_I2MULTISUM(I2, I2,x_W+D,y_W,x_W+D,y_W);
  PADDD_RR (0,5);
  // Store S
  MOVQ_RM (5,((MM_U_32 *)S)[x_W+D]);
  // Store R
  MOVQ_RM (0,((MM_U_32 *)R->p[yMODW])[x_W+D]);
      }
  
      // Equivalent C function
      //M[x+D+1] = M[x+D] - S[x+D] + S[x+D+W];
      // reg 6 & 6 = M
      MOVQ_MR (((MM_U_32 *)M)[x+D],6);  
      // Sub S[x+D] from M
      PSUBD_MR (((MM_U_32 *)S)[x+D],6);
      // Add S[x+D+W] to M
      MOVQ_MR (((MM_U_32 *)S)[x_W+D],1);
      PADDD_RR (1,6);
      // Store M[x+1][d]
      MOVQ_RM (6,((MM_U_32 *)M)[x+D+2]);
#ifdef FAST_SQRT
      sqrt2 = fast_sqrt(((MM_U_32 *)M)[x+D+2]);
      sqrt3 = fast_sqrt(((MM_U_32 *)M)[x+D+3]);
      ((MM_U_32 *)MINV)[x+D+2] = ((((unsigned long long)0xffffffff) + sqrt2 )/ sqrt2);
      ((MM_U_32 *)MINV)[x+D+3] = ((((unsigned long long)0xffffffff) + sqrt3 )/ sqrt3);
#else
      ((MM_U_32 *)MINV)[x+D+2] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+D+2])/((MM_U_32 *)M)[x+D+2]);
      ((MM_U_32 *)MINV)[x+D+3] = ((((unsigned long long)0xffffffff) + ((MM_U_32 *)M)[x+D+3])/((MM_U_32 *)M)[x+D+3]);
#endif
      //printf("%d %d %ld %ld\n",y,x,((MM_U_32 *)M)[x+2],((MM_U_32 *)M)[x+3]);
      
      max_d.mmu32[0] = 0;
      max_d.mmu32[1] = 0;    

      max.mmu32[0] = 0;
      max.mmu32[1] = 0;    

      //--- Main D loop ---
      for (d=0;d<D;d++) {
  //fprintf(stderr,"depthmap: d:%d \n",d);
  if (x<0) {
    if (y>=0) {
      // Equivalent C function
      //N[0][d]=0;
      //for(i=0;i<W;i++) {
      //  Q[i][d] -= P[i][y%W][d];
      //  P[i][y%W][d] = ((MM_U_8 *)I1->p[y+W])[i+d_2] * ((MM_U_8 *)I2->p[y+W])[i+d];
      //  Q[i][d] += P[i][y%W][d];
      //  N[0][d] += Q[i][d];
      //}
      //Q[W][d] -= P[W][y%W][d];
      //P[W][y%W][d] = ((MM_U_8 *)I1->p[y+W])[W+d_2] * ((MM_U_8 *)I2->p[y+W])[W+d];
      //Q[W][d] += P[W][y%W][d];
    
      // reg 6&6 = N
      PXOR_RR (6,6);
      for(i=0;i<W;i+=2) {
        // reg 5 & 4 = Q
        MOVQ_MR (((MM_U_32 *)Q->p[d])[i],5);  
        // Sub prev P from Q
        PSUBD_MR (((MM_U_32 *)P[yMODW]->p[d])[i],5);
        // Add curr P to Q
        I1_I2MULTISUM(I1, I2, i+d_2,y_W,i+d,y_W);
        // Add new P
        PADDD_RR (0,5);
        // Store Q
        MOVQ_RM (5,((MM_U_32 *)Q->p[d])[i]);
        // Store P
        MOVQ_RM (0,((MM_U_32 *)P[yMODW]->p[d])[i]);
        // Add curr Q to N
        PADDD_RR (5,6);
      } // for i
      // Store N[d][0]
      MOVQ_RM (6,((MM_U_32 *)N->p[d])[0]);

      // Do for i=W
      // reg 5 & 4 = Q
      MOVQ_MR (((MM_U_32 *)Q->p[d])[i],5);  
      // Sub prev P from Q
      PSUBD_MR (((MM_U_32 *)P[yMODW]->p[d])[i],5);
      // Add curr P to Q
      // lbw in 0
      I1_I2MULTISUM(I1, I2, i+d_2,y_W,i+d,y_W);
      // Add new P
      PADDD_RR (0,5);
      // Store Q
      MOVQ_RM (5,((MM_U_32 *)Q->p[d])[i]);
      // Store P
      MOVQ_RM (0,((MM_U_32 *)P[yMODW]->p[d])[i]);
    } // y>=0
  } // x <0
  else
  {
    if (y<0) {
      // Equivalent C function
      //Q[x+W][d] = 0;
      //for (j=0;j<W;j++) {
      //  P[x+W][j][d]= ((MM_U_8 *)I1->p[j])[x+W+d_2] * ((MM_U_8 *)I2->p[j])[x+W+d];
      //  Q[x+W][d] += P[x+W][j][d];
      //}

      // reg 5&4 = Q
      PXOR_RR (5,5);
      
      // window down image
      for (j=0;j<W;j++) {
        I1_I2MULTISUM(I1, I2,x_W+d_2,j,x_W+d,j);
        MOVQ_RM (0,((MM_U_32 *)P[j]->p[d])[x_W]);
        // Add curr P to Q
        PADDD_RR (0,5);
      }
      // Store Q
      MOVQ_RM (5,((MM_U_32 *)Q->p[d])[x_W]);
      
    } // y<0 
    else
    {
      // Equivalent C function
      // P is [X][W][D] the W dimension is a wrap around buffer.
      //Q[x+W][d] -= P[x+W][y%W][d];
      //P[x+W][y%W][d] = ((MM_U_8 *)I1->p[y+W])[x+W+d_2] * ((MM_U_8 *)I2->p[y+W])[x+W+d];
      //Q[x+W][d] += P[x+W][y%W][d];

      // reg 5&4 = Q
      MOVQ_MR (((MM_U_32 *)Q->p[d])[x_W],5);
      // subtract prev P
      PSUBD_MR (((MM_U_32 *)P[yMODW]->p[d])[x+W],5);
      // Add curr P to Q
      I1_I2MULTISUM(I1, I2,x_W+d_2,y_W,x_W+d,y_W);
      PADDD_RR (0,5);
        
      // Store Q
      MOVQ_RM (5,((MM_U_32 *)Q->p[d])[x_W]);
      
      // Store P
      MOVQ_RM (0,((MM_U_32 *)P[yMODW]->p[d])[x_W]);

    } // y>=0

    // Equivalent C function
    //N[x+1][d] = N[x][d] - Q[x][d] + Q[x+W][d];
    
    // reg 6 & 6 = N
    MOVQ_MR (((MM_U_32 *)N->p[d])[x],6);  

    // Sub curr Q[x][d] from N
    PSUBD_MR (((MM_U_32 *)Q->p[d])[x],6);
      
    // Add Q[x+W][d] to N
    MOVQ_MR (((MM_U_32 *)Q->p[d])[x_W],1);
    PADDD_RR (1,6);
      
    // Store N[x+1][d]
    MOVQ_RM (6,((MM_U_32 *)N->p[d])[x+2]);
    
  } // x>=0
  // Equivalent C function              
  //C1 = ((unsigned long long)((MM_U_32 *)N->p[d])[x+2]*((MM_U_32 *)N->p[d])[x+2]/((MM_U_32 *)M)[x+d+2]);
  //if (C1 > max1) {
  //   max1 = C1;
  //   max1_d = d;
  //}
  //C2 = ((unsigned long long)((MM_U_32 *)N->p[d])[x+3]*((MM_U_32 *)N->p[d])[x+3]/((MM_U_32 *)M)[x+d+3]);
  //if (C2 > max2) {
  //  max2 = C2;
  //  max2_d = d;
  //}

#ifdef FAST_SQRT
  curr.mmu32[0] = ((unsigned long long)((MM_U_32 *)N->p[d])[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
  curr.mmu32[1] = ((unsigned long long)((MM_U_32 *)N->p[d])[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#else
  curr.mmu32[0] = ((unsigned long long)((MM_U_32 *)N->p[d])[x+2]*((MM_U_32 *)N->p[d])[x+2]*((MM_U_32 *)MINV)[x+d+2])>>32;
  curr.mmu32[1] = ((unsigned long long)((MM_U_32 *)N->p[d])[x+3]*((MM_U_32 *)N->p[d])[x+3]*((MM_U_32 *)MINV)[x+d+3])>>32;
#endif

  
  //curr.mmu32[0] = ((MM_U_32 *)N->p[d])[x+2]/fast_sqrt(((MM_U_32 *)M)[x+d+2]);
  //curr.mmu32[1] = ((MM_U_32 *)N->p[d])[x+3]/fast_sqrt(((MM_U_32 *)M)[x+d+3]);
  

  //curr.mmu32[0] = ((unsigned long long)((MM_U_32 *)N->p[d])[x+2]*((MM_U_32 *)N->p[d])[x+2]/((MM_U_32 *)M)[x+d+2]);
  //curr.mmu32[1] = ((unsigned long long)((MM_U_32 *)N->p[d])[x+3]*((MM_U_32 *)N->p[d])[x+3]/((MM_U_32 *)M)[x+d+3]);
  
  // reg 5 = curr 
  MOVQ_MR(curr,5);
        MOVQ_RR(5,0);
  //PRINTREG32(5,"curr");

  // reg 6 = max
        MOVQ_MR(max,6);
  MOVQ_RR(6,2);
  //PRINTREG32(6,"max");
        
  // reg 2 becomes mask
        PCMPGTD_RR(0,2);
        MOVQ_RR(2,1);
  //PRINTREG32(2,"mask");

        // reg 0 is preserved values larger than max
  MOVQ_RR(6,0);
  PAND_RR(2,0);
  
  // reg 1 is curr
  PANDN_RR(5,1);
  
  // reg 1 is new max
  POR_RR(0,1);
  MOVQ_RM(1,max);
  //PRINTREG32(1,"new_max");

  // reg 1 is preserved d values
  MOVQ_MR(max_d,3);
        MOVQ_RR(2,1)
  PAND_RR(3,1);

  // reg 0 = current d value
  curr_d.mmu32[0] = d;
  curr_d.mmu32[1] = d;    

  MOVQ_MR(curr_d,4);
  PANDN_RR(4,2);
  
  // reg 6 = new d's for minimum values
  POR_RR(2,1);
  MOVQ_RM(1,max_d);
  //PRINTREG32(1,"new_max_d");


      }// for d            
      ((MM_U_8 *)Out->p[y+1])[x+2] = max_d.mmu8[0];
      ((MM_U_8 *)Out->p[y+1])[x+3] = max_d.mmu8[4];
      
      //EMMS;
      // clear reg 7
      //PXOR_RR (7,7);

    }// for x
  }// for y

  EMMS;
}
















