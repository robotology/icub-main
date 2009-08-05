 
#include "sptmpmmx.h"

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


// I1_I2D:
// abs(I1(x1,y1)-I2(x2,y2))
// abs(I1(x1,y1)-I2(x2+1,y2))
// abs(I1(x1,y1)-I2(x2+2,y2))
// abs(I1(x1,y1)-I2(x2+3,y2))
#define I1_I2D(I1,I2,x1,y1,x2,y2) \
{\
  movq_m2r(((MM_U_8 *)I1->p[y1].m)[x1],mm0);\
  punpcklbw_r2r(mm7,mm0);\
  punpcklwd_r2r(mm0,mm0);\
  punpckldq_r2r(mm0,mm0);\
  movq_r2r(mm0,mm4);\
  movq_m2r(((MM_U_8 *)I2->p[y2].m)[x2],mm5);\
  punpcklbw_r2r(mm7,mm5);\
  movq_r2r(mm5,mm6);\
  psubusw_r2r(mm4,mm5);\
  psubusw_r2r(mm6,mm0);\
  por_r2r(mm5,mm0);\
}


//-----------------------------------------------------------------------
//----------------------------------------------------------------
int Depthmap_FLOW(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *O)
{
  const int win = 16;
  const int d_max = 16;
  const int e_max = 16;


  int y_max,x_max,i, j;
  int sum, best=0, best_d=0;

  register int d,e,x,y;

  
  int w_2 = win/2;
  int d_2 = d_max/2;
  int e_2 = e_max/2;

  x_max = I1->cols;
  y_max = I1->rows;

  
  for(y=0;y<y_max;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);

    for(x=0;x<x_max;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
	
      if ((y<(e_2+w_2))||(x<(d_2+w_2))||(y>(y_max-1-e_2-w_2))||(x>(x_max-1-d_2-w_2))) {
	((MM_U_8 *)O->p[y].m)[x]=0;
      }
      else
      {
	for (d=0;d<d_max;d++) {
	  //fprintf(stderr,"depthmap: d:%d \n",d);
	  for (e=0;e<e_max;e++) {
	    //fprintf(stderr,"depthmap: d:%d \n",d);
	    sum = 0;
	    for(j=0;j<win;j++) {
	      //fprintf(stderr,"depthmap: j:%d \n",j);

	      for(i=0;i<win;i++) {
		//fprintf(stderr,"depthmap: i:%d \n",i);
		
		sum+= abs(((MM_U_8 *)I1->p[y-w_2+j].m)[x-w_2+i]-((MM_U_8 *)I2->p[y+e-e_2-w_2+j].m)[x+d-d_2-w_2+i]);
	      } // i	  
	    }// j
	    if (((d==0)&&(e==0))||(sum < best)) {
	      best = sum;
	      best_d = ((e>>1)<<4)+(d>>1);
	    }
	  }//e
	} //d
	((MM_U_8 *)O->p[y].m)[x]= (MM_U_8) best_d;
	
      } //if

      /*
      if (y>8) {
	e_rl = (((MM_U_8 *)Out->p[y-8].m)[x])/8;
	d_rl = (((MM_U_8 *)Out->p[y-8].m)[x])&0x0f;
	//printf("e:%d d:%d\n",e_rl,d_rl);
	min_rl = N[x][(y-8)%(win+8)][e_rl][d_rl];

	for (e=0;e<e_max;e++) {	   
	  y_rl = y-8+e_rl-e;
	  for (d=0;d<d_max;d++) {	   
	    x_rl = x+d_rl-d;
	    if ((x_rl>-1)&&(x_rl<x_max)&&
		(y_rl>-1)&&(y_rl<y_max)&&
		(min_rl > N[x_rl][y_rl%(win+8)][e][d])) {
	      ((MM_U_8 *)Out->p[y-8].m)[x] = 0x00;
	      break;
	    }
	  }
	}
      }
      */


    } //x
  } //y

  return 0;
}

//----------------------------------------------------------------
int Recursive_Depthmap_FLOW(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out)
{

  const int win = 16;
  const int d_max = 8;
  //  const int x_max = 128;
  //const int y_max = 128;
  const int x_max = 64;
  const int y_max = 64;
  const int e_max = 8;


  register int i, j;
  int min=0, min_d=0, min_e=0;

  register int d,e,x,y;

  
  int d_2 = d_max/2;
  int e_2 = e_max/2;

  //  x_max = I1->cols;
  //y_max = I1->rows;

  short P[x_max][win+4][e_max][d_max];
  short Q1[x_max][win+4][e_max][d_max];
  short Q2[x_max][win+4][e_max][d_max];
  short N[x_max][win+4][e_max][d_max];

  int y_1_win_m, y_win_m, y_m, y_1_m;

  for(y=0;y<y_max-e_max-win;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    y_1_win_m = (y-1+win)%(win+8);
    y_1_m = (y-1)%(win+8);
    y_m = (y)%(win+8);
    y_win_m = (y+win)%(win+8);

    for(x=0;x<x_max-d_max-win;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      for (e=0;e<e_max;e++) {
	//fprintf(stderr,"depthmap: e:%d \n",e);
	for (d=0;d<d_max;d++) {
	  //fprintf(stderr,"depthmap: d:%d \n",d);
	  if ((x==0)&&(y==0)) {
	    N[0][0][e][d]=0;
	    // window across image
	    for(i=0;i<=win;i++) 
	      Q1[i][0][e][d] = 0;
	    for(j=0;j<=win;j++) 
	      Q2[0][j][e][d] = 0;
	    
	    // find Q1(mm0,mm0) Q2(0,0)
	    for(i=0;i<=win;i++) {
	      // window down image
	      for (j=0;j<=win;j++) {	       
		P[i][j][e][d]= abs(((MM_U_8 *)I1->p[j+e_2].m)[i+d_2] - ((MM_U_8 *)I2->p[j+e].m)[i+d]);
		if (j<win)
		  Q1[i][0][e][d] += P[i][j][e][d];
		if (i<win)
		  Q2[0][j][e][d] += P[i][j][e][d];
	     
	      }
	      if (i<win)
		N[0][0][e][d] += Q1[i][0][e][d];
	    }
	    //N[0][0][e][d] -= Q1[win][0][e][d];

	  }
	  else if (x==0) {
	    N[0][y_m][e][d]=0;
	    Q2[0][y_m][e][d] = 0;
	    for(i=0;i<win;i++) {
	      P[i][y_1_win_m][e][d] = abs(((MM_U_8 *)I1->p[y-1+win+e_2].m)[i+d_2] - ((MM_U_8 *)I2->p[y+win-1+e].m)[i+d]);
	      Q1[i][y_m][e][d] = Q1[i][y_1_m][e][d] + P[i][y_1_win_m][e][d] - P[i][y_1_m][e][d];
	      Q2[0][y_1_win_m][e][d] += P[i][y_1_win_m][e][d];
	      N[0][y_m][e][d] += Q1[i][y_m][e][d];	
	    }
	  }
	  else if (y==0) {
	    N[x][0][e][d]=0;
	    Q1[x+win-1][0][e][d] = 0;
	    for (j=0;j<win;j++) {
	      P[x+win-1][j][e][d] = abs(((MM_U_8 *)I1->p[j+e_2].m)[x+win-1+d_2] - ((MM_U_8 *)I2->p[j+e].m)[x+win-1+d]);
	      Q2[x][j][e][d] = Q2[x-1][j][e][d] + P[x+win-1][j][e][d] - P[x-1][j][e][d];
	      Q1[x+win-1][0][e][d] += P[x+win-1][j][e][d];
	      N[x][0][e][d] += Q2[x][j][e][d];
	    }
	  }  
	  else {
	    // P is [N][win][e_max][d_max] the win dimension is a wrap around buffer.
	    
	    P[x-1+win][y_1_win_m][e][d] = abs(((MM_U_8 *)I1->p[y+win-1+e_2].m)[x-1+win+d_2] - ((MM_U_8 *)I2->p[y+win-1+e].m)[x-1+win+d]);	    
	    
	    Q1[x-1+win][y_m][e][d] = Q1[x-1+win][y_1_m][e][d] + P[x-1+win][y_1_win_m][e][d] - P[x-1+win][y_1_m][e][d];
	    Q2[x][y_1_win_m][e][d] = Q2[x-1][y_1_win_m][e][d] + P[x-1+win][y_1_win_m][e][d] - P[x-1][y_1_win_m][e][d];


	    N[x][y_m][e][d] = N[x-1][y_1_m][e][d] 
	      + Q1[x-1+win][y_1_m][e][d] - Q1[x-1][y_1_m][e][d]
	      + Q2[x-1][y_1_win_m][e][d] - Q2[x-1][y_1_m][e][d] 
	      + P[x-1][y_1_m][e][d] + P[x-1+win][y_1_win_m][e][d]
	      - P[x-1+win][y_1_m][e][d] - P[x-1][y_1_win_m][e][d];        
	  }

          if (((d==0)&&(e==0))||(N[x][y_m][e][d] < min)) {
	    min = N[x][y_m][e][d];
	    min_d = d;
	    min_e = e;
	  }
	}// for d
      }// for e
      ((MM_U_8 *)Out->p[y].m)[x]= min_e*8+min_d; 
      //((MM_U_8 *)Out->p[y].m)[x]= min_e; 

      /*
      // DOESNT WORK BECAUSE y+e_rl-e not be calculated yet.
      if (y>8) {
	e_rl = (((MM_U_8 *)Out->p[y-4].m)[x])/8;
	d_rl = (((MM_U_8 *)Out->p[y-4].m)[x])&0x0f;
	//printf("e:%d d:%d\n",e_rl,d_rl);
	min_rl = N[x][(y-4)%(win+8)][e_rl][d_rl];

	for (e=0;e<e_max;e++) {	   
	  y_rl = y-4+e_rl-e;
	  for (d=0;d<d_max;d++) {	   
	    x_rl = x+d_rl-d;
	    if ((x_rl>-1)&&(x_rl<x_max)&&
		(y_rl>-1)&&(y_rl<y_max)&&
		(min_rl > N[x_rl][y_rl%(win+8)][e][d])) {
	      ((MM_U_8 *)Out->p[y-4].m)[x] = 0x00;
	      break;
	    }
	  }
	}
      }
      */

    }// for x
  }// for y

  return 0;
}

// find pointer offset
//#define MEM(base,j,i) ((base)[((j)%win_BUF)*x_max+(i)])
// win_BUF = 32
//#define MEM(base,j,i) ((base)[(((j)&0x001f)*x_max)+(i)])
// win_BUF = 16
#define MEM(base,j,i) ((base)[(((j)&0x000f)*x_max)+(i)])
//#define MEM(base,j,i) ((base)[(((j)&0x001f)<<8)+(i)])
//#define MEM(base,j,i) ((base)[(((j)&0x000f)*x_max)+(i)])


// assumes new P is in reg 0
// Q1(x,y+1) = Q1(x,y) - P(x,y) + P(x,y+win)
#define Q1inc(Q1,x,y,e,d,P) \
{\
		  /* reg 1 = Q1*/\
		  movq_m2r (((MM_U_16 *)MEM(Q1,y,x)->p[e].m)[d],mm1);\
		  /* subtract prev P*/\
		  psubw_m2r (((MM_U_16 *)MEM(P,y,x)->p[e].m)[d],mm1);\
		  /* Add curr P to Q*/\
		  paddw_r2r (mm0,mm1);\
		  /* Store Q*/\
		  movq_r2m (mm1,((MM_U_16 *)MEM(Q1,y+1,x)->p[e].m)[d]);\
}

// assumes new P is in reg 0
// Q2(x+1,y) = Q2(x,y) - P(x,y) + P(x+win,y)
#define Q2inc(Q2,x,y,e,d,P) \
{\
     /* reg 2 = Q2*/ \
     movq_m2r (((MM_U_16 *)MEM(Q2,y,x)->p[e].m)[d],mm2); \
     /* subtract prev P */\
     psubw_m2r (((MM_U_16 *)MEM(P,y,x)->p[e].m)[d],mm2);\
     /* Add curr P to Q*/\
     paddw_r2r (mm0,mm2);\
     /* Store Q*/ \
     movq_r2m (mm2,((MM_U_16 *)MEM(Q2,y,x+1)->p[e].m)[d]);\
}


// N[x][y_m][e][d] = N[x-1][y-1][e][d] 
//  + Q1[x-1+win][y-1][e][d] - Q1[x-1][y-1][e][d]
//  + Q2[x-1][y-1+win_m][e][d] - Q2[x-1][y-1][e][d] 
//  + P[x-1][y-1][e][d] + P[x-1+win][y-1+win][e][d]
//  - P[x-1+win][y-1][e][d] - P[x-1][y-1+win][e][d];       
// assumes new P is in reg 0
#define Ninc(N,x,y,e,d,Q1,Q2,P) \
{\
     /* reg 2 = N */\
     movq_m2r (((MM_U_16 *)MEM(N,y,x)->p[e].m)[d],mm3);\
     /* add Q1 */\
     paddw_m2r (((MM_U_16 *)MEM(Q1,y,x+win)->p[e].m)[d],mm3);\
     /* subtract prev Q1 */\
     psubw_m2r (((MM_U_16 *)MEM(Q1,y,x)->p[e].m)[d],mm3);\
     /* add Q2 */\
     paddw_m2r (((MM_U_16 *)MEM(Q2,y+win,x)->p[e].m)[d],mm3);\
     /* subtract prev Q2 */\
     psubw_m2r (((MM_U_16 *)MEM(Q2,y,x)->p[e].m)[d],mm3);\
     /* subtract prev P */\
     psubw_m2r (((MM_U_16 *)MEM(P,y,x+win)->p[e].m)[d],mm3);\
     /* subtract prev P */\
     psubw_m2r (((MM_U_16 *)MEM(P,y+win,x)->p[e].m)[d],mm3);\
     /* add prev P */\
     paddw_m2r (((MM_U_16 *)MEM(P,y,x)->p[e].m)[d],mm3);\
     /* add curr P */\
     paddw_r2r (mm0,mm3);\
     /* Store N */\
     movq_r2m (mm3,((MM_U_16 *)MEM(N,y+1,x+1)->p[e].m)[d]);\
}


//----------------------------------------------------------------
int MMXRecursive_Depthmap_FLOW(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out)
{ 
  const int win = 15;
  const int win_BUF = 16;
  const int d_max = 8;
  const int e_max = 8;
  const short d_2 = 4; // d_max/2
  const short e_2 = 4; // e_max/2


  int x_max;
  int y_max;

  int i,j,d,e,x,y, x_1, y_1, x_1pwin, y_1pwin;
  int e_lr, d_lr, y_rl, x_rl, e_rl,d_rl,min_rl;


  unsigned short overall_min_lr;
  unsigned char overall_min_d_lr;
  unsigned char overall_min_e_lr;

  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t curr_e_lr;
  mmx_t min_d_lr;
  mmx_t min_e_lr;

  static int firsttime =1;
  static MMXMatrix **P;
  static MMXMatrix **Q1;
  static MMXMatrix **Q2;
  static MMXMatrix **N;



  x_max = I1->cols;
  y_max = I1->rows;

  if (firsttime) {
    P = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*x_max*win_BUF); 
    Q1 = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*x_max*win_BUF); 
    Q2 = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*x_max*win_BUF); 
    N = (MMXMatrix **) Alloc(sizeof(MMXMatrix *)*x_max*win_BUF); 

    for (y=0;y<win_BUF;y++) 
      for (x=0;x<x_max;x++) 
        MEM(P,y,x) = MMXMatrixAlloc(MMT_U_16, d_max, e_max);

    for (y=0;y<win_BUF;y++) 
      for (x=0;x<x_max;x++) 
        MEM(Q1,y,x) = MMXMatrixAlloc(MMT_U_16, d_max, e_max);

    for (y=0;y<win_BUF;y++) 
      for (x=0;x<x_max;x++) 
        MEM(Q2,y,x) = MMXMatrixAlloc(MMT_U_16, d_max, e_max);

    for (y=0;y<win_BUF;y++)
      for (x=0;x<x_max;x++) 
        MEM(N,y,x) = MMXMatrixAlloc(MMT_U_16, d_max, e_max);  
    firsttime =0;
  }

  emms();
  pxor_r2r(mm7,mm7);

  for(y=0;y<y_max-e_max-win;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    y_1 = y-1;
    y_1pwin = y-1+win;
    for(x=0;x<x_max-d_max-win;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      x_1 = x-1;
      x_1pwin = x-1+win;

      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      // set e
      movq_r2m(mm7,curr_e_lr);
 
      for (e=0;e<e_max;e++) {
	//fprintf(stderr,"depthmap: e:%d \n",e);

	// set d
	curr_d_lr.uw[0] = 0x000;
	curr_d_lr.uw[1] = 0x001;
	curr_d_lr.uw[2] = 0x002;
	curr_d_lr.uw[3] = 0x003;
	
	for (d=0;d<d_max-3;d+=4) {
	  //fprintf(stderr,"depthmap: d:%d \n",d);

	  if ((!x)&&(!y)) {
	    movq_r2m(mm7,((MM_U_16 *)MEM(N,0,0)->p[e].m)[d]);
	    // window across image
	    for(i=0;i<=win;i++) {
	      movq_r2m(mm7,((MM_U_16 *)MEM(Q1,0,i)->p[e].m)[d]);
	      movq_r2m(mm7,((MM_U_16 *)MEM(Q2,i,0)->p[e].m)[d]);
	    }
	    
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    // find Q1 & Q2
	    for(i=0;i<win;i++) {
	      // window down image
	      for (j=0;j<win;j++) {	       
		//P[i][j][e][d]= abs(((MM_U_8 *)I1->p[j+e_2].m)[i+d_2] - ((MM_U_8 *)I2->p[j+e].m)[i+d]);
		//Q1[i][0][e][d] += P[i][j][e][d];
		//Q2[0][j][e][d] += P[i][j][e][d];
		// compute |I1-I2| store result in reg 0
		I1_I2D(I1, I2,i+d_2,j+e_2,i+d,j+e);
		movq_r2m(mm0,((MM_U_16 *)MEM(P,j,i)->p[e].m)[d]);
		
		if (j<win) {
		  movq_m2r(((MM_U_16 *)MEM(Q1,0,i)->p[e].m)[d],mm1);
		  paddw_r2r(mm0,mm1);
		  movq_r2m(mm1,((MM_U_16 *)MEM(Q1,0,i)->p[e].m)[d]);
		}
		if (i<win) { 
		  movq_m2r(((MM_U_16 *)MEM(Q2,j,0)->p[e].m)[d],mm2);
		  paddw_r2r(mm0,mm2);
		  movq_r2m(mm2,((MM_U_16 *)MEM(Q2,j,0)->p[e].m)[d]);
		}
		if ((j<win)&&(i<win)) {
		  paddw_r2r(mm0,mm3);
		}
	      }
	    }
	    movq_r2m(mm3,((MM_U_16 *)MEM(N,0,0)->p[e].m)[d]);	    
	  }
	  else if (!x) {
	    // Q2 = reg 2
	    pxor_r2r(mm2,mm2);
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    for(i=0;i<win;i++) {
	      // compute |I1-I2| store result in reg 0
	      I1_I2D(I1, I2,i+d_2,y_1pwin+e_2,i+d,y_1pwin+e);
	      movq_r2m(mm0,((MM_U_16 *)MEM(P,y_1pwin,i)->p[e].m)[d]);
	      
	      Q1inc(Q1,i,y_1,e,d,P);
	      
	      paddw_r2r(mm0,mm2);
	      paddw_r2r(mm1,mm3);
	    }
	    movq_r2m(mm2,((MM_U_16 *)MEM(Q2,y_1pwin,0)->p[e].m)[d]);
	    movq_r2m(mm3,((MM_U_16 *)MEM(N,y,0)->p[e].m)[d]);	    
	  }
	  else if (!y) {
	    // Q1 = reg 1
	    pxor_r2r(mm1,mm1);
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    for (j=0;j<win;j++) {
	      // compute |I1-I2| store result in reg 0
	      I1_I2D(I1, I2,x_1pwin+d_2,j+e_2,x_1pwin+d,j+e);
	      movq_r2m(mm0,((MM_U_16 *)MEM(P,j,x_1pwin)->p[e].m)[d]);

	      Q2inc(Q2,x_1,j,e,d,P);

	      paddw_r2r(mm0,mm1);
	      paddw_r2r(mm2,mm3);
	    }
	    movq_r2m(mm1,((MM_U_16 *)MEM(Q1,0,x_1pwin)->p[e].m)[d]);
	    movq_r2m(mm3,((MM_U_16 *)MEM(N,0,x)->p[e].m)[d]);
	  }  
	  else {
	    //===  x & y > 0
	    I1_I2D(I1, I2,x_1pwin+d_2,y_1pwin+e_2,x_1pwin+d,y_1pwin+e);
	    movq_r2m(mm0,((MM_U_16 *)MEM(P,y_1pwin,x_1pwin)->p[e].m)[d]);
	    
	    Q1inc(Q1,x_1pwin,y_1,e,d,P);
	    
	    Q2inc(Q2,x_1,y_1pwin,e,d,P);

	    Ninc(N,x_1,y_1,e,d,Q1,Q2,P);
	  }
	  
	  // ---------------------------------------------
	  // find minimum N Left->Right for C[x][y][e][d]    	
	  // ---------------------------------------------
	  // first create & store 0x0001000100010001 in 6
	  // for future reference
	  pxor_r2r(mm6,mm6);
	  pcmpeqw_r2r(mm1,mm1);
	  psubw_r2r(mm1,mm6);
	  //PRINTREG16(mm6,"0x0001");


	  // reg 3 = N 
	  //movq_m2r (((MM_U_16 *)MEM(N,y,x)->p[e].m)[d],mm3);
	  movq_r2r(mm3,mm0);

	  //reg 4 = minimum N
	  movq_m2r(min_lr,mm4);
	  movq_r2r(mm4,mm5);

	  //fill 1 with 0x80008000s (sign bit)
	  movq_r2r(mm6,mm1);
	  psllw_i2r(15,mm1);
	  //PRINTREG16(mm1,"0x800");

	  paddw_r2r(mm1,mm5);
	  paddw_r2r(mm1,mm0);
	  //PRINTREG16(mm5,"min_lr");
	  //PRINTREG16(mm0,"N");
        
	  // reg 5 becomes mask
	  pcmpgtw_r2r(mm0,mm5);
	  movq_r2r(mm5,mm1);
	
	  //PRINTREG16(mm2,"mask");
	
	  // reg 0 is N values smaller than minimum
	  movq_r2r(mm3,mm0);
	  pand_r2r(mm5,mm0);
		
	  // reg 4 is preserved minimum
	  pandn_r2r(mm4,mm1);
	
	  // reg 1 is new minimum
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_lr);
	  //PRINTREG16(mm1,"min_lr");

	  // -----------------------------------------------
	  // reg 4 is d values for minimum
	  movq_m2r(min_d_lr,mm4);
	  // reg 1 is preserved d values
	  movq_r2r(mm5,mm1);
	  pandn_r2r(mm4,mm1);

	  // reg 0 = current d value
	  movq_m2r(curr_d_lr,mm3);
	  movq_r2r(mm3,mm0);


	  // reg 0 = current d value for updated minimum
	  pand_r2r(mm5,mm0);
	
	  // reg 1 = new d's for minimum values
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_d_lr);
	  //PRINTREG16(mm1,"new_min_d_lr");

	  // increase current d's
	  // fill with 0x0004000400040004 
	  movq_r2r(mm6,mm0);
	  psllw_i2r(2,mm0);
	  //PRINTREG16(mm0,"0x0004");
	  paddw_r2r(mm0,mm3);
	  
	  movq_r2m(mm3,curr_d_lr);
	  //PRINTREG16(mm3,"curr_d_lr");

	  //-------------------------------------------------
	  // reg 4 is e values for minimum
	  movq_m2r(min_e_lr,mm4);
	  // reg 1 is preserved e values
	  movq_r2r(mm5,mm1);
	  pandn_r2r(mm4,mm1);

	  // reg 0 = current e value
	  movq_m2r(curr_e_lr,mm3);
	  movq_r2r(mm3,mm0);

	  // reg 0 = current e value for updated minimum
	  pand_r2r(mm5,mm0);
	
	  // reg 1 = new e's for minimum values
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_e_lr);
	  //PRINTREG16(mm1,"new_min_e_lr");
	  
	}// for d

	// increase current e's by 0x00010001...
	paddw_r2r(mm6,mm3);
	movq_r2m(mm3,curr_e_lr);
	//PRINTREG16(mm3,"curr_e_lr");
	
      }// for e
                  
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]    	
      // ---------------------------------------------
      overall_min_lr   = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      overall_min_e_lr = min_e_lr.uw[0];
      if (overall_min_lr > min_lr.uw[1]) {
	overall_min_lr   = min_lr.uw[1];
	overall_min_d_lr = min_d_lr.uw[1];
	overall_min_e_lr = min_e_lr.uw[1];
      }
      if (overall_min_lr > min_lr.uw[2]) {
	overall_min_lr   = min_lr.uw[2];
	overall_min_d_lr = min_d_lr.uw[2];
	overall_min_e_lr = min_e_lr.uw[2];
      }
      if (overall_min_lr > min_lr.uw[3]) {
	overall_min_lr   = min_lr.uw[3];
	overall_min_d_lr = min_d_lr.uw[3];
	overall_min_e_lr = min_e_lr.uw[3];
      }
      //((MM_U_8 *)Out->p[y].m)[x] = (overall_min_d_lr);
      //((MM_U_8 *)Out->p[y].m)[x] = (overall_min_e_lr);
      //#define rely
#ifdef rely
      ((MM_U_8 *)Out->p[y].m)[x] = ((overall_min_lr&0xff00)>>8);
#else
      ((MM_U_8 *)Out->p[y].m)[x] = (overall_min_e_lr<<4)+ (overall_min_d_lr&0x000f);
      //printf("orig e:%d d:%d min:%d %d\n",overall_min_e_lr ,overall_min_d_lr, overall_min_lr, ((MM_U_8 *)Out->p[y].m)[x]);
      
      if ((y>8)&&(x>8)) {
	e_lr = e_rl = (((MM_U_8 *)Out->p[y-4].m)[x]>>4);
	d_lr = d_rl = (((MM_U_8 *)Out->p[y-4].m)[x])&0x000f;
	min_rl = ((MM_U_16 *)MEM(N,y-4,x)->p[e_lr].m)[d_lr];
	//printf("e:%d d:%d min:%d %d\n",e_rl,d_rl,min_rl, ((MM_U_8 *)Out->p[y-4].m)[x]);
	

	for (e=0;e<e_max;e++) {	   
	  y_rl = y-4+e_lr-e;
	  for (d=0;d<d_max;d++) {	   
	    x_rl = x+d_lr-d;
	    if ((x_rl>-1)&&(x_rl<x_max)&&
		(y_rl>-1)&&(y_rl<y_max)&&
		(min_rl > ((MM_U_16 *)MEM(N,y_rl-4,x_rl)->p[e].m)[d])) {
	      //((MM_U_8 *)Out->p[y-4].m)[x] = 0x00;
	      min_rl = ((MM_U_16 *)MEM(N,y_rl-4,x_rl)->p[e].m)[d];
	      e_rl = e;
	      d_rl = d;
	    }
	  }
	}
	//printf("e_rl:%d d_rl:%d\n",e_rl,d_rl);
	if ((abs(e_rl-e_lr)>1)||(abs(d_rl-d_lr)>1)) {
	  ((MM_U_8 *)Out->p[y-4].m)[x] = 0x00;
	}
	else 
	{
	  //((MM_U_8 *)Out->p[y-4].m)[x] = ((((MM_U_8 *)Out->p[y-4].m)[x])&0x0f); 
	  //((MM_U_8 *)Out->p[y-4].m)[x] = ((((MM_U_8 *)Out->p[y-4].m)[x])&0xf0)>>4;
        }
	
      }
#endif           
    }// for x
  }// for y

  emms();
  return 0;
}


// find pointer offset
// for win_BUF = 16
#define MEM2(base,y,x,e,d) (((MM_U_16 *)(base)->p[(y)&0x000f].m)[((((x)<<3)+(e))<<3)+(d)])
// for win_BUF = 8
//#define MEM2(base,y,x,e,d) (((MM_U_16 *)(base)->p[(y)&0x0007].m)[((((x)<<3)+(e))<<3)+(d)])


// assumes new P is in reg 0
// Q1(x,y+1) = Q1(x,y) - P(x,y) + P(x,y+win)
#define Q1inc2(Q1,x,y,e,d,P) \
{\
		  /* reg 1 = Q1*/\
		  movq_m2r (MEM2(Q1,y,x,e,d),mm1);\
		  /* subtract prev P*/\
		  psubw_m2r (MEM2(P,y,x,e,d),mm1);\
		  /* Add curr P to Q*/\
		  paddw_r2r (mm0,mm1);\
		  /* Store Q*/\
		  movq_r2m (mm1,MEM2(Q1,y+1,x,e,d));\
}

// assumes new P is in reg 0
// Q2(x+1,y) = Q2(x,y) - P(x,y) + P(x+win,y)
#define Q2inc2(Q2,x,y,e,d,P) \
{\
     /* reg 2 = Q2*/ \
     movq_m2r (MEM2(Q2,y,x,e,d),mm2); \
     /* subtract prev P */\
     psubw_m2r (MEM2(P,y,x,e,d),mm2);\
     /* Add curr P to Q*/\
     paddw_r2r (mm0,mm2);\
     /* Store Q*/ \
     movq_r2m (mm2,MEM2(Q2,y,x+1,e,d));\
}


// N[x][y_m][e][d] = N[x-1][y-1][e][d] 
//  + Q1[x-1+win][y-1][e][d] - Q1[x-1][y-1][e][d]
//  + Q2[x-1][y-1+win_m][e][d] - Q2[x-1][y-1][e][d] 
//  + P[x-1][y-1][e][d] + P[x-1+win][y-1+win][e][d]
//  - P[x-1+win][y-1][e][d] - P[x-1][y-1+win][e][d];       
// assumes new P is in reg 0
#define Ninc2(N,x,y,e,d,Q1,Q2,P) \
{\
     /* reg 2 = N */\
     movq_m2r (MEM2(N,y,x,e,d),mm3);\
     /* add Q1 */\
     paddw_m2r (MEM2(Q1,y,x+win,e,d),mm3);\
     /* subtract prev Q1 */\
     psubw_m2r (MEM2(Q1,y,x,e,d),mm3);\
     /* add Q2 */\
     paddw_m2r (MEM2(Q2,y+win,x,e,d),mm3);\
     /* subtract prev Q2 */\
     psubw_m2r (MEM2(Q2,y,x,e,d),mm3);\
     /* subtract prev P */\
     psubw_m2r (MEM2(P,y,x+win,e,d),mm3);\
     /* subtract prev P */\
     psubw_m2r (MEM2(P,y+win,x,e,d),mm3);\
     /* add prev P */\
     paddw_m2r (MEM2(P,y,x,e,d),mm3);\
     /* add curr P */\
     paddw_r2r (mm0,mm3);\
     /* Store N */\
     movq_r2m (mm3, MEM2(N,y+1,x+1,e,d));\
}

//----------------------------------------------------------------
int MMXRecursive_Depthmap_FLOW2(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out)
{ 
  const int win = 15;
  const int win_BUF = 16;
  //const int win = 7;
  //const int win_BUF = 8;
  //const short w_2 = 4; // win/2

  const int d_max = 8;
  const int e_max = 8;
  const short d_2 = 4; // d_max/2
  const short e_2 = 4; // e_max/2


  int x_max;
  int y_max;

  int i,j,d,e,x,y, x_1, y_1, x_1pwin, y_1pwin;
  int e_lr, d_lr, y_rl, x_rl, e_rl,d_rl,min_rl;


  unsigned short overall_min_lr;
  unsigned char overall_min_d_lr;
  unsigned char overall_min_e_lr;

  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t curr_e_lr;
  mmx_t min_d_lr;
  mmx_t min_e_lr;

  static int firsttime =1;
  static MMXMatrix *P;
  static MMXMatrix *Q1;
  static MMXMatrix *Q2;
  static MMXMatrix *N;

  x_max = I1->cols;
  y_max = I1->rows;

  if (firsttime) {
    P = MMXMatrixAlloc(MMT_U_16, d_max*e_max*x_max, win_BUF);
    Q1 = MMXMatrixAlloc(MMT_U_16, d_max*e_max*x_max, win_BUF);
    Q2 = MMXMatrixAlloc(MMT_U_16, d_max*e_max*x_max, win_BUF);
    N = MMXMatrixAlloc(MMT_U_16, d_max*e_max*x_max, win_BUF);

    firsttime =0;
  }

  emms();
  pxor_r2r(mm7,mm7);

  for(y=0;y<y_max-e_max-win;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    y_1 = y-1;
    y_1pwin = y-1+win;
    for(x=0;x<x_max-d_max-win;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      x_1 = x-1;
      x_1pwin = x-1+win;

      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      // set e
      movq_r2m(mm7,curr_e_lr);
 
      for (e=0;e<e_max;e++) {
	//fprintf(stderr,"depthmap: e:%d \n",e);

	// set d
	curr_d_lr.uw[0] = 0x000;
	curr_d_lr.uw[1] = 0x001;
	curr_d_lr.uw[2] = 0x002;
	curr_d_lr.uw[3] = 0x003;
	
	for (d=0;d<d_max-3;d+=4) {
	  //fprintf(stderr,"depthmap: d:%d \n",d);

	  if ((!x)&&(!y)) {
	    movq_r2m(mm7,MEM2(N,0,0,e,d));
	    // window across image
	    for(i=0;i<=win;i++) {
	      movq_r2m(mm7,MEM2(Q1,0,i,e,d));
	      movq_r2m(mm7,MEM2(Q2,i,0,e,d));
	    }
	    
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    // find Q1 & Q2
	    for(i=0;i<win;i++) {
	      // window down image
	      for (j=0;j<win;j++) {	       
		//P[i][j][e][d]= abs(((MM_U_8 *)I1->p[j+e_2].m)[i+d_2] - ((MM_U_8 *)I2->p[j+e].m)[i+d]);
		//Q1[i][0][e][d] += P[i][j][e][d];
		//Q2[0][j][e][d] += P[i][j][e][d];
		// compute |I1-I2| store result in reg 0
		I1_I2D(I1, I2,i+d_2,j+e_2,i+d,j+e);
		movq_r2m(mm0,MEM2(P,j,i,e,d));
		
		if (j<win) {
		  movq_m2r(MEM2(Q1,0,i,e,d),mm1);
		  paddw_r2r(mm0,mm1);
		  movq_r2m(mm1,MEM2(Q1,0,i,e,d));
		}
		if (i<win) { 
		  movq_m2r(MEM2(Q2,j,0,e,d),mm2);
		  paddw_r2r(mm0,mm2);
		  movq_r2m(mm2,MEM2(Q2,j,0,e,d));
		}
		if ((j<win)&&(i<win)) {
		  paddw_r2r(mm0,mm3);
		}
	      }
	    }
	    movq_r2m(mm3,MEM2(N,0,0,e,d));	    
	  }
	  else if (!x) {
	    // Q2 = reg 2
	    pxor_r2r(mm2,mm2);
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    for(i=0;i<win;i++) {
	      // compute |I1-I2| store result in reg 0
	      I1_I2D(I1, I2,i+d_2,y_1pwin+e_2,i+d,y_1pwin+e);
	      movq_r2m(mm0,MEM2(P,y_1pwin,i,e,d));
	      
	      Q1inc2(Q1,i,y_1,e,d,P);
	      
	      paddw_r2r(mm0,mm2);
	      paddw_r2r(mm1,mm3);
	    }
	    movq_r2m(mm2,MEM2(Q2,y_1pwin,0,e,d));
	    movq_r2m(mm3,MEM2(N,y,0,e,d));	    
	  }
	  else if (!y) {
	    // Q1 = reg 1
	    pxor_r2r(mm1,mm1);
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    for (j=0;j<win;j++) {
	      // compute |I1-I2| store result in reg 0
	      I1_I2D(I1, I2,x_1pwin+d_2,j+e_2,x_1pwin+d,j+e);
	      movq_r2m(mm0,MEM2(P,j,x_1pwin,e,d));

	      Q2inc2(Q2,x_1,j,e,d,P);

	      paddw_r2r(mm0,mm1);
	      paddw_r2r(mm2,mm3);
	    }
	    movq_r2m(mm1,MEM2(Q1,0,x_1pwin,e,d));
	    movq_r2m(mm3,MEM2(N,0,x,e,d));
	  }  
	  else {
	    //===  x & y > 0
	    I1_I2D(I1, I2,x_1pwin+d_2,y_1pwin+e_2,x_1pwin+d,y_1pwin+e);
	    movq_r2m(mm0,MEM2(P,y_1pwin,x_1pwin,e,d));
	    
	    Q1inc2(Q1,x_1pwin,y_1,e,d,P);  
	    Q2inc2(Q2,x_1,y_1pwin,e,d,P);
	    Ninc2(N,x_1,y_1,e,d,Q1,Q2,P);

	  }
	  
	  // ---------------------------------------------
	  // find minimum N Left->Right for C[x][y][e][d]    	
	  // ---------------------------------------------
	  // first create & store 0x0001000100010001 in 6
	  // for future reference
	  pxor_r2r(mm6,mm6);
	  pcmpeqw_r2r(mm1,mm1);
	  psubw_r2r(mm1,mm6);
	  //PRINTREG16(mm6,"0x0001");


	  // reg 3 = N 
	  //movq_m2r (((MM_U_16 *)MEM(N,y,x)->p[e].m)[d],mm3);
	  movq_r2r(mm3,mm0);

	  //reg 4 = minimum N
	  movq_m2r(min_lr,mm4);
	  movq_r2r(mm4,mm5);

	  //fill 1 with 0x80008000s (sign bit)
	  movq_r2r(mm6,mm1);
	  psllw_i2r(15,mm1);
	  //PRINTREG16(mm1,"0x800");

	  paddw_r2r(mm1,mm5);
	  paddw_r2r(mm1,mm0);
	  //PRINTREG16(mm5,"min_lr");
	  //PRINTREG16(mm0,"N");
        
	  // reg 5 becomes mask
	  pcmpgtw_r2r(mm0,mm5);
	  movq_r2r(mm5,mm1);
	
	  //PRINTREG16(mm2,"mask");
	
	  // reg 0 is N values smaller than minimum
	  movq_r2r(mm3,mm0);
	  pand_r2r(mm5,mm0);
		
	  // reg 4 is preserved minimum
	  pandn_r2r(mm4,mm1);
	
	  // reg 1 is new minimum
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_lr);
	  //PRINTREG16(mm1,"min_lr");

	  // -----------------------------------------------
	  // reg 4 is d values for minimum
	  movq_m2r(min_d_lr,mm4);
	  // reg 1 is preserved d values
	  movq_r2r(mm5,mm1);
	  pandn_r2r(mm4,mm1);

	  // reg 0 = current d value
	  movq_m2r(curr_d_lr,mm3);
	  movq_r2r(mm3,mm0);


	  // reg 0 = current d value for updated minimum
	  pand_r2r(mm5,mm0);
	
	  // reg 1 = new d's for minimum values
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_d_lr);
	  //PRINTREG16(mm1,"new_min_d_lr");

	  // increase current d's
	  // fill with 0x0004000400040004 
	  movq_r2r(mm6,mm0);
	  psllw_i2r(2,mm0);
	  //PRINTREG16(mm0,"0x0004");
	  paddw_r2r(mm0,mm3);
	  
	  movq_r2m(mm3,curr_d_lr);
	  //PRINTREG16(mm3,"curr_d_lr");

	  //-------------------------------------------------
	  // reg 4 is e values for minimum
	  movq_m2r(min_e_lr,mm4);
	  // reg 1 is preserved e values
	  movq_r2r(mm5,mm1);
	  pandn_r2r(mm4,mm1);

	  // reg 0 = current e value
	  movq_m2r(curr_e_lr,mm3);
	  movq_r2r(mm3,mm0);

	  // reg 0 = current e value for updated minimum
	  pand_r2r(mm5,mm0);
	
	  // reg 1 = new e's for minimum values
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_e_lr);
	  //PRINTREG16(mm1,"new_min_e_lr");
	  
	}// for d

	// increase current e's by 0x00010001...
	paddw_r2r(mm6,mm3);
	movq_r2m(mm3,curr_e_lr);
	//PRINTREG16(mm3,"curr_e_lr");
	
      }// for e
                  
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]    	
      // ---------------------------------------------
      overall_min_lr   = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      overall_min_e_lr = min_e_lr.uw[0];
      if (overall_min_lr > min_lr.uw[1]) {
	overall_min_lr   = min_lr.uw[1];
	overall_min_d_lr = min_d_lr.uw[1];
	overall_min_e_lr = min_e_lr.uw[1];
      }
      if (overall_min_lr > min_lr.uw[2]) {
	overall_min_lr   = min_lr.uw[2];
	overall_min_d_lr = min_d_lr.uw[2];
	overall_min_e_lr = min_e_lr.uw[2];
      }
      if (overall_min_lr > min_lr.uw[3]) {
	overall_min_lr   = min_lr.uw[3];
	overall_min_d_lr = min_d_lr.uw[3];
	overall_min_e_lr = min_e_lr.uw[3];
      }
      //((MM_U_8 *)Out->p[y].m)[x] = (overall_min_d_lr);
      //((MM_U_8 *)Out->p[y].m)[x] = (overall_min_e_lr);
      //#define rely
#ifdef rely
      ((MM_U_8 *)Out->p[y].m)[x] = ((overall_min_lr&0xff00)>>8);
#else
      ((MM_U_8 *)Out->p[y].m)[x] = (overall_min_e_lr<<4)+ (overall_min_d_lr&0x000f);
      //printf("orig e:%d d:%d min:%d %d\n",overall_min_e_lr ,overall_min_d_lr, overall_min_lr, ((MM_U_8 *)Out->p[y].m)[x]);
      
      if ((y>8)&&(x>8)) {
	e_lr = e_rl = (((MM_U_8 *)Out->p[y-4].m)[x]>>4);
	d_lr = d_rl = (((MM_U_8 *)Out->p[y-4].m)[x])&0x000f;
	min_rl = MEM2(N,y-4,x,e_lr,d_lr);
	//printf("e:%d d:%d min:%d %d\n",e_rl,d_rl,min_rl, ((MM_U_8 *)Out->p[y-4].m)[x]);
	

	for (e=0;e<e_max;e++) {	   
	  y_rl = y-4+e_lr-e;
	  for (d=0;d<d_max;d++) {	   
	    x_rl = x+d_lr-d;
	    if ((x_rl>-1)&&(x_rl<x_max)&&
		(y_rl>-1)&&(y_rl<y_max)&&
		(min_rl > MEM2(N,y_rl-4,x_rl,e,d))) {
	      //((MM_U_8 *)Out->p[y-4].m)[x] = 0x00;
	      min_rl = MEM2(N,y_rl-4,x_rl,e,d);
	      e_rl = e;
	      d_rl = d;
	    }
	  }
	}
	//printf("e_rl:%d d_rl:%d\n",e_rl,d_rl);
	if ((abs(e_rl-e_lr)>1)||(abs(d_rl-d_lr)>1)) {
	  ((MM_U_8 *)Out->p[y-4].m)[x] = 0x00;
	}
	else 
	{
	  //((MM_U_8 *)Out->p[y-4].m)[x] = ((((MM_U_8 *)Out->p[y-4].m)[x])&0x0f); 
	  //((MM_U_8 *)Out->p[y-4].m)[x] = ((((MM_U_8 *)Out->p[y-4].m)[x])&0xf0)>>4;
        }
	
      }
#endif           
    }// for x
  }// for y

  emms();
  return 0;
}





// assumes new P is in reg 0
// Q1(x,y+1) = Q1(x,y) - P(x,y) + P(x,y+win)
#define Q1inc3(Q1_x_1pwin_y_1,Q1_x_1pwin_y,P_x_1pwin_y_1) \
{\
		  /* reg 1 = Q1*/\
		  movq_m2r (*Q1_x_1pwin_y_1,mm1);\
		  /* subtract prev P*/\
		  psubw_m2r (*P_x_1pwin_y_1,mm1);\
		  /* Add curr P to Q*/\
		  paddw_r2r (mm0,mm1);\
		  /* Store Q*/\
		  movq_r2m (mm1,*Q1_x_1pwin_y);\
}


//----------------------------------------------------------------
int MMXRecursive_Depthmap_FLOW3(MMXMatrix *I1, MMXMatrix *I2, MMXMatrix  *Out)
{ 
  const int win = 15;
  const int win_BUF = 16;
  //const int win = 7;
  //const int win_BUF = 8;
  //const short w_2 = 4; // win/2

  const int d_max = 8;
  const int e_max = 8;
  const short d_2 = 4; // d_max/2
  const short e_2 = 4; // e_max/2


  int x_max;
  int y_max;

  int i,j,d,e,x,y, x_1, y_1, x_1pwin, y_1pwin;
  int e_lr, d_lr, y_rl, x_rl, e_rl,d_rl,min_rl;


  unsigned short overall_min_lr;
  unsigned char overall_min_d_lr;
  unsigned char overall_min_e_lr;

  mmx_t min_lr;
  mmx_t curr_d_lr;
  mmx_t curr_e_lr;
  mmx_t min_d_lr;
  mmx_t min_e_lr;

  static int firsttime =1;
  static MMXMatrix *P;
  static MMXMatrix *Q1;
  static MMXMatrix *Q2;
  static MMXMatrix *N;

  unsigned short *P_x_1pwin_y_1;
  unsigned short *Q1_x_1pwin_y, *Q1_x_1pwin_y_1;


  x_max = I1->cols;
  y_max = I1->rows;

  if (firsttime) {
    P = MMXMatrixAlloc(MMT_U_16, d_max*e_max*x_max, win_BUF);
    Q1 = MMXMatrixAlloc(MMT_U_16, d_max*e_max*x_max, win_BUF);
    Q2 = MMXMatrixAlloc(MMT_U_16, d_max*e_max*x_max, win_BUF);
    N = MMXMatrixAlloc(MMT_U_16, d_max*e_max*x_max, win_BUF);

    firsttime =0;
  }

  emms();
  pxor_r2r(mm7,mm7);

  for(y=0;y<y_max-e_max-win;y++){
    //fprintf(stderr,"depthmap: y:%d \n",y);
    y_1 = y-1;
    y_1pwin = y-1+win;


    P_x_1pwin_y_1 = &(MEM2(Q1,win-1,y_1,0,0));
    Q1_x_1pwin_y_1 = &(MEM2(Q1,win-1,y_1,0,0));
    Q1_x_1pwin_y = &(MEM2(Q1,win-1,y,0,0));

    for(x=0;x<x_max-d_max-win;x++) {
      //fprintf(stderr,"depthmap: x:%d \n",x);
      x_1 = x-1;
      x_1pwin = x-1+win;

      // fill min with 111111111111111s (max number)
      pcmpeqw_r2r(mm0,mm0);
      movq_r2m(mm0,min_lr);

      // set e
      movq_r2m(mm7,curr_e_lr);

 
      for (e=0;e<e_max;e++) {
	//fprintf(stderr,"depthmap: e:%d \n",e);

	// set d
	curr_d_lr.uw[0] = 0x000;
	curr_d_lr.uw[1] = 0x001;
	curr_d_lr.uw[2] = 0x002;
	curr_d_lr.uw[3] = 0x003;
	
	for (d=0;d<d_max-3;d+=4) {
	  //fprintf(stderr,"depthmap: d:%d \n",d);

	  if ((!x)&&(!y)) {
	    movq_r2m(mm7,MEM2(N,0,0,e,d));
	    // window across image
	    for(i=0;i<=win;i++) {
	      movq_r2m(mm7,MEM2(Q1,0,i,e,d));
	      movq_r2m(mm7,MEM2(Q2,i,0,e,d));
	    }
	    
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    // find Q1 & Q2
	    for(i=0;i<win;i++) {
	      // window down image
	      for (j=0;j<win;j++) {	       
		//P[i][j][e][d]= abs(((MM_U_8 *)I1->p[j+e_2].m)[i+d_2] - ((MM_U_8 *)I2->p[j+e].m)[i+d]);
		//Q1[i][0][e][d] += P[i][j][e][d];
		//Q2[0][j][e][d] += P[i][j][e][d];
		// compute |I1-I2| store result in reg 0
		I1_I2D(I1, I2,i+d_2,j+e_2,i+d,j+e);
		movq_r2m(mm0,MEM2(P,j,i,e,d));
		
		if (j<win) {
		  movq_m2r(MEM2(Q1,0,i,e,d),mm1);
		  paddw_r2r(mm0,mm1);
		  movq_r2m(mm1,MEM2(Q1,0,i,e,d));
		}
		if (i<win) { 
		  movq_m2r(MEM2(Q2,j,0,e,d),mm2);
		  paddw_r2r(mm0,mm2);
		  movq_r2m(mm2,MEM2(Q2,j,0,e,d));
		}
		if ((j<win)&&(i<win)) {
		  paddw_r2r(mm0,mm3);
		}
	      }
	    }
	    movq_r2m(mm3,MEM2(N,0,0,e,d));	     
	  }
	  else if (!x) {
	    // Q2 = reg 2
	    pxor_r2r(mm2,mm2);
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    for(i=0;i<win;i++) {
	      // compute |I1-I2| store result in reg 0
	      I1_I2D(I1, I2,i+d_2,y_1pwin+e_2,i+d,y_1pwin+e);
	      movq_r2m(mm0,MEM2(P,y_1pwin,i,e,d));
	      
	      Q1inc2(Q1,i,y_1,e,d,P);
	      
	      paddw_r2r(mm0,mm2);
	      paddw_r2r(mm1,mm3);
	    }
	    movq_r2m(mm2,MEM2(Q2,y_1pwin,0,e,d));
	    movq_r2m(mm3,MEM2(N,y,0,e,d));	    

	  }
	  else if (!y) {
	    // Q1 = reg 1
	    pxor_r2r(mm1,mm1);
	    // N = reg 3
	    pxor_r2r(mm3,mm3);
	    for (j=0;j<win;j++) {
	      // compute |I1-I2| store result in reg 0
	      I1_I2D(I1, I2,x_1pwin+d_2,j+e_2,x_1pwin+d,j+e);
	      movq_r2m(mm0,MEM2(P,j,x_1pwin,e,d));

	      Q2inc2(Q2,x_1,j,e,d,P);

	      paddw_r2r(mm0,mm1);
	      paddw_r2r(mm2,mm3);
	    }
	    movq_r2m(mm1,MEM2(Q1,0,x_1pwin,e,d));
	    movq_r2m(mm3,MEM2(N,0,x,e,d));

	  }  
	  else {
	    //===  x & y > 0
	    I1_I2D(I1, I2,x_1pwin+d_2,y_1pwin+e_2,x_1pwin+d,y_1pwin+e);
	    movq_r2m(mm0,MEM2(P,y_1pwin,x_1pwin,e,d));
	    
	    //Q1inc2(Q1,x_1pwin,y_1,e,d,P);  
	    Q1inc3(Q1_x_1pwin_y_1,Q1_x_1pwin_y,P_x_1pwin_y_1);  

	    Q2inc2(Q2,x_1,y_1pwin,e,d,P);
	    Ninc2(N,x_1,y_1,e,d,Q1,Q2,P);

	  }
	  
	  // ---------------------------------------------
	  // find minimum N Left->Right for C[x][y][e][d]    	
	  // ---------------------------------------------
	  // first create & store 0x0001000100010001 in 6
	  // for future reference
	  pxor_r2r(mm6,mm6);
	  pcmpeqw_r2r(mm1,mm1);
	  psubw_r2r(mm1,mm6);
	  //PRINTREG16(mm6,"0x0001");


	  // reg 3 = N 
	  //movq_m2r (((MM_U_16 *)MEM(N,y,x)->p[e].m)[d],mm3);
	  movq_r2r(mm3,mm0);

	  //reg 4 = minimum N
	  movq_m2r(min_lr,mm4);
	  movq_r2r(mm4,mm5);

	  //fill 1 with 0x80008000s (sign bit)
	  movq_r2r(mm6,mm1);
	  psllw_i2r(15,mm1);
	  //PRINTREG16(mm1,"0x800");

	  paddw_r2r(mm1,mm5);
	  paddw_r2r(mm1,mm0);
	  //PRINTREG16(mm5,"min_lr");
	  //PRINTREG16(mm0,"N");
        
	  // reg 5 becomes mask
	  pcmpgtw_r2r(mm0,mm5);
	  movq_r2r(mm5,mm1);
	
	  //PRINTREG16(mm2,"mask");
	
	  // reg 0 is N values smaller than minimum
	  movq_r2r(mm3,mm0);
	  pand_r2r(mm5,mm0);
		
	  // reg 4 is preserved minimum
	  pandn_r2r(mm4,mm1);
	
	  // reg 1 is new minimum
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_lr);
	  //PRINTREG16(mm1,"min_lr");

	  // -----------------------------------------------
	  // reg 4 is d values for minimum
	  movq_m2r(min_d_lr,mm4);
	  // reg 1 is preserved d values
	  movq_r2r(mm5,mm1);
	  pandn_r2r(mm4,mm1);

	  // reg 0 = current d value
	  movq_m2r(curr_d_lr,mm3);
	  movq_r2r(mm3,mm0);


	  // reg 0 = current d value for updated minimum
	  pand_r2r(mm5,mm0);
	
	  // reg 1 = new d's for minimum values
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_d_lr);
	  //PRINTREG16(mm1,"new_min_d_lr");

	  // increase current d's
	  // fill with 0x0004000400040004 
	  movq_r2r(mm6,mm0);
	  psllw_i2r(2,mm0);
	  //PRINTREG16(mm0,"0x0004");
	  paddw_r2r(mm0,mm3);
	  
	  movq_r2m(mm3,curr_d_lr);
	  //PRINTREG16(mm3,"curr_d_lr");

	  //-------------------------------------------------
	  // reg 4 is e values for minimum
	  movq_m2r(min_e_lr,mm4);
	  // reg 1 is preserved e values
	  movq_r2r(mm5,mm1);
	  pandn_r2r(mm4,mm1);

	  // reg 0 = current e value
	  movq_m2r(curr_e_lr,mm3);
	  movq_r2r(mm3,mm0);

	  // reg 0 = current e value for updated minimum
	  pand_r2r(mm5,mm0);
	
	  // reg 1 = new e's for minimum values
	  por_r2r(mm0,mm1);
	  movq_r2m(mm1,min_e_lr);
	  //PRINTREG16(mm1,"new_min_e_lr");

	  P_x_1pwin_y_1 +=8;
	  Q1_x_1pwin_y_1 +=8;
	  Q1_x_1pwin_y +=8;
	  
	}// for d

	// increase current e's by 0x00010001...
	paddw_r2r(mm6,mm3);
	movq_r2m(mm3,curr_e_lr);
	//PRINTREG16(mm3,"curr_e_lr");
	
      }// for e
                  
      // ---------------------------------------------
      // find overall minimum C Left->Right for C[x+1][d]    	
      // ---------------------------------------------
      overall_min_lr   = min_lr.uw[0];
      overall_min_d_lr = min_d_lr.uw[0];
      overall_min_e_lr = min_e_lr.uw[0];
      if (overall_min_lr > min_lr.uw[1]) {
	overall_min_lr   = min_lr.uw[1];
	overall_min_d_lr = min_d_lr.uw[1];
	overall_min_e_lr = min_e_lr.uw[1];
      }
      if (overall_min_lr > min_lr.uw[2]) {
	overall_min_lr   = min_lr.uw[2];
	overall_min_d_lr = min_d_lr.uw[2];
	overall_min_e_lr = min_e_lr.uw[2];
      }
      if (overall_min_lr > min_lr.uw[3]) {
	overall_min_lr   = min_lr.uw[3];
	overall_min_d_lr = min_d_lr.uw[3];
	overall_min_e_lr = min_e_lr.uw[3];
      }
      //((MM_U_8 *)Out->p[y].m)[x] = (overall_min_d_lr);
      //((MM_U_8 *)Out->p[y].m)[x] = (overall_min_e_lr);
      //#define rely
#ifdef rely
      ((MM_U_8 *)Out->p[y].m)[x] = ((overall_min_lr&0xff00)>>8);
#else
      ((MM_U_8 *)Out->p[y].m)[x] = (overall_min_e_lr<<4)+ (overall_min_d_lr&0x000f);
      //printf("orig e:%d d:%d min:%d %d\n",overall_min_e_lr ,overall_min_d_lr, overall_min_lr, ((MM_U_8 *)Out->p[y].m)[x]);
      
      if ((y>8)&&(x>8)) {
	e_lr = e_rl = (((MM_U_8 *)Out->p[y-4].m)[x]>>4);
	d_lr = d_rl = (((MM_U_8 *)Out->p[y-4].m)[x])&0x000f;
	min_rl = MEM2(N,y-4,x,e_lr,d_lr);
	//printf("e:%d d:%d min:%d %d\n",e_rl,d_rl,min_rl, ((MM_U_8 *)Out->p[y-4].m)[x]);
	

	for (e=0;e<e_max;e++) {	   
	  y_rl = y-4+e_lr-e;
	  for (d=0;d<d_max;d++) {	   
	    x_rl = x+d_lr-d;
	    if ((x_rl>-1)&&(x_rl<x_max)&&
		(y_rl>-1)&&(y_rl<y_max)&&
		(min_rl > MEM2(N,y_rl-4,x_rl,e,d))) {
	      //((MM_U_8 *)Out->p[y-4].m)[x] = 0x00;
	      min_rl = MEM2(N,y_rl-4,x_rl,e,d);
	      e_rl = e;
	      d_rl = d;
	    }
	  }
	}
	//printf("e_rl:%d d_rl:%d\n",e_rl,d_rl);
	if ((abs(e_rl-e_lr)>1)||(abs(d_rl-d_lr)>1)) {
	  ((MM_U_8 *)Out->p[y-4].m)[x] = 0x00;
	}
	else 
	{
	  //((MM_U_8 *)Out->p[y-4].m)[x] = ((((MM_U_8 *)Out->p[y-4].m)[x])&0x0f); 
	  //((MM_U_8 *)Out->p[y-4].m)[x] = ((((MM_U_8 *)Out->p[y-4].m)[x])&0xf0)>>4;
        }
	
      }
#endif           
    }// for x
  }// for y

  emms();
  return 0;
}














