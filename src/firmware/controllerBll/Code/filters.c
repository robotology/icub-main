#include "filters.h"
#include "controller.h"
#include "pid.h"


Int32 lpf_ord1_3hz(Int32 input, int j)
{
		static float x_filt[6][JN],  y_filt[6][JN];	
       	//order1 3Hz
		x_filt[0][j] = x_filt[1][j]; 
        x_filt[1][j] = input / 1.071001538e+02;
        y_filt[0][j] = y_filt[1][j]; 
        y_filt[1][j] = (x_filt[0][j] + x_filt[1][j])
        	 	  + (  0.9813258905  * y_filt[0][j]);
        return (Int32)(y_filt[1][j]);
}

Int32 lpf_ord1_30hz(Int32 input, int j)
{
		static float x_filt[6][JN],  y_filt[6][JN];	
		//order1 30Hz
		x_filt[0][j] = x_filt[1][j]; 
        x_filt[1][j] = input / 1.157889499e+01;
        y_filt[0][j] = y_filt[1][j]; 
        y_filt[1][j] =   (x_filt[0][j] + x_filt[1][j])
                      + (  0.8272719460  * y_filt[0][j]);
        return (Int32)(y_filt[1][j]);
}

         
Int32 lpf_ord2_30hz(Int32 input, int j)
{		
		static float x_filt[6][JN],  y_filt[6][JN];	
        //order2 30Hz         
        x_filt[0][j] = x_filt[1][j]; x_filt[1][j] = x_filt[2][j]; 
        x_filt[2][j] = input / 1.278738361e+02;
        y_filt[0][j] = y_filt[1][j]; y_filt[1][j] = y_filt[2][j]; 
        y_filt[2][j] =   (x_filt[0][j] + x_filt[2][j]) + 2 * x_filt[1][j]
                     + ( -0.7660066009 * y_filt[0][j]) + (  1.7347257688 * y_filt[1][j]);
        return (Int32)(y_filt[2][j]);
}
   


Int32 lpf_ord4_30hz(Int32 input, int j)
{
		static float x_filt[6][JN],  y_filt[6][JN];	
		//order4 30Hz
        x_filt[0][j] = x_filt[1][j]; x_filt[1][j] = x_filt[2][j]; x_filt[2][j] = x_filt[3][j]; x_filt[3][j] = x_filt[4][j]; 
        x_filt[4][j] = input / 1.602898462e+04;
        y_filt[0][j] = y_filt[1][j]; y_filt[1][j] = y_filt[2][j]; y_filt[2][j] = y_filt[3][j]; y_filt[3][j] = y_filt[4][j]; 
        y_filt[4][j] =   (x_filt[0][j] + x_filt[4][j]) + 4 * (x_filt[1][j] + x_filt[3][j]) + 6 * x_filt[2][j]
                     + ( -0.6105348076 * y_filt[0][j]) + (  2.7426528211 * y_filt[1][j])
                     + ( -4.6409024127 * y_filt[2][j]) + (  3.5077862074 * y_filt[3][j]);
        return (Int32)(y_filt[4][j]);	
}




