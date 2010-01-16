#include <iostream>
using namespace std;

#define MAX_JN 6
#define MAX_FILTER_ORDER 6

double lpf_ord1_3hz(double input, int j)
{ 
	if (j<0 || j> MAX_JN)
	{
		cout<<"Received an invalid joint index to filter"<<endl;
		return 0;
	}

	static double xv[MAX_FILTER_ORDER][MAX_JN];
	static double yv[MAX_FILTER_ORDER][MAX_JN];
	xv[0][j] = xv[1][j] ; 
    xv[1][j] = input / 1.157889499e+01;
    yv[0][j] = yv[1][j] ; 
    yv[1][j] =   (xv[0][j]  + xv[1][j] ) + (  0.8272719460 * yv[0][j] );
    return (yv[1][j]);
}
