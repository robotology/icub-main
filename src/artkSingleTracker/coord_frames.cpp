/*

Coordinate frames for: 1) reference pattern, 2) robot, 3) target in
 a) the camera coordinate system or
 b) the reference pattern system
 i.e. a total of 6=3*2 coordinate frames

16.4.08, 3.5.08 (better timers) J. Gaspar

*/


#include <iostream>
using namespace std;
#include "coord_frames.h"
#include <math.h>
#define PI 3.14159265

#define NFRAMES 6

// note the i,j indexes run as in mathematics / matlab: i is line, j is column
#define dataEntry(id,i,j) data[id*12+i*4+j]


cCoordFrames::cCoordFrames()
{
    data= new double[NFRAMES*12];

    // place identity matrices everywhere
    for (int i=0; i<NFRAMES; i++)
        for (int j=0; j<12; j++)
            if (j==0 || j==5 || j==10)
                data[i*12+j]= 1.0;
            else
                data[i*12+j]= 0.0;

    // timing the events
    tickTimes= new unsigned int[NFRAMES];
    for (int i=0; i<NFRAMES; i++)
        tickTimes[i]= 0;

    // tick counter to have an idea of "touching" times
    currTickTime= 0;

    // 
    nRows= NFRAMES;
    nCols= 12;
}


cCoordFrames::~cCoordFrames()
{
    delete[] data;
    delete[] tickTimes;
}


void cCoordFrames::myError(char *str)
{
    cout << "Error: " << str << endl;
}


void cCoordFrames::currTickTimeIncr()
{
    unsigned int tickTime= currTickTime; // save currTickTime

    currTickTime += 1;

    /* Detect increment failure (due to overflow) and reset all timers if so.
       Most probably this will never happen since 4 bytes integer implies:
       2^32/25/3600/24/365 = 5.4477 years
    */
    if (currTickTime < tickTime) {
        currTickTime=1;
        for (int i=0; i<NFRAMES; i++)
            tickTimes[i]= 0;
    }
}


#define touchFrame(id) tickTimes[id]= currTickTime

void cCoordFrames::setEntry(int id, int i, int j, double x)
{
    if (id<0 || NFRAMES<=id) {
        myError("Invalid frame id"); return;
    }
    if (i<0 || 2<i) {
        myError("i is not in 0..2 (max 3 lines)"); return;
    }
    if (j<0 || 3<j) {
        myError("j is not in 0..3 (max 4 cols)"); return;
    }
    dataEntry(id,i,j)= x;

    // save the touching time
    touchFrame(id);
}


double cCoordFrames::getEntry(int id, int i, int j)
{
    if (id<0 || NFRAMES<=id) {
        myError("Invalid frame id"); return 0;
    }
    if (i<0 || 2<i) {
        myError("i is not in 0..2 (max 3 lines)"); return 0;
    }
    if (j<0 || 3<j) {
        myError("j is not in 0..3 (max 4 cols)"); return 0;
    }
    return dataEntry(id,i,j);
}


void cCoordFrames::setTransf(double src[3][4], int dst)
{
    if (dst<0 || NFRAMES<=dst) {
        myError("invalid dst id"); return;
    }
    for (int i=0; i<3; i++)
        for (int j=0; j<4; j++)
            setEntry(dst,i,j, src[i][j]);
}


void cCoordFrames::invertTransf(int src, int dst)
{
    if (src<0 || NFRAMES<=src) {
        myError("invalid src id"); return;
    }
    if (dst<0 || NFRAMES<=dst) {
        myError("invalid dst id"); return;
    }
    if (src==dst) {
        myError("dst id == src id"); return;
    }

    // transpose the rotation
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            setEntry(dst, i,j, dataEntry(src,j,i)); // notice the swaping i,j

    // t -> -R'*t
    dataEntry(dst,0,3)=
        -(dataEntry(dst,0,0)*dataEntry(src,0,3)
        + dataEntry(dst,0,1)*dataEntry(src,1,3)
        + dataEntry(dst,0,2)*dataEntry(src,2,3) );
    dataEntry(dst,1,3)=
        -(dataEntry(dst,1,0)*dataEntry(src,0,3)
        + dataEntry(dst,1,1)*dataEntry(src,1,3)
        + dataEntry(dst,1,2)*dataEntry(src,2,3) );
    dataEntry(dst,2,3)=
        -(dataEntry(dst,2,0)*dataEntry(src,0,3)
        + dataEntry(dst,2,1)*dataEntry(src,1,3)
        + dataEntry(dst,2,2)*dataEntry(src,2,3) );

    // save the touching time
    touchFrame(dst);
}


void cCoordFrames::composeTransf(int s1, int s2, int dst)
{
    if (s1<0 || NFRAMES<=s1) {
        myError("invalid src id 1"); return;
    }
    if (s2<0 || NFRAMES<=s2) {
        myError("invalid src id 1"); return;
    }
    if (dst<0 || NFRAMES<=dst) {
        myError("invalid dst id"); return;
    }
    if (dst==s1 || dst==s2) {
        myError("dst id == src id1 or id2"); return;
    }

    // [R1 t1; 0 1]*[R2 t2; 0 1]= [R1*R2  R1*t2+t1; 0 1]

    // the rotation part: R1*R2 -> R
    for (int i=0; i<3; i++) { // i==line j==column
        for (int j=0; j<3; j++) {
            dataEntry(dst,i,j)= 0.0;
            for (int k=0; k<3; k++)
                dataEntry(dst,i,j) += dataEntry(s1,i,k)*dataEntry(s2,k,j);
            }
    }

    // the translation part: R1*t2+t1 -> t
    for (int i=0; i<3; i++) {
        dataEntry(dst,i,3)= dataEntry(s1,i,3)
            +dataEntry(s1,i,0)*dataEntry(s2,0,3)
            +dataEntry(s1,i,1)*dataEntry(s2,1,3)
            +dataEntry(s1,i,2)*dataEntry(s2,2,3);
    }

    // save the touching time
    touchFrame(dst);
}


double * cCoordFrames::getFrame(int src, int *nValues)
{
    if (src<0 || NFRAMES<=src) {
        myError("invalid src id"); return 0;
    }
    if (nValues != 0)
        *nValues= 12;

    return &(dataEntry(src,0,0));
}


double * cCoordFrames::getAllFrames(int *nValues)
{
    if (nValues != 0)
        *nValues= NFRAMES*12;

    return data;
}


unsigned int * cCoordFrames::getTickTimes(int *nValues)
{
    if (nValues != 0)
        *nValues= NFRAMES;

    return tickTimes;
}


void cCoordFrames::setAllFrames(double *vp, int nValues)
{
    if (vp==0) return;

    double *p; int n;
    p= getAllFrames(&n);
    if (nValues > n) {
        myError("frames array too long."); return;
    }
    for (int i=0; i<nValues; i++)
        p[i]= vp[i];
}


void cCoordFrames::setTickTimes(unsigned int *vp, int nValues)
{
    if (vp==0) return;

    unsigned int *p; int n;
    p= getTickTimes(&n);
    if (nValues > n) {
        myError("tickTimes array too long."); return;
    }

    /* update the tickTimes array and
       the currTickTime to the maximum of the tickTimes
    */
    for (int i=0; i<nValues; i++) {
        p[i]= vp[i];
        if (p[i] > (double)currTickTime)
            currTickTime= (unsigned long int)p[i];
    }
}


/* compare the frame id tickTime with the tickTime of the reference frame
returns:
 0 in the ideal situation of watching the reference 
   and id frames in the same tickTime
 non-zero otherwise
*/
int cCoordFrames::tickTimeStatus(int id)
{
    if (id<0 || NFRAMES<=id) {
        myError("Invalid frame id"); return -1;
    }

    unsigned int *p; int n;
    p= getTickTimes(&n);

    int ret;
    if (id>0 && p[id]==p[0]) /* frame id and reference (0) seen in the same tickTime */
        ret= 0;
    else if (p[0]<1) /* never seen the reference frame */
        ret= 1;
    else if (p[0]<p[id]) /* reference frame is old */
        ret= 2;
    else /* reference frame is newer */
        ret= 3;

    return ret;
}


/* get a target location from its coordinate system (frame):
xyz come directly from the translation vector (4th column)
*/
void cCoordFrames::getXYZ(int id, double *x, double *y, double *z)
{
    if (x!=0)
        *x= getEntry(id, 0,3);
    if (y!=0)
        *y= getEntry(id, 1,3);
    if (z!=0)
        *z= getEntry(id, 2,3);
}


/* get a wheeled mobile robot location from its coordinate system (frame):
xy come directly from the translation vector (4th column)
theta is the angle of the Ox axis in the Oxy plane of the world frame
*/
void cCoordFrames::getXYTheta(int id, double *x, double *y, double *theta)
{
    if (x!=0)
        *x= getEntry(id, 0,3);
    if (y!=0)
        *y= getEntry(id, 1,3);
    if (theta!=0)
        *theta= atan2(getEntry(id, 1,0), getEntry(id, 0,0)) *180/PI;
}
