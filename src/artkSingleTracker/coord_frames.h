/*

Coordinate frames for: 1) reference pattern, 2) robot, 3) target in
 a) the camera coordinate system or
 b) the reference pattern system
 i.e. a total of 6=3*2 coordinate frames

16.4.08, 3.5.08 (better timers) J. Gaspar

*/

#ifndef _COORD_FRAMES
#define _COORD_FRAMES

class cCoordFrames {
public:
    double *data;
    int nRows, nCols;
    unsigned int *tickTimes;
    unsigned int currTickTime;

    cCoordFrames();
    ~cCoordFrames();

    void myError(char *str);

    void currTickTimeIncr();

    void setEntry(int id, int i, int j, double x);
    void setTransf(double src[3][4], int dst);
    void invertTransf(int src, int dst);
    void composeTransf(int s1, int s2, int dst);

    double getEntry(int id, int i, int j);
    double *getFrame(int src, int *nValues);
    double *getAllFrames(int *nValues);
    void setAllFrames(double *vp, int nValues);

    unsigned int *getTickTimes(int *nValues);
    void setTickTimes(unsigned int *vp, int nValues);
    int tickTimeStatus(int id);

    void getXYZ(int id, double *x, double *y, double *z);
    void getXYTheta(int id, double *x, double *y, double *theta);
};

#endif /* _COORD_FRAMES */
