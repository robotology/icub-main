// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <assert.h>
#include <math.h>
#include <iostream>

#include "HarmonicFunction.h"

#include <highgui.h>

#include <yarp/math/Math.h>

using namespace std;
using namespace iCub;
using namespace yarp::sig;
using namespace yarp::math;

HarmonicFunction::HarmonicFunction(Vector min, Vector max, Vector res, double om=0.9, int maxIter=500, double thresh=1e-4, double plotFac=0.001, bool dispMode=true) :
    convergenceThreshold(thresh),
    omega(om),
    plotScaleFactor(plotFac),
    maxIterations(maxIter),
    dims(2),
    displayMode(dispMode),
    pathComputed(false),
    leaveRobotTrace(false),
    rangeMin(min),
    rangeMax(max),
    resolution(res),
    potential(4),
    noOfBins(dims),
    grad(dims)
{

    cout << "Creating HarmonicFunction with size: " << dims << endl;  
    assert(min.size() == dims && max.size() == dims && res.size() == dims);

    init();

}

HarmonicFunction::~HarmonicFunction() {
    if(displayMode) {
        cvReleaseImage(&potentialImage);
    }
    cvReleaseImage(&worldImage);
}

void HarmonicFunction::init() {

    // set map dimensions
    for(int i=0; i<dims; i++) {
        noOfBins[i] = (int)((rangeMax[i] - rangeMin[i])/resolution[i]) + 1;
        cout << "noOfBins[" << i << "] : " << noOfBins[i] << endl;
    }

    // create images
    if(displayMode) {
        potentialImage = cvCreateImage(cvSize(noOfBins[0],noOfBins[1]),IPL_DEPTH_8U,3);        potentialImage = cvCreateImage(cvSize(noOfBins[0],noOfBins[1]),IPL_DEPTH_8U,3);
    }
    worldImage = cvCreateImage(cvSize(noOfBins[0],noOfBins[1]),IPL_DEPTH_8U,3);

    // set potential values for different types of things
    potential[FREESPACE] = FREESPACE_POTENTIAL;
    potential[OBSTACLE] = OBSTACLE_POTENTIAL;
    potential[DIRICHLET] = DIRICHLET_POTENTIAL;
    potential[GOAL] = GOAL_POTENTIAL;

    // set colors for world map image
    freespaceColor = CV_RGB(255,255,255);
    goalColor = CV_RGB(0,0,255);
    obstacleColor = CV_RGB(0,0,0);
    dirichletColor = CV_RGB(0,255,0);

    // set values of images/maps

    CvScalar pot;
    for(int i=0; i<noOfBins[0]; i++) {
        for(int j=0; j<noOfBins[1]; j++) {
            pair<int,int> p(i,j);
            if(i == 0 || j == 0 || i == noOfBins[0]-1 || j == noOfBins[1]-1) {
                worldMap[p] = DIRICHLET;
                cvSet2D(worldImage,i,j,dirichletColor);
            } else{
                worldMap[p] = FREESPACE;
                cvSet2D(worldImage,i,j,freespaceColor);
            }	    
            potentialMap[p] =  potential[worldMap[p]];
            if(displayMode) {
                pot = CV_RGB(potentialMap[p]*255,potentialMap[p]*255,potentialMap[p]*255);
                cvSet2D(potentialImage,i,j,pot);
            }
        }
    }
    
    //Flag value is used to decided if the matrices have been intialized
    cout << "HARMONIC MAP INITIALIZED" << endl;
    
}

void HarmonicFunction::loadMap(string map) {
    IplImage *img = cvLoadImage(map.c_str());

    // scale to internal range at this point
    cvResize(img, worldImage, CV_INTER_NN);
    cvReleaseImage(&img);

    CvScalar pot;
    int w = worldImage->width;
    int h = worldImage->height;
    int step = worldImage->widthStep;
    int channels = worldImage->nChannels;
    uchar *data = (uchar *)worldImage->imageData;

    if(w!=noOfBins[0] || h!=noOfBins[1]) {
        cout << "Size mismatch in reading map from file..." << endl;
        return;
    }

    // go though the map that was loaded in and set:
    // - the worldMap 
    // - the potentialMap
    // - the potentialImage
    for(int i=0; i<noOfBins[0]; i++) {
        for(int j=0; j<noOfBins[1]; j++) {

            pair<int,int> p(i,j);
            
            if( data[i*step+j*channels+0] == dirichletColor.val[0] &&
                data[i*step+j*channels+1] == dirichletColor.val[1] &&
                data[i*step+j*channels+2] == dirichletColor.val[2] ) {
                worldMap[p]=DIRICHLET;
            } else if( data[i*step+j*channels+0] == goalColor.val[0] &&
                data[i*step+j*channels+1] == goalColor.val[1] &&
                data[i*step+j*channels+2] == goalColor.val[2] ) {
                worldMap[p]=GOAL;
            } else if( data[i*step+j*channels+0] == obstacleColor.val[0] &&
                data[i*step+j*channels+1] == obstacleColor.val[1] &&
                data[i*step+j*channels+2] == obstacleColor.val[2] ) {
                worldMap[p]=OBSTACLE;
            } else {
                worldMap[p]=FREESPACE;
            }

            potentialMap[p] =  potential[worldMap[p]];
            
            if(displayMode) {
                if(worldMap[p]==OBSTACLE) {
                    pot = CV_RGB(0,0,255);
                    cvSet2D(potentialImage,i,j,pot);
               } else {
                    pot = CV_RGB(potentialMap[p]*255,potentialMap[p]*255,potentialMap[p]*255);
                    cvSet2D(potentialImage,i,j,pot);
                }
            }
        }
    }


}

void HarmonicFunction::saveMap(string map, int width=0) {

    int height;
    if(width == 0) {
        width = noOfBins[0];
        height = noOfBins[1];
    } else {
        height = (int)((noOfBins[1]/noOfBins[0])*(double)width);
    }

    IplImage * img = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    cvResize(worldImage,img,CV_INTER_NN);
    if(!cvSaveImage(map.c_str(),img)) {
        cout << "Could not save world map image: " << map.c_str() << endl;
    }
    cvReleaseImage(&img);
}

void HarmonicFunction::setGoal(const Vector &goal) {   

    if(goal.size() != dims) return;
    CvScalar pot;
    pair<int,int> p = getMapCoordinate(goal);

    if(worldMap[p] != DIRICHLET) {
        worldMap[p] = GOAL;
        cvSet2D(worldImage,p.first,p.second,goalColor);
        potentialMap[p] = potential[GOAL];
        if(displayMode) {
            pot = CV_RGB(potentialMap[p]*255,potentialMap[p]*255,potentialMap[p]*255);
            cvSet2D(potentialImage,p.first,p.second,pot);
        }
    }
}


void HarmonicFunction::setObstacle(const Vector &pos, double rad) {
    // NOTE: NOT USING RADIUS OF OBSTACLE AT THIS POINT
    if(pos.size() != dims) return;      

    CvScalar pot;

    Vector v00(dims);
    Vector v01(dims);
    Vector v10(dims);
    Vector v11(dims);

    pair<int,int> p;
    pair<int,int> p00;
    pair<int,int> p01;
    pair<int,int> p10;
    pair<int,int> p11;

    // get bounding box for obstacle, being careful not to go past boundaries of map
    v00[0] = max(pos[0] - rad,rangeMin[0]);
    v00[1] = max(pos[1] - rad,rangeMin[1]);

    v01[0] = min(pos[0] + rad,rangeMax[0]);
    v01[1] = max(pos[1] - rad,rangeMin[1]);

    v10[0] = max(pos[0] - rad,rangeMin[0]);
    v10[1] = min(pos[1] + rad,rangeMax[1]);

    v11[0] = min(pos[0] + rad,rangeMax[0]);
    v11[1] = min(pos[1] + rad,rangeMax[1]);
    
    p = getMapCoordinate(pos);
    p00 = getMapCoordinate(v00);
    p01 = getMapCoordinate(v01);
    p10 = getMapCoordinate(v10);
    p11 = getMapCoordinate(v11);

    for(int i=p00.first; i<p11.first; i++) {
        for(int j=p00.second; j<p01.second; j++) {
            pair<int,int> idx(i,j);
            worldMap[idx] = OBSTACLE;
            cvSet2D(worldImage,i,j,obstacleColor);
            potentialMap[idx] = potential[OBSTACLE];
            if(displayMode) {
                pot = CV_RGB(0,0,255);
                cvSet2D(potentialImage,idx.first,idx.second,pot);
            }
        } 
    }
}
 
void HarmonicFunction::printWorldMap() {
    for(int i=0; i<noOfBins[0]; i++) {
        for(int j=0; j<noOfBins[1]; j++) {
            pair<int,int> p(i,j);
            cout << worldMap[p] << "  ";
        }
        cout << endl;
    }
    cout << endl;
}

void HarmonicFunction::printPotentialMap() {
    for(int i=0; i<noOfBins[0]; i++) {
        for(int j=0; j<noOfBins[1]; j++) {
            pair<int,int> p(i,j);
            cout << potentialMap[p] << "  ";
        }
        cout << endl;
    }
    cout << endl;
}
   
void HarmonicFunction::SOR() {
    int converged = false;
    iterationCount = 0;
    while(!converged && (iterationCount < maxIterations)) {
        if (sorOnce() < convergenceThreshold) converged = true;        
        if(iterationCount % 50 == 0) cout << "iteration: " << iterationCount << endl;
        iterationCount++;
    }
    cout << "SOR relaxtion complete..." << endl;
}

void HarmonicFunction::gaussSeidel() {
    int converged = false;
    iterationCount = 0;
    cout << "Performing Gauss-Seidel relaxation..." << endl;
    while(!converged && (iterationCount < maxIterations)) {
        if (gaussSeidelOnce() < convergenceThreshold) converged = true;
        iterationCount++;
        if(iterationCount % 50 == 0) cout << "iteration: " << iterationCount << endl;
    }
    cout << "Gauss-Seidel relaxtion complete..." << endl;
}

double HarmonicFunction::sorOnce() {
    int ipos, ineg, jpos, jneg;
    double residual, max, front, back, up, down;

    CvScalar pot;
    double x;
 
    max = 0.0;
    for(int i=1; i<noOfBins[0]-1; ++i) {
        ipos = i+1; ineg = i-1;
        for (int j=1;j<noOfBins[1]-1;++j) {
            jpos = j+1; jneg = j-1;
            pair<int,int> p(i,j);

            if (worldMap[p] == FREESPACE) {

                pair<int,int> p_up(i,jpos);
                pair<int,int> p_down(i,jneg);
                pair<int,int> p_front(ipos,j);
                pair<int,int> p_back(ineg,j);
                up = potentialMap[p_up];
                down = potentialMap[p_down];
                front = potentialMap[p_front];
                back = potentialMap[p_back];
                
                residual = front + back + up + down - 4.0*potentialMap[p];
                //potentialMap[p] = (1-omega)*potentialMap[p] + (omega * residual)/4.0;
                potentialMap[p] = potentialMap[p] + ((1+omega) * residual)/4.0;
                if(displayMode) {
                    //x = potentialMap[p];
                    x = ((pow(plotScaleFactor,potentialMap[p])-1.0))/(plotScaleFactor-1.0);
                    pot = CV_RGB(x*255,x*255,x*255);
                    cvSet2D(potentialImage,i,j,pot);
                }
                if (fabs(residual) > max) max = fabs(residual);
            }

        }
    }
    return(max);
} 


double HarmonicFunction::gaussSeidelOnce() {
    int ipos, ineg, jpos, jneg;
    double residual, max, front, back, up, down;

    CvScalar pot;
    double x;
    
    max = 0.0;
    for(int i=1; i<noOfBins[0]-1; ++i) {
        ipos = i+1; ineg = i-1;
        for (int j=1;j<noOfBins[1]-1;++j) {
            jpos = j+1; jneg = j-1;
            pair<int,int> p(i,j);
            //            cout << "i: " << i << ", j: " << j << endl;
            if (worldMap[p] == FREESPACE) {

                pair<int,int> p_up(i,jpos);
                pair<int,int> p_down(i,jneg);
                pair<int,int> p_front(ipos,j);
                pair<int,int> p_back(ineg,j);
                up = potentialMap[p_up];
                down = potentialMap[p_down];
                front = potentialMap[p_front];
                back = potentialMap[p_back];

                residual = front + back + up + down - 4.0*potentialMap[p];
                potentialMap[p] = (front + back + up + down)/4.0;	    

                if(displayMode) {
                    //x = potentialMap[p];
                    x = ((pow(plotScaleFactor,potentialMap[p])-1.0))/(plotScaleFactor-1.0);
                    pot = CV_RGB(x*255,x*255,x*255);
                    cvSet2D(potentialImage,i,j,pot);
                }

                if (fabs(residual) > max) max = fabs(residual);

            }
        }
    }
    return(max);
    
}

Vector HarmonicFunction::computeGradient(const Vector &pos) {
    
    int i0, i1, j0, j1;
    double del_i, del_j;
    Vector ret(2);

    i0 = (int) (pos[1] / resolution[1]); i1 = i0+1;
    j0 = (int) (pos[0] / resolution[0]); j1 = j0+1;
    
    del_i = (pos[1]-rangeMin[1]) / resolution[1] - i0;
    del_j = (pos[0]-rangeMin[0]) / resolution[0] - j0;
    
    //cout << endl << "i0 : " << i0 << "\t" << "j0 : " << j0 << endl;
    //cout << "i1 : " << i1 << "\t" << "j1 : " << j1 << endl;
    //cout << "del_i : " << del_i << "\t" << "del_j : " << del_j << endl;

    pair<int,int> p00(i0,j0);    
    pair<int,int> p01(i0,j1);    
    pair<int,int> p10(i1,j0);    
    pair<int,int> p11(i1,j1);    

    grad[1] = ((1.0-del_j)/resolution[0]) * (potentialMap[p10] - potentialMap[p00])
        + (del_j/resolution[1]) * (potentialMap[p11] - potentialMap[p01]);
    grad[0] = ((1.0-del_i)/resolution[1]) * (potentialMap[p01] - potentialMap[p00])
        + (del_i/resolution[0]) * (potentialMap[p11] - potentialMap[p10]);
    
    //    return(sqrt(pow(grad[0],2)+pow(grad[1],2)));
    ret = grad;
    return ret;
}

void HarmonicFunction::computePath(const Vector &pos, bool drawTrace=false) {

    bool atGoal = false;
    double goalThresh = 0.00001;
    int simulationIterations = 0;
    int maxSimulationIterations = 2000;
    CvScalar path, rbt;
    pair<int,int> p;

    Vector gr;
    Vector cur = pos;
    double scaleFactor = 100.0;

    bool pathFromGradient = false;
    
    map<pair<int,int>,double> potStore;
    double pot_max;
    pair<int,int> p_max;

    leaveRobotTrace = drawTrace;

    if(pathComputed) {    
        clearPath();
    }

    currentPosition = pos;
    p = getMapCoordinate(currentPosition);
    
    if(!leaveRobotTrace) {
        pathPoints.push_back(p);
    }

    if(displayMode) {       
        rbt = CV_RGB(0, 255, 0);
        //cvCircle(potentialImage,cvPoint(p.second,p.first),2,rbt);        
        cvSet2D(potentialImage,p.first,p.second,rbt);        
    }

    while(!atGoal && (simulationIterations<maxSimulationIterations)) {

        if(pathFromGradient) {       

            gr = computeGradient(cur)*scaleFactor;       
            cur  = cur + gr;
            p = getMapCoordinate(cur);

        } else {

            potStore.clear();
            pot_max = -1.0;
            for(int i=-1;i<=1;i++) {
                for(int j=-1;j<=1;j++) {                                   
                    pair<int,int> p_tmp(p.first+i,p.second+j);
                    potStore[p_tmp] = potentialMap[p_tmp];
                    if(potStore[p_tmp] >= pot_max) {
                        pot_max = potStore[p_tmp];
                        p_max = p_tmp;
                    }
                }
            }
            p = p_max;

        }

        pathPoints.push_back(p);
        //        cout << "iteration["<<simulationIterations<<"] -- at ("<<cur[0]<<","<<cur[1]<<")"<<endl;

        if(displayMode) {
            path = CV_RGB(255,0,0);
            cvSet2D(potentialImage,p.first,p.second,path);                        
        }
        

        if(potentialMap[p] > (1.0-goalThresh)) {
            //cout << "simulation reached goal..." << endl;
            atGoal = true;
        }

        simulationIterations++;
    }


    pathComputed = true;
}

void HarmonicFunction::clearPath() {

    double x;
    CvScalar pot;
    pair<int,int> p;

    if(displayMode) {
        for(unsigned int i=0; i<pathPoints.size(); i++) {
            p = pathPoints[i];
            x = ((pow(plotScaleFactor,potentialMap[p])-1.0))/(plotScaleFactor-1.0);
            pot = CV_RGB(x*255,x*255,x*255);
            cvSet2D(potentialImage,p.first,p.second,pot);
        }
    }

    pathPoints.clear();
    pathComputed = false;
}

