// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "CartesianPositionHarmonicFunction.h"
#include <yarp/math/Math.h>
#include <math.h>
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

void CB::CartesianPositionHarmonicFunction::startPotentialFunction() {
    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "Couldn't connect to YARP input port in startPotentialFunction()..." << endl;
            return;
        }
    }
    start();     // mandatory start function
}

void CB::CartesianPositionHarmonicFunction::stopPotentialFunction() {
    stop();     // mandatory stop function
}

bool CB::CartesianPositionHarmonicFunction::updatePotentialFunction() {

    bool ok = true;
    Bottle *b[2];
    int offset;
    string t;
    double rad;
    Vector obs(3);
    Vector goal(3);
    
    // first read port 1 to see if there are any obstacles or goals being added
    if(inputPorts.size() != 2) {
        cout << "CartesianPositionHarmonicFunction::update() -- wrong number of input ports!!" << endl;
        return false;
    }
    b[1] = inputPorts[1]->read(true);

    if(b[1]!=NULL) {        
        t = b[1]->get(0).asString();

        if(t=="obstacle") {
            
            obs[0] = b[1]->get(1).asDouble();
            obs[1] = b[1]->get(2).asDouble();
            obs[2] = b[1]->get(3).asDouble();
            rad = b[1]->get(4).asDouble();
            setObstacle(obs,rad);
            
            cout << "CartesianPositionHarmonicFunction::update() -- got new obstacle with radius " <<
                rad << " at:" << endl << obs[0] << endl << obs[1] << endl << obs[2] << endl;
        
        } else if(t=="goal") {
            
            goal[0] = b[1]->get(1).asDouble();
            goal[1] = b[1]->get(2).asDouble();
            goal[2] = b[1]->get(3).asDouble();
            setGoal(goal);
            
            cout << "CartesianPositionHarmonicFunction::update() -- got new goal at: " <<
                endl << goal[0] << endl << goal[1] << endl << goal[2] << endl;
        }
        
    }
    
    // read the position
    b[0] = inputPorts[0]->read(true);
    if(b[0]==NULL) {
        // non fatal error (probably due to asynchronous update rates)
        //    cout << "CartesianPositionHarmonicFunction::update() -- could not read input data!" << endl;
        return ok;
    }
    
    offset = 1;
    for(int i=0; i<size; i++) {
        (*inputs[0])[i] = b[0]->get(i+offset).asDouble();
    }
    
    // find out potential here...
    curPos[0] = (*inputs[0])[0];
    curPos[1] = (*inputs[0])[1];
    curPos[2] = (*inputs[0])[2];
    
    int x = (int)(((*inputs[0])[0] - rangeMin[0]) / resolution[0]); 
    int y = (int)(((*inputs[0])[1] - rangeMin[1]) / resolution[1]); 
    int z = (int)(((*inputs[0])[2] - rangeMin[2]) / resolution[2]); 
    potential = potentialMap[x][y][z];
    
    gradient = computeGradient(curPos);
    return ok;
}


bool CB::CartesianPositionHarmonicFunction::connectToInputs() {
    
    bool ok = true;
    
    cout << "CartesianHarmonicFunction::connectToInputs():\n\t" << inputNames[0].c_str() << endl << endl;

    string posCurName = inputNames[0] + "/data:o";
    string prefixStr = "/cb/" + getSpace();
    int s = prefixStr.size();
    string tmp0 = inputNames[0];
    tmp0.erase(0,s);

    string posCurNameIn = "/cb/cartesianposition/harmonic_pf" + tmp0 + "/data:i";   

    cout << "CartesianPositionHarmonicFunction::connectToInputs() -- opening current input port..." << endl;
    ok &= inputPorts[0]->open(posCurNameIn.c_str());
    if(!ok) {
      cout << "CartesianHarmonicFunction::connectToInputs() -- failed opening current input port..." << endl;
      return ok;
    }

    ok &= Network::connect(posCurName.c_str(),posCurNameIn.c_str(),"udp");
    if(!ok) {
        cout << "CartesianHarmonicFunction::connectToInputs() -- failed connecting to current input pors...\n\n\n" << endl;
        return ok;
    }

    cout << "CartesianPositionHarmonicFunction done connecting to YARP input ports (size=" << gradient.size() << ")..." << endl;

    connectedToInputs = true;
    return ok; 
}



bool CB::CartesianPositionHarmonicFunction::initHarmonicFunction()  {

    threshold = 0.0001;
    omega = 0.9;

    rangeMin.resize(3);
    rangeMax.resize(3);
    resolution.resize(3);
    noOfBins.resize(3);
    potentialVals.resize(4);
    potentialVals[FREESPACE] = FREESPACE_POTENTIAL;
    potentialVals[OBSTACLE] = OBSTACLE_POTENTIAL;
    potentialVals[DIRICHLET] = DIRICHLET_POTENTIAL;
    potentialVals[GOAL] = GOAL_POTENTIAL;
    
    rangeMin[0] = -1.0;
    rangeMin[1] = -1.0;
    rangeMin[2] = -1.0;
    
    rangeMax[0] = 1.0;
    rangeMax[1] = 1.0;
    rangeMax[2] = 1.0;
    
    resolution[0] = 0.05;
    resolution[1] = 0.05;
    resolution[2] = 0.05;
    
    for (int i = 0; i < 3; i++) {
        noOfBins[i] = (int)((rangeMax[i] - rangeMin[i]) / resolution[i]) + 1;
        cout << "CartesianPositionHarmonicFunction::initHF() -- noOfBins[" << i << "]=" << noOfBins[i] << endl;
    }
    
    initializeMap();
    gaussSeidel();
    
    Vector goal(3);
    
    // initial random goal pos  
    goal[0] = -0.3;
    goal[1] =  0.0;
    goal[2] =  0.2;
    
    setGoal(goal);
    
    cout << "Cartesian Harmonic Function Initiallized" << endl;
    return true;
    
}

//Function to Initialize the world and potential maps
//and set the boundary dirichlet conditions
void CB::CartesianPositionHarmonicFunction::initializeMap() {

    //Initialize the matrices
    worldMap = (int ***)malloc(noOfBins[0] * sizeof(int **));
    potentialMap = (double ***)malloc(noOfBins[0] * sizeof(double **));
    
    for(int i=0; i<noOfBins[0]; i++) {
        worldMap[i] = (int **)malloc(noOfBins[1] * sizeof(int *));
        potentialMap[i] = (double **)malloc(noOfBins[1] * sizeof(double *));
    }
    
    for(int i=0; i<noOfBins[0]; i++) {
        for(int j=0; j<noOfBins[1]; j++) {
            worldMap[i][j] = (int *)malloc(noOfBins[2] * sizeof(int));
            potentialMap[i][j] = (double *)malloc(noOfBins[2] * sizeof(double));
        }
    }
    
    //Set the dirichlet boundary conditons                                                                                                                                        
    for(int i=0; i<noOfBins[0]; i++)
        for(int j=0; j<noOfBins[1]; j++)
            for(int k=0; k<noOfBins[2]; k++) {
                if(i == 0 || j == 0 || k == 0 || i == noOfBins[0]-1 || j == noOfBins[1]-1 || k == noOfBins[2]-1)
                    worldMap[i][j][k] = DIRICHLET;
                else
                    worldMap[i][j][k] = FREESPACE;
                potentialMap[i][j][k] = potentialVals[worldMap[i][j][k]];
            }
    
    cout << "Cartesian Harmonic Function map initialized..." << endl;
}


//Function to set the goal in the potential map
void CB::CartesianPositionHarmonicFunction::setGoal(const Vector &goal) {

    int i, j, k;
  
    i = (int)((goal[0] - rangeMin[0]) / resolution[0]);
    j = (int)((goal[1] - rangeMin[1]) / resolution[1]);
    k = (int)((goal[2] - rangeMin[2]) / resolution[2]);
    
    if (worldMap[i][j][k] != DIRICHLET) {
        worldMap[i][j][k] = GOAL;
        potentialMap[i][j][k] = potentialVals[GOAL];
    }
    
    cout << "setting goal at: (" << goal[0] << ", " << goal[1] << ", " << goal[2] << ")" << endl;
    
    // compute HF
    gaussSeidel();
}

//Function to set the obstacles in the potential map - 
void CB::CartesianPositionHarmonicFunction::setObstacle(const Vector &pos, double rad)  {

    int i, j, k;
    
    // centroid
    i = (int)((pos[0] - rangeMin[0]) / resolution[0]);
    j = (int)((pos[1] - rangeMin[1]) / resolution[1]);
    k = (int)((pos[2] - rangeMin[2]) / resolution[2]);
    
    worldMap[i][j][k] = OBSTACLE;
    potentialMap[i][j][k] = potentialVals[OBSTACLE];
    
    // x +/- rad
    i = (int)(((pos[0] + rad) - rangeMin[0]) / resolution[0]);
    j = (int)((pos[1] - rangeMin[1]) / resolution[1]);
    k = (int)((pos[2] - rangeMin[2]) / resolution[2]);
    
    worldMap[i][j][k] = OBSTACLE;
    potentialMap[i][j][k] = potentialVals[OBSTACLE];
    
    i = (int)(((pos[0] - rad) - rangeMin[0]) / resolution[0]);
    j = (int)((pos[1] - rangeMin[1]) / resolution[1]);
    k = (int)((pos[2] - rangeMin[2]) / resolution[2]);
    
    worldMap[i][j][k] = OBSTACLE;
    potentialMap[i][j][k] = potentialVals[OBSTACLE];
    
    // y +/- rad
    i = (int)((pos[0] - rangeMin[0]) / resolution[0]);
    j = (int)(((pos[1] + rad) - rangeMin[1]) / resolution[1]);
    k = (int)((pos[2] - rangeMin[2]) / resolution[2]);
    
    worldMap[i][j][k] = OBSTACLE;
    potentialMap[i][j][k] = potentialVals[OBSTACLE];
    
    i = (int)((pos[0] - rangeMin[0]) / resolution[0]);
    j = (int)(((pos[1] - rad) - rangeMin[1]) / resolution[1]);
    k = (int)((pos[2] - rangeMin[2]) / resolution[2]);
    
    worldMap[i][j][k] = OBSTACLE;
    potentialMap[i][j][k] = potentialVals[OBSTACLE];
    
    // z +/- rad
    i = (int)((pos[0] - rangeMin[0]) / resolution[0]);
    j = (int)((pos[1] - rangeMin[1]) / resolution[1]);
    k = (int)(((pos[2] + rad) - rangeMin[2]) / resolution[2]);
    
    worldMap[i][j][k] = OBSTACLE;
    potentialMap[i][j][k] = potentialVals[OBSTACLE];
    
    i = (int)((pos[0] - rangeMin[0]) / resolution[0]);
    j = (int)((pos[1] - rangeMin[1]) / resolution[1]);
    k = (int)(((pos[2] - rad) - rangeMin[2]) / resolution[2]);
    
    worldMap[i][j][k] = OBSTACLE;
    potentialMap[i][j][k] = potentialVals[OBSTACLE];
    
    // compute HF
    gaussSeidel();
    
}

//Function to iteratively call the Successive Over Relaxation(SOR) until the
//difference is less than the THRESHOLD
void CB::CartesianPositionHarmonicFunction::SOR()  {
    bool converged = false;
    iterationCount = 0;
    
    while (!converged && (iterationCount < MAX_ITERATION))
        {
            if (sorOnce() < threshold) converged = true;
            iterationCount++;
        }
}

//Function to iteratively call the Gauss-Seidel until the
//difference is less than the THRESHOLD
void CB::CartesianPositionHarmonicFunction::gaussSeidel() {
    bool converged = false;
    iterationCount = 0;
    
    cout << "Performing Gauss Seidel Relaxation..." << endl;
    while (!converged && (iterationCount < MAX_ITERATION))
        {
            if (gaussSeidelOnce() < threshold) converged = true;
            iterationCount++;
        }
    cout << "Finished Relaxation after " << iterationCount << " iterations" << endl;
}

//Function implements one iteration of SOR for 3 dimensions.
//The harmonic function is computed only at those bins which have been designated as FREESPACE
//It returns the maximum change in value after each iteration
double CB::CartesianPositionHarmonicFunction::sorOnce() {
    int i, j, k;
    int ipos, ineg, jpos, jneg, kpos, kneg;
    double residual, max;
    double front, back, up, down, right, left;
    
    max = 0.0;
    for (i = 1; i < (noOfBins[0] - 1); i++)
        {
            ipos = i + 1; ineg = i - 1;
            for (j = 1; j < (noOfBins[1] - 1); j++)
                {
                    jpos = j + 1; jneg = j - 1;
                    for (k = 1; k < (noOfBins[2] - 1); k++)
                        {
                            kpos = k + 1; kneg = k - 1;
                            
                            if (worldMap[i][j][k] == FREESPACE)
                                {
                                    
                                    up = potentialMap[i][jpos][k];
                                    down = potentialMap[i][jneg][k];
                                    front = potentialMap[ipos][j][k];
                                    back = potentialMap[ineg][j][k];
                                    right = potentialMap[i][j][kpos];
                                    left = potentialMap[i][j][kneg];
                                    
                                    residual = front + back + up + down + right + left - 6.0 * potentialMap[i][j][k];
                                    potentialMap[i][j][k] = (1 - omega) * potentialMap[i][j][k] - (omega * residual) / 6.0;
                                    
                                    if (fabs(residual) > max) max = fabs(residual);
                                    
                                }
                        }
                }
        }
    
    // cout << "MAX : %lf\n",max);
    return max;
}

//Function implements one iteration of Gauss-Seidel for 3 dimensions.
//The harmonic function is computed only at those bins which have been designated as FREESPACE
//It returns the maximum change in value after each iteration
double CB::CartesianPositionHarmonicFunction::gaussSeidelOnce() {
    int i, j, k;
    int ipos, ineg, jpos, jneg, kpos, kneg;
    double residual, max;
    double front, back, up, down, right, left;
    
    max = 0.0;
    for (i = 1; i < (noOfBins[0] - 1); i++)
        {
            ipos = i + 1; ineg = i - 1;
            for (j = 1; j < (noOfBins[1] - 1); j++)
                {
                    jpos = j + 1; jneg = j - 1;
                    for (k = 1; k < (noOfBins[2] - 1); k++)
                        {
                            kpos = k + 1; kneg = k - 1;
                            
                            if (worldMap[i][j][k] == FREESPACE)
                                {
                                    
                                    up = potentialMap[i][jpos][k];
                                    down = potentialMap[i][jneg][k];
                                    front = potentialMap[ipos][j][k];
                                    back = potentialMap[ineg][j][k];
                                    right = potentialMap[i][j][kpos];
                                    left = potentialMap[i][j][kneg];
                                    
                                    residual = front + back + up + down + right + left - 6.0 * potentialMap[i][j][k];
                                    potentialMap[i][j][k] = (front + back + up + down + right + left) / 6.0;
                                    
                                    if (fabs(residual) > max) max = fabs(residual);
                                    
                                }
                        }
                }
        }
    
    // cout << "MAX : %lf\n",max);
    return (max);
}

//Function to compute the gradient to be followed in 3D space
Vector CB::CartesianPositionHarmonicFunction::computeGradient(const Vector &pos)  {

    int i0, i1, j0, j1, k0, k1;
    double del_i, del_j, del_k, mag;
    double phi_000, phi_001, phi_010, phi_011;
    double phi_100, phi_101, phi_110, phi_111;
    
    Vector grad(3);

    //  cout << "computing HF gradient...\n" << endl;
    
    i0 = (int)((pos[0] - rangeMin[0]) / resolution[0]); i1 = i0 + 1;
    j0 = (int)((pos[1] - rangeMin[1]) / resolution[1]); j1 = j0 + 1;
    k0 = (int)((pos[2] - rangeMin[2]) / resolution[2]); k1 = k0 + 1;
    
    del_i = (pos[0] - rangeMin[0]) / resolution[0] - i0;
    del_j = (pos[1] - rangeMin[1]) / resolution[1] - j0;
    del_k = (pos[2] - rangeMin[2]) / resolution[2] - k0;
    
    phi_000 = potentialMap[i0][j0][k0];
    phi_001 = potentialMap[i0][j0][k1];
    phi_010 = potentialMap[i0][j1][k0];
    phi_011 = potentialMap[i0][j1][k1];
    phi_100 = potentialMap[i1][j0][k0];
    phi_101 = potentialMap[i1][j0][k1];
    phi_110 = potentialMap[i1][j1][k0];
    phi_111 = potentialMap[i1][j1][k1];
    
    // d(phi)/dx = d(phi)/d(delta) * d(delta)/dx     d(delta)/dx = 1.0/DELTA
    grad[0] = (1.0 - del_j) * del_k * (phi_101 - phi_001) + del_j * (1.0 - del_k) * (phi_110 - phi_010)
        + del_j * del_k * (phi_111 - phi_011) + (1.0 - del_j) * (1.0 - del_k) * (phi_100 - phi_000);
    
    // d(phi)/dy = d(phi)/d(delta) * d(delta)/dy     d(delta)/dy = -(1.0/DELTA)
    grad[1] = (1.0 - del_i) * del_k * (phi_011 - phi_001) + del_i * (1.0 - del_k) * (phi_110 - phi_100)
        + del_i * del_k * (phi_111 - phi_101) + (1.0 - del_i) * (1.0 - del_k) * (phi_010 - phi_000);
    
    grad[2] = (1.0 - del_i) * del_j * (phi_011 - phi_010) + del_i * (1.0 - del_j) * (phi_101 - phi_100)
        + del_i * del_j * (phi_111 - phi_110) + (1.0 - del_i) * (1.0 - del_j) * (phi_001 - phi_000);
    
    return grad*-1.0;
    
}


