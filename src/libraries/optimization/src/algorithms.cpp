/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/optimization/algorithms.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::optimization;


/****************************************************************/
bool iCub::optimization::minVolumeEllipsoid(const deque<Vector> &points,
                                            const double tol,
                                            Matrix &A, Vector &c)
{
    // This code is a C++ version of the MATLAB script written by:
    // Nima Moshtagh (nima@seas.upenn.edu), University of Pennsylvania
    //
    // It makes use of the Khachiyan algorithm and aims at minimizing
    // iteratively the following problem:
    //
    // min log(det(A))
    // s.t. (points[i]-c)'*A*(points[i]-c)<=1

    if (points.empty())
        return false;

    // initialization
    int d=(int)points.front().length();
    int N=(int)points.size();
    Vector u(N,1.0/N);
    Matrix U(N,N); U.diagonal(u);
    Matrix Q(d+1,N);
    for (int row=0; row<Q.rows(); row++)
        for (int col=0; col<Q.cols(); col++)
            Q(row,col)=points[col][row];
    for (int col=0; col<Q.cols(); col++)
        Q(Q.rows()-1,col)=1.0;
    Matrix Qt=Q.transposed();

    // run the Khachiyan algorithm
    Matrix M;
    while (true)
    {
        M=Qt*pinv(Q*U*Qt)*Q;
        int j=0; double max=M(j,j);
        for (int row=1; row<M.rows(); row++)
        {
            if (M(row,row)>max)
            {
                max=M(row,row);
                j=row;
            }
        }
        double step_size=(max-d-1.0)/((d+1.0)*(max-1.0));
        Vector new_u=(1.0-step_size)*u;
        new_u[j]+=step_size;
        if (norm(new_u-u)<tol)
            break;
        u=new_u;
        U.diagonal(u);
    }

    // compute the ellipsoid parameters
    Matrix P=Q.removeRows(Q.rows()-1,1);
    c=P*u;
    Matrix C(c.length(),c.length());
    for (int col=0; col<C.cols(); col++)
        C.setCol(col,c[col]*c);
    A=(1.0/d)*pinv(P*U*P.transposed()-C);

    return true;
}


