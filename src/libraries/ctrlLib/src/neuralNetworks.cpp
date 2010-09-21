/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
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
#include <iCub/ctrl/neuralNetworks.h>

#include <stdio.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/***************************************************************************/
ff2LayNN::ff2LayNN()
{
    configured=false;
}


/***************************************************************************/
ff2LayNN::ff2LayNN(Property &options)
{
    configure(options);
}


/***************************************************************************/
bool ff2LayNN::getItem(Property &options, const char *tag, Vector &item)
{
    Bottle *b=options.find(tag).asList();

    if (b!=NULL)
    {
        item.resize(b->size());

        for (int i=0; i<item.length(); i++)
            item[i]=b->get(i).asDouble();

        return true;
    }
    else
        return false;
}


/***************************************************************************/
bool ff2LayNN::configure(Property &options)
{
    IW.clear();
    LW.clear();

    inMinMaxX.clear();
    inMinMaxY.clear();
   
    outMinMaxX.clear();
    outMinMaxY.clear();

    configured=false;

    // acquire options
    if (!options.check("numInput")       ||
        !options.check("numHiddenNodes") || 
        !options.check("numOutputNodes"))
        return false;

    int numHiddenNodes=options.find("numHiddenNodes").asInt();    

    for (int i=0; i<numHiddenNodes; i++)
    {
        char tag[255];
        Vector item;

        sprintf(tag,"IW_%d",i);

        if (getItem(options,tag,item))
            IW.push_back(item);
        else
            return false;
    }
    
    if (!getItem(options,"b1",b1))
        return false;

    int numOutputNodes=options.find("numOutputNodes").asInt();    

    for (int i=0; i<numOutputNodes; i++)
    {
        char tag[255];
        Vector item;

        sprintf(tag,"LW_%d",i);

        if (getItem(options,tag,item))
            LW.push_back(item);
        else
            return false;
    }

    if (!getItem(options,"b2",b2))
        return false;

    int numInput=options.find("numInput").asInt();    

    for (int i=0; i<numInput; i++)
    {
        char tagX[255], tagY[255];
        Vector itemX, itemY;

        sprintf(tagX,"inMinMaxX_%d",i);
        sprintf(tagY,"inMinMaxY_%d",i);

        if (!getItem(options,tagX,itemX) || !getItem(options,tagY,itemY))
            return false;
        else
        {
            minmax X, Y;

            X.min=itemX[0];
            X.max=itemX[1];
            Y.min=itemY[0];
            Y.max=itemY[1];

            inMinMaxX.push_back(X);
            inMinMaxY.push_back(Y);
        }
    }

    for (int i=0; i<numOutputNodes; i++)
    {
        char tagX[255], tagY[255];
        Vector itemX, itemY;

        sprintf(tagX,"outMinMaxX_%d",i);
        sprintf(tagY,"outMinMaxY_%d",i);

        if (!getItem(options,tagX,itemX) || !getItem(options,tagY,itemY))
            return false;
        else
        {
            minmax X, Y;

            X.min=itemX[0];
            X.max=itemX[1];
            Y.min=itemY[0];
            Y.max=itemY[1];

            outMinMaxX.push_back(X);
            outMinMaxY.push_back(Y);
        }
    }

    // prepare some internal variables
    inMinX.resize(inMinMaxX.size());
    inMinY.resize(inMinMaxX.size());
    inRatio.resize(inMinMaxX.size());

    for (int i=0; i<inMinX.length(); i++)
    {
        inMinX[i]=inMinMaxX[i].min;
        inMinY[i]=inMinMaxY[i].min;
        inRatio[i]=(inMinMaxY[i].max-inMinMaxY[i].min)/(inMinMaxX[i].max-inMinMaxX[i].min);
    }

    outMinX.resize(outMinMaxX.size());
    outMinY.resize(outMinMaxX.size());
    outRatio.resize(outMinMaxX.size());

    for (int i=0; i<outMinX.length(); i++)
    {
        outMinX[i]=outMinMaxX[i].min;
        outMinY[i]=outMinMaxY[i].min;
        outRatio[i]=(outMinMaxX[i].max-outMinMaxX[i].min)/(outMinMaxY[i].max-outMinMaxY[i].min);
    }

    return configured=true;
}


/***************************************************************************/
Vector ff2LayNN::predict(const Vector &x)
{
    if (configured)
    {
        // input preprocessing
        Vector x1=inRatio*(x-inMinX)+inMinY;
    
        // compute the output a1 of hidden layer
        Vector n1(IW.size());
        for (int i=0; i<n1.length(); i++)
            n1[i]=yarp::math::dot(IW[i],x1)+b1[i];
    
        Vector a1=hiddenLayerFcn(n1);
    
        // compute the output a2 of the network
        Vector n2(LW.size());
        for (int i=0; i<n2.length(); i++)
            n2[i]=yarp::math::dot(LW[i],a1)+b2[i];
    
        Vector a2=outputLayerFcn(n2);
    
        // output postprocessing
        return outRatio*(a2-outMinY)+outMinX;
    }
    else
        return Vector(1);
}


/***************************************************************************/
ff2LayNN_tansig_purelin::ff2LayNN_tansig_purelin() :
                         ff2LayNN()
{
}


/***************************************************************************/
ff2LayNN_tansig_purelin::ff2LayNN_tansig_purelin(Property &options) :
                         ff2LayNN(options)
{
}


/***************************************************************************/
Vector ff2LayNN_tansig_purelin::hiddenLayerFcn(const Vector &x)
{
    Vector y(x.length());

    for (int i=0; i<x.length(); i++)
        y[i]=2.0/(1.0+exp(-2.0*x[i]))-1.0;

    return y;
}


/***************************************************************************/
Vector ff2LayNN_tansig_purelin::outputLayerFcn(const Vector &x)
{
    return x;
}



