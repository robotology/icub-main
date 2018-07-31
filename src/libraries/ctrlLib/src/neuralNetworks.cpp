/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

#include <sstream>
#include <iomanip>
#include <cmath>

#include <yarp/math/Math.h>
#include <iCub/ctrl/neuralNetworks.h>

using namespace std;
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
ff2LayNN::ff2LayNN(const Property &options)
{
    configure(options);
}


/***************************************************************************/
bool ff2LayNN::isValid() const
{
    return configured;
}


/***************************************************************************/
void ff2LayNN::setItem(Property &options, const string &tag, const Vector &item) const
{
    Bottle b; Bottle &v=b.addList();
    for (size_t i=0; i<item.length(); i++)
        v.addDouble(item[i]);

    options.put(tag,b.get(0));
}


/***************************************************************************/
bool ff2LayNN::getItem(const Property &options, const string &tag, Vector &item) const
{
    if (Bottle *b=options.find(tag).asList())
    {
        item.resize(b->size());
        for (size_t i=0; i<item.length(); i++)
            item[i]=b->get(i).asDouble();

        return true;
    }
    else
        return false;
}


/***************************************************************************/
void ff2LayNN::prepare()
{
    inMinX.resize(inMinMaxX.size());
    inMinY.resize(inMinMaxX.size());
    inRatio.resize(inMinMaxX.size());

    for (size_t i=0; i<inMinX.length(); i++)
    {
        inMinX[i]=inMinMaxX[i].min;
        inMinY[i]=inMinMaxY[i].min;
        inRatio[i]=(inMinMaxY[i].max-inMinMaxY[i].min)/(inMinMaxX[i].max-inMinMaxX[i].min);
    }

    outMinX.resize(outMinMaxX.size());
    outMinY.resize(outMinMaxX.size());
    outRatio.resize(outMinMaxX.size());

    for (size_t i=0; i<outMinX.length(); i++)
    {
        outMinX[i]=outMinMaxX[i].min;
        outMinY[i]=outMinMaxY[i].min;
        outRatio[i]=(outMinMaxX[i].max-outMinMaxX[i].min)/(outMinMaxY[i].max-outMinMaxY[i].min);
    }
}


/***************************************************************************/
bool ff2LayNN::configure(const Property &options)
{
    IW.clear();
    LW.clear();

    inMinMaxX.clear();
    inMinMaxY.clear();
   
    outMinMaxX.clear();
    outMinMaxY.clear();

    configured=false;

    // acquire options
    if (!options.check("numInputNodes") || !options.check("numHiddenNodes") || 
        !options.check("numOutputNodes"))
        return false;

    int numHiddenNodes=options.find("numHiddenNodes").asInt();
    for (int i=0; i<numHiddenNodes; i++)
    {
        ostringstream tag;
        Vector item;

        tag<<"IW_"<<i;
        if (getItem(options,tag.str(),item))
            IW.push_back(item);
        else
            return false;
    }
    
    if (!getItem(options,"b1",b1))
        return false;

    int numOutputNodes=options.find("numOutputNodes").asInt();
    for (int i=0; i<numOutputNodes; i++)
    {
        ostringstream tag;
        Vector item;

        tag<<"LW_"<<i;
        if (getItem(options,tag.str(),item))
            LW.push_back(item);
        else
            return false;
    }

    if (!getItem(options,"b2",b2))
        return false;

    int numInputNodes=options.find("numInputNodes").asInt();
    for (int i=0; i<numInputNodes; i++)
    {
        ostringstream tagX, tagY;
        Vector itemX, itemY;

        tagX<<"inMinMaxX_"<<i;
        tagY<<"inMinMaxY_"<<i;
        if (!getItem(options,tagX.str(),itemX) || !getItem(options,tagY.str(),itemY))
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
        ostringstream tagX, tagY;
        Vector itemX, itemY;

        tagX<<"outMinMaxX_"<<i;
        tagY<<"outMinMaxY_"<<i;
        if (!getItem(options,tagX.str(),itemX) || !getItem(options,tagY.str(),itemY))
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
    prepare();

    return configured=true;
}


/***************************************************************************/
Vector ff2LayNN::scaleInputToNetFormat(const Vector &x) const
{
    return (inRatio*(x-inMinX)+inMinY);
}


/***************************************************************************/
Vector ff2LayNN::scaleInputFromNetFormat(const Vector &x) const
{
    return ((x-inMinY)/inRatio+inMinX);
}


/***************************************************************************/
Vector ff2LayNN::scaleOutputToNetFormat(const Vector &x) const
{
    return ((x-outMinX)/outRatio+outMinY);
}


/***************************************************************************/
Vector ff2LayNN::scaleOutputFromNetFormat(const Vector &x) const
{
    return (outRatio*(x-outMinY)+outMinX);
}


/***************************************************************************/
Vector ff2LayNN::predict(const Vector &x) const
{
    if (configured)
    {
        // input preprocessing
        Vector x1=scaleInputToNetFormat(x);

        // compute the output a1 of hidden layer
        Vector n1(IW.size());
        for (size_t i=0; i<n1.length(); i++)
            n1[i]=yarp::math::dot(IW[i],x1)+b1[i];
        Vector a1=hiddenLayerFcn(n1);

        // compute the output a2 of the network
        Vector n2(LW.size());
        for (size_t i=0; i<n2.length(); i++)
            n2[i]=yarp::math::dot(LW[i],a1)+b2[i];
        Vector a2=outputLayerFcn(n2);
    
        // output postprocessing
        return scaleOutputFromNetFormat(a2);
    }
    else
        return Vector(1);
}


/***************************************************************************/
bool ff2LayNN::getStructure(Property &options) const
{
    options.clear();

    options.put("numHiddenNodes",(int)IW.size());
    options.put("numOutputNodes",(int)LW.size());
    options.put("numInputNodes",(int)inMinMaxX.size());

    for (size_t i=0; i<IW.size(); i++)
    {
        ostringstream tag;
        tag<<"IW_"<<i;

        setItem(options,tag.str(),IW[i]);
    }

    setItem(options,"b1",b1);

    for (size_t i=0; i<LW.size(); i++)
    {
        ostringstream tag;
        tag<<"LW_"<<i;

        setItem(options,tag.str(),LW[i]);
    }

    setItem(options,"b2",b2);

    for (size_t i=0; i<inMinMaxX.size(); i++)
    {
        ostringstream tagX, tagY;
        tagX<<"inMinMaxX_"<<i;
        tagY<<"inMinMaxY_"<<i;

        Vector X(2);
        X[0]=inMinMaxX[i].min;
        X[1]=inMinMaxX[i].max;

        Vector Y(2);
        Y[0]=inMinMaxY[i].min;
        Y[1]=inMinMaxY[i].max;

        setItem(options,tagX.str(),X);
        setItem(options,tagY.str(),Y);
    }

    for (size_t i=0; i<outMinMaxX.size(); i++)
    {
        ostringstream tagX, tagY;
        tagX<<"outMinMaxX_"<<i;
        tagY<<"outMinMaxY_"<<i;

        Vector X(2);
        X[0]=outMinMaxX[i].min;
        X[1]=outMinMaxX[i].max;

        Vector Y(2);
        Y[0]=outMinMaxY[i].min;
        Y[1]=outMinMaxY[i].max;

        setItem(options,tagX.str(),X);
        setItem(options,tagY.str(),Y);
    }

    return true;
}


/***************************************************************************/
bool ff2LayNN::printStructure(ostream &stream) const
{
    stream<<"***** Input Layer Range *****"<<endl;
    for (size_t i=0; i<inMinMaxX.size(); i++)
        stream<<i<<": X ["<<inMinMaxX[i].min<<" "<<inMinMaxX[i].max
              <<"]; Y ["<<inMinMaxY[i].min<<" "<<inMinMaxY[i].max<<"]"<<endl;

    stream<<"***** Hidden Layer Weights *****"<<endl;
    for (size_t i=0; i<IW.size(); i++)
        stream<<"IW_"<<i<<": ["<<IW[i].toString(16,1)<<"]"<<endl;

    stream<<"***** Hidden Layer Bias *****"<<endl;
    stream<<"b1: ["<<b1.toString(16,1)<<"]"<<endl;

    stream<<"***** Output Layer Weights *****"<<endl;
    for (size_t i=0; i<LW.size(); i++)
        stream<<"LW_"<<i<<": ["<<LW[i].toString(16,1)<<"]"<<endl;

    stream<<"***** Output Layer Bias *****"<<endl;
    stream<<"b2: ["<<b2.toString(16,1)<<"]"<<endl;

    stream<<"***** Output Layer Range *****"<<endl;
    for (size_t i=0; i<outMinMaxX.size(); i++)
        stream<<i<<": Y ["<<outMinMaxY[i].min<<" "<<outMinMaxY[i].max
              <<"]; X ["<<outMinMaxX[i].min<<" "<<outMinMaxX[i].max<<"]"<<endl;

    return stream.good();
}


/***************************************************************************/
ff2LayNN_tansig_purelin::ff2LayNN_tansig_purelin() :
                         ff2LayNN()
{
}


/***************************************************************************/
ff2LayNN_tansig_purelin::ff2LayNN_tansig_purelin(const Property &options) :
                         ff2LayNN(options)
{
}


/***************************************************************************/
Vector ff2LayNN_tansig_purelin::hiddenLayerFcn(const Vector &x) const
{
    Vector y(x.length());
    for (size_t i=0; i<x.length(); i++)
        y[i]=2.0/(1.0+exp(-2.0*x[i]))-1.0;

    return y;
}


/***************************************************************************/
Vector ff2LayNN_tansig_purelin::outputLayerFcn(const Vector &x) const
{
    return x;
}


/***************************************************************************/
Vector ff2LayNN_tansig_purelin::hiddenLayerGrad(const Vector &x) const
{
    Vector y(x.length());
    for (size_t i=0; i<x.length(); i++)
    {
        double tmp1=exp(-2.0*x[i]);
        double tmp2=1.0+tmp1;
        y[i]=(4.0*tmp1)/(tmp2*tmp2);
    }

    return y;
}


/***************************************************************************/
Vector ff2LayNN_tansig_purelin::outputLayerGrad(const Vector &x) const
{
    return Vector(x.length(),1.0);
}


