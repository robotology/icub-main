// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

////////////////////////////////////

#include "iCubBoard.h"

////////////////////////////////////

bool iCubBLLBoard::findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)
{
    int index=addr.find(",");
    std::string sID=index<0?addr:addr.substr(0,index);

    if (sID.length()==0) return false; //should never happen

    if (ID!=mID!=atoi(sID.c_str())) return false;

    ++index;

    addr=addr.substr(index,addr.length()-index);

    for (int i=0; i<2; ++i)
    {
        if (mChannel[i]->findAndWrite(addr,dataDouble,dataBool,dataInt))
        {
            return true;
        }
    }

    return false;
}

