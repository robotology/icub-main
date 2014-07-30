// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <stdio.h>
#include <Debug.h>
#include <yarp/os/Bottle.h>
#include "SkinConfigReader.h"

using namespace yarp::os;

SkinConfigReader::SkinConfigReader()
{;}

SkinConfigReader::SkinConfigReader(char *name)
{
    if (NULL != name)
    {
        strncpy(_name, name, sizeof(_name));
    }
}
//bool SkinConfigReader::readPatchesList(yarp::os::Searchable& config, SkinConfig *skCfg)
//{
//    if(NULL == skCfg)
//        return false;
//
//    Bottle bPatches, bPatchList, xtmp;
//    //reset total num of cards
//    skCfg->totalCardsNum = 0;
//
//
//    bPatches = config.findGroup("patches", "skin patches connected to this device");
//    if(bPatches.isNull())
//    {
//        yError() << "skin " << _name << "patches group is missed!";
//        return(false);
//    }
//
//    bPatchList = bPatches.findGroup("patchesIdList");
//    if(bPatchList.isNull())
//    {
//       yError() << "skin " << _name << "patchesList is missed!";
//       return(false);
//    }
//
//    skCfg->numOfPatches = bPatchList.size()-1;
//
//    skCfg->patchInfoList->clear();
//    skCfg->patchInfoList->resize(skCfg->numOfPatches);
//
//    for(int j=1; j<skCfg->numOfPatches+1; j++)
//    {
//       int id = bPatchList.get(j).asInt();
//#warning VALE aggiungi questo controllo in embObjskin
////       if((id!=1) && (id!=2))
////       {
////           yError() << "skin board num " << _fId.boardNum << "ems expected only patch num 1 or 2";
////           return false;
////       }
//       skCfg->patchInfoList[j-1].idPatch = id;
//#warning VALE metti in embObjSkin la conversione con indexNV
//       //skCfg->patchInfoList[j-1].indexNv = convertIdPatch2IndexNv(id);
//    }
//
//   //    //VALE solo per debug
//   //    yError() << "numOfPatches=" << numOfPatches;
//   //    for(int j=0; j<numOfPatches; j++ )
//   //    {
//   //        yError() << " patchInfoList[" << j << "]: patch=" << patchInfoList[j].idPatch << "indexNv=" << patchInfoList[j].indexNv;
//   //    }
//
//
//   for(int i=0; i<skCfg->numOfPatches; i++)
//   {
//       char tmp[80];
//       int id = skCfg->patchInfoList[i].idPatch;
//       snprintf(tmp, sizeof(tmp), "skinCanAddrsPatch%d", id);
//
//       xtmp = bPatches.findGroup(tmp);
//       if(xtmp.isNull())
//       {
//           yError() << "skin " << _name << "doesn't find " << tmp << "in xml file";
//           return false;
//       }
//
//       skCfg->patchInfoList[i].cardAddrList.resize(xtmp.size()-1);
//
//       for(int j=1; j<xtmp.size(); j++)
//       {
//           int addr = xtmp.get(j).asInt();
//           skCfg->totalCardsNum++;
//           skCfg->patchInfoList[i].cardAddrList[j-1] = addr;
//       }
//   }
//
//   //    //VALE solo per debug
//   //    yError() << "totalCardsNum=" << totalCardsNum;
//   //    for(int i=0; i<patchInfoList.size(); i++)
//   //    {
//   //        for(int j=0; j<patchInfoList[i].cardAddrList.size(); j++)
//   //        {
//   //            yError() << " elem num " << j << "of patch " <<patchInfoList[i].idPatch << "is " << patchInfoList[i].cardAddrList[j];
//   //        }
//   //    }
//
//   return(true);
//}
bool SkinConfigReader::isDefaultBoardCfgPresent(yarp::os::Searchable& config)
{
    Bottle boardCfgDefGroup =config.findGroup("defaultCfgBoard", "Default configuration for skin boards");
    if (boardCfgDefGroup.isNull()==true)
        return false;
    else
        return true;
}


bool SkinConfigReader::readDefaultBoardCfg(yarp::os::Searchable& config, SkinBoardCfgParam *boardCfg)
{
    if(NULL == boardCfg)
        return false;
    
    Bottle boardCfgDefGroup =config.findGroup("defaultCfgBoard", "Default configuration for skin boards");
    if (boardCfgDefGroup.isNull()==true)
    {
        yError() << "skin " << _name << "defaultCfgBoard param is not present in config file";
        return false;
    }
    Bottle xtmp;
    xtmp = boardCfgDefGroup.findGroup("period");
    if (xtmp.isNull())
    {
        yError() << "skin " << _name << "doesn't find period in defaultCfgBoard group in xml file";
        return false;
    }
    boardCfg->period = xtmp.get(1).asInt();

    xtmp = boardCfgDefGroup.findGroup("skinType");
    if (xtmp.isNull())
    {
        yError() << "skin " << _name << "doesn't find skinType in defaultCfgBoard group in xml file";
        return false;
    }
    boardCfg->skinType= xtmp.get(1).asInt();

    xtmp = boardCfgDefGroup.findGroup("noLoad");
    if (xtmp.isNull())
    {
        yError() << "skin " << _name << "doesn't find noLoad in defaultCfgBoard group in xml file";
        return false;
    }
    boardCfg->noLoad= xtmp.get(1).asInt();

    xtmp = boardCfgDefGroup.findGroup("diagnostic");
    if (xtmp.isNull())
    {
        yError() << "skin " << _name << "doesn't find diagnostic in defaultCfgBoard group in xml file";
        return false;
    }
    return true;
}

bool SkinConfigReader::isDefaultTriangleCfgPresent(yarp::os::Searchable& config)
{
    Bottle triangleCfgDefGroup =config.findGroup("defaultCfgTriangle", "Default configuration for skin triangle");

    if (triangleCfgDefGroup.isNull()==true)
        return false;
    else
        return true;
}

bool SkinConfigReader::readDefaultTriangleCfg(yarp::os::Searchable& config, SkinTriangleCfgParam *triangCfg)
{
    if(NULL == triangCfg)
        return false;
    
    Bottle triangleCfgDefGroup =config.findGroup("defaultCfgTriangle", "Default configuration for skin triangle");
    
    if (triangleCfgDefGroup.isNull()==true)
    {
        yError() << "skin " << _name << "defaultCfgTriangle param is not present in config file";
        return false;
    }

    Bottle xtmp;
    xtmp = triangleCfgDefGroup.findGroup("enabled");
    if (xtmp.isNull())
    {
        yError() << "skin " << _name << "doesn't find enabled in defaultCfgTriangle group in xml file";
        return false;
    }
    triangCfg->enabled = xtmp.get(1).asBool();

    xtmp = triangleCfgDefGroup.findGroup("shift");
    if (xtmp.isNull())
    {
        yError() << "skin " << _name << "doesn't find shift in defaultCfgTriangle group in xml file";
        return false;
    }
    triangCfg->shift = xtmp.get(1).asInt();

    xtmp = triangleCfgDefGroup.findGroup("cdcOffset");
    if (xtmp.isNull())
    {
        yError() << "skin " << _name << "doesn't find cdcOffset in defaultCfgTriangle group in xml file";
        return false;
    }
    triangCfg->cdcOffset = xtmp.get(1).asInt();
    
    return true;
}

bool SkinConfigReader::readSpecialBoardCfg(yarp::os::Searchable& config, SpecialSkinBoardCfgParam *boardCfg, int *numofcfg)
{
    Bottle          bNumOfset;
    int             numOfSets;

    
    if((NULL == boardCfg) || (NULL == numofcfg))
        return false;
    
    Bottle boardCfgSpecialGroup = config.findGroup("specialCfgBoards", "Special configuration for skin boards");
    if(boardCfgSpecialGroup.isNull()==true) //not mandatory field
    {
        *numofcfg = 0;
        return true;
    }


    bNumOfset = boardCfgSpecialGroup.findGroup("numOfSets", "number of special sets of triangles");
    if(bNumOfset.isNull())
    {
        yError() << "skin " << _name << "numOfSet is missed from specialCfgBoards";
        return(false);
    }
    
    numOfSets =  bNumOfset.get(1).asInt();
    if(numOfSets > *numofcfg)
    {
        yWarning() << "skin " << _name << "numOfSet is too big. Max is " << *numofcfg;
    }
    else
    {
        *numofcfg = numOfSets;
    }

    for(int j=1;j<=*numofcfg;j++)
    {
        char tmp[80];
        sprintf(tmp, "boardSetCfg%d", j);

        Bottle &xtmp = boardCfgSpecialGroup.findGroup(tmp);
        if(xtmp.isNull())
        {
            yError() << "skin " << _name << "doesn't find " << tmp << "in specialCfgBoards group in xml file";
            return false;
        }

        boardCfg[j-1].patch          = xtmp.get(1).asInt();
        boardCfg[j-1].boardAddrStart = xtmp.get(2).asInt();
        boardCfg[j-1].boardAddrEnd   = xtmp.get(3).asInt();
        boardCfg[j-1].cfg.period     = xtmp.get(4).asInt();
        boardCfg[j-1].cfg.skinType   = xtmp.get(5).asInt();
        boardCfg[j-1].cfg.noLoad     = xtmp.get(6).asInt();
    }
    return true;
}


bool SkinConfigReader::readSpecialTriangleCfg(yarp::os::Searchable& config, SpecialSkinTriangleCfgParam *triangleCfg, int *numofcfg)
{
    Bottle          bNumOfset;
    int             numOfSets;
    
   if((NULL == triangleCfg) || (NULL == numofcfg))
       return false;
   
    Bottle triangleCfgSpecialGroup = config.findGroup("specialCfgTriangles", "Special configuration for skin triangles");
    if(triangleCfgSpecialGroup.isNull()) //not mandatory field
    {
        *numofcfg = 0;
        return true;
    }

    bNumOfset = triangleCfgSpecialGroup.findGroup("numOfSets", "number of special sets of triangles");
    if(bNumOfset.isNull())
    {
        yError() << "skin "<< _name << "numOfSet is missed from SpecialCfgTriangles";
        return(false);
    }

    numOfSets =  bNumOfset.get(1).asInt();
    if(numOfSets > *numofcfg)
    {
        yWarning() << "skin " << _name << "numOfSets for specialCfgTriangles is too big!! Max is " <<  *numofcfg;
    }
    else
    {
        *numofcfg = numOfSets;
    }

    for(int j=1;j<=*numofcfg;j++)
    {
        char tmp[80];
        sprintf(tmp, "triangleSetCfg%d", j);

        Bottle &xtmp = triangleCfgSpecialGroup.findGroup(tmp);
        if(xtmp.isNull())
        {
            yError() << "skin " << _name << "doesn't find " << tmp << "in SpecialCfgTriangles group in xml file";
            return false;
        }

        triangleCfg[j-1].patch          = xtmp.get(1).asInt();
        triangleCfg[j-1].boardAddr      = xtmp.get(2).asInt();
        triangleCfg[j-1].triangleStart  = xtmp.get(3).asInt();
        triangleCfg[j-1].triangleEnd    = xtmp.get(4).asInt();
        triangleCfg[j-1].cfg.enabled    = xtmp.get(5).asInt();
        triangleCfg[j-1].cfg.shift      = xtmp.get(6).asInt();
        triangleCfg[j-1].cfg.cdcOffset  = xtmp.get(7).asInt();


//        //VALE: solo per debug:
//        yError() << "num of set: " << j;
//        yError() << "tcfg.patch" << triangleCfg[j-1].patch ;
//        yError() << "tcfg.boardaddr" << triangleCfg[j-1].boardAddr ;
//        yError() << "tcfg.idstart" << triangleCfg[j-1].triangleStart;
//        yError() << "tcfg.idend" << triangleCfg[j-1].triangleEnd;
//        yError() << "tcfg.cfg.enable" << triangleCfg[j-1].cfg.enabled;
//        yError() << "tcfg.cfg.shift" << triangleCfg[j-1].cfg.shift;
//        yError() << "tcfg.cfg.CDCoffset" << triangleCfg[j-1].cfg.cdcOffset;
    }
    return true;
}
