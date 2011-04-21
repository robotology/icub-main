// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick
* email:   vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu
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

#include "iCubLogicalJoints.h"

#include <stdlib.h>

iCubLogicalJoints::iCubLogicalJoints(RobotConfig& config) {
    RobotFlags& flags = config.getFlags();
    if (!flags.valid) {
        printf("Robot flags are not set when creating iCubLogicalJoints\n");
        exit(1);
    }
    
    ////////////////////////////////////////////////////////////
    // Setting up the head
    
    int head = PART_HEAD;
    int rawHead = PART_HEAD_RAW;
    if (flags.actHead){      
        const char *headName = "head";
        // for (int i=0; i<4; i++) {
        getController(head,0).init(headName,"hinge",0,+1);
        getController(head,1).init(headName,"hinge",1,-1);
        getController(head,2).init(headName,"hinge",2,-1);
        getController(head,3).init(headName,"hinge",3,+1);
        // }
        // for the eyes, we need to map vergence/version onto
        // individual hinge joints
        getController(rawHead,4).init(headName,"hinge",4,-1);
        getController(rawHead,5).init(headName,"hinge",5,+1);
        
        getController(head,4).init(getController(rawHead,4),
                                   getController(rawHead,5),
                                   getController(head,5),
                                   +1);
        getController(head,5).init(getController(rawHead,4),
                                   getController(rawHead,5),
                                   getController(head,4),
                                   -1);
    }
    ////////////////////////////////////////////////////////////
    // Setting up the left and right arms
    
    for (int arm = PART_ARM_LEFT; arm <= PART_ARM_RIGHT; arm++) {
        const char *armName = (arm==PART_ARM_LEFT)?"leftarm":"rightarm";

        if (arm == PART_ARM_RIGHT && flags.actRArm){
            getController(arm,0).init(armName,"hinge",0,-1);
            getController(arm,1).init(armName,"hinge",1,+1);
            getController(arm,2).init(armName,"hinge",2,-1);
            getController(arm,3).init(armName,"hinge",3,+1);
            getController(arm,4).init(armName,"hinge",4,-1);
            getController(arm,5).init(armName,"universalAngle1",5,-1);
            getController(arm,6).init(armName,"universalAngle2",5,-1);
        }

        if (arm == PART_ARM_LEFT && flags.actLArm){
            getController(arm,0).init(armName,"hinge",0,-1);
            getController(arm,1).init(armName,"hinge",1,-1);
            getController(arm,2).init(armName,"hinge",2,+1);
            getController(arm,3).init(armName,"hinge",3,+1);
            getController(arm,4).init(armName,"hinge",4,+1);
            getController(arm,5).init(armName,"universalAngle1",5,+1);
            getController(arm,6).init(armName,"universalAngle2",5,-1);

        }
        if (arm == PART_ARM_RIGHT && flags.actRHand){
            getController(arm,7).init(armName,"hinge",6,+1);
            OdeLogicalJoint *sub = getController(arm,7).nest(1);
            sub[0].init(armName,"hinge",8,-1);

            getController(arm,8).init(armName,"universalAngle1",22,-1);//thumb
            getController(arm,9).init(armName,"universalAngle2",22,-1);//thumb

            getController(arm,10).init(armName,"hinge",23,-1);
            sub = getController(arm,10).nest(1);
            sub[0].init(armName,"hinge",24,-1);
        
            getController(arm,11).init(armName,"hinge",10,-1);//index proximal
            getController(arm,12).init(armName,"hinge",14,-1);
            sub = getController(arm,12).nest(1);
            sub[0].init(armName,"hinge",18,-1);

            getController(arm,13).init(armName,"hinge",11,-1);//middle finger
            getController(arm,14).init(armName,"hinge",15,-1);
            sub = getController(arm,14).nest(1);
            sub[0].init(armName,"hinge",19,-1);

            getController(arm,15).init(armName,"hinge",12,-1);//ring + pinky
            sub = getController(arm,15).nest(2);
            sub[0].init(armName,"hinge",16,-1);
            sub[1].init(armName,"hinge",20,-1);
        }
        if (arm == PART_ARM_LEFT && flags.actLHand ){
            getController(arm,7).init(armName,"hinge",6,+1);
            OdeLogicalJoint *sub = getController(arm,7).nest(1);
            sub[0].init(armName,"hinge",8,-1);

            /*getController(arm,8).init(armName,"hinge",22,+1);//thumb
              getController(arm,9).init(armName,"hinge",23,+1);
              getController(arm,10).init(armName,"hinge",24,+1);
            */
            getController(arm,8).init(armName,"universalAngle1",22,+1);//thumb
            getController(arm,9).init(armName,"universalAngle2",22,-1);//thumb

            getController(arm,10).init(armName,"hinge",23,+1);
            sub = getController(arm,10).nest(1);
            sub[0].init(armName,"hinge",24,+1);
        
            getController(arm,11).init(armName,"hinge",10,+1);//index proximal
            getController(arm,12).init(armName,"hinge",14,+1);
            sub = getController(arm,12).nest(1);
            sub[0].init(armName,"hinge",18,+1);

            getController(arm,13).init(armName,"hinge",11,+1);//middle finger
            getController(arm,14).init(armName,"hinge",15,+1);
            sub = getController(arm,14).nest(1);
            sub[0].init(armName,"hinge",19,+1);

            getController(arm,15).init(armName,"hinge",12,+1);//ring + pinky
            sub = getController(arm,15).nest(2);
            sub[0].init(armName,"hinge",16,+1);
            sub[1].init(armName,"hinge",20,+1);
        }
    }

    if (flags.actLegs){
        for (int leg = PART_LEG_LEFT; leg <= PART_LEG_RIGHT; leg++) {
            const char *legName = (leg==PART_LEG_LEFT)?"leftleg":"rightleg";
            //       changed for demo look below for previous joint setup
            if (leg == PART_LEG_RIGHT){
                getController(leg,0).init(legName,"hinge",5,-1);
                getController(leg,1).init(legName,"hinge",4,-1);
                getController(leg,2).init(legName,"hinge",3,-1);
                getController(leg,3).init(legName,"hinge",2,-1);
                getController(leg,4).init(legName,"hinge",1,+1);
                getController(leg,5).init(legName,"hinge",0,-1);
            }else{
                getController(leg,0).init(legName,"hinge",5,-1);
                getController(leg,1).init(legName,"hinge",4,+1);
                getController(leg,2).init(legName,"hinge",3,+1);
                getController(leg,3).init(legName,"hinge",2,-1);
                getController(leg,4).init(legName,"hinge",1,+1);
                getController(leg,5).init(legName,"hinge",0,+1);
            }
        }
    }

    if (flags.actTorso){
        int torso = PART_TORSO;
        const char *torsoName = "torso";
        getController(torso,0).init(torsoName,"hinge",2,+1);
        getController(torso,1).init(torsoName,"hinge",1,+1);
        getController(torso,2).init(torsoName,"hinge",0,-1);
    }
}

LogicalJoint& iCubLogicalJoints::control(int part, int axis) {
    return getController(part, axis);
}
