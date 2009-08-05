// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
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
#ifndef __EMBODIED_ARTICULATED_TREE_H__
#define __EMBODIED_ARTICULATED_TREE_H__

#include "TreeParser.h"
#include "Shape.h"
#include "KinematicTree.h"


/**
 * @brief An articulated tree with rendring function for openGL display
 */
class EmbodiedArticulatedTree : public ArticulatedTree
{
 protected:
  Shape *solid;

 protected:
  void RenderBegin();
  void RenderEnd();

 public:
    EmbodiedArticulatedTree();
    EmbodiedArticulatedTree(const pTree tree);
    virtual ~EmbodiedArticulatedTree();
    void SetShape(Shape *sol);
    void UpdateShape();  
    Shape *GetShape(){return solid;};
    int SetAllShapes(pTree shape_tree);
    /**
     * @param update 0 if the shape should not be updated according the ArticulatedTree
     */ 
 void Render();
  void Highlight(){solid->Highlight();};
  virtual void StreamTree(ostream& out)const;
};


class EmbodiedKinematicTree : public KinematicTree
{
 protected:
  EmbodiedArticulatedTree *e_root;
  
public:
    EmbodiedKinematicTree();
    EmbodiedKinematicTree(char *filename1, char *filename2);
    void Render(){e_root->Render();};
    void UpdateShape(){e_root->UpdateShape();};
    int CheckAllCollisions(KinematicTree *update_tree);
    static int CollisionDetection(pArticulatedTree_t t1,
                                  pArticulatedTree_t t2, void *arg);
};


#endif
