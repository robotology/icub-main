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
#ifndef __GLSNAPSHOT_H__
#define __GLSNAPSHOT_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gl2ps.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


enum GLSnapshotMode{
  GLSnap_NONE,
  GLSnap_EPS,
  GLSnap_RGB
};

class GLSnapshot
{
private:
  FILE           *m_File;
  GLSnapshotMode  m_Mode;

  char            m_Filename[512];

  int             m_Width;
  int             m_Height;

public:
  GLSnapshot();
  ~GLSnapshot();

  void  SetViewport(int width, int height);
  void  SetFilename(const char *filename);
  void  SetMode(GLSnapshotMode mode);
  int   Begin();
  int   Finish();
};


#endif
