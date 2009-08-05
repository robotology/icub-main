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
#include "GLSnapshot.h"
//#include "gl2ps.h"



GLSnapshot::GLSnapshot(){
  m_File    = NULL;
  m_Mode    = GLSnap_NONE;
  m_Width   = 100;
  m_Height  = 100;
}

GLSnapshot::~GLSnapshot(){
  if(m_File!=NULL)
    Finish();
}

void  GLSnapshot::SetViewport(int width, int height){
  m_Width   = width;
  m_Height  = height;
}
void  GLSnapshot::SetFilename(const char *filename){
  strcpy(m_Filename,filename);
}
void  GLSnapshot::SetMode(GLSnapshotMode mode){
  m_Mode = mode;
}

int GLSnapshot::Begin(){
  if(m_File!=NULL)
    Finish();

  if(m_Mode==GLSnap_NONE)
    return TRUE;

  m_File = fopen(m_Filename, "wb");
  
  if(m_File==NULL)
    return FALSE;

  switch(m_Mode){
  case GLSnap_NONE:
    break;
//   case GLSnap_SVG:
//   case GLSnap_PDF:
  case GLSnap_EPS: {


    GLint viewport[4];
    viewport[0] = 0; viewport[1] = 0; viewport[2] = 800; viewport[3] = 600;

    int options   = GL2PS_USE_CURRENT_VIEWPORT;// | GL2PS_OCCLUSION_CULL;
    int buffsize  = 1024*1024;
    int nbColors  = 0;
    int sort      = GL2PS_SIMPLE_SORT;//GL2PS_SIMPLE_SORT;//GL2PS_BSP_SORT; //GL2PS_NO_SORT,

    int ret;
    ret = gl2psBeginPage(m_Filename,
                   "GLSnapshot",
                   NULL, 
                   GL2PS_EPS, 
                   sort, 
                   options,
                   GL_RGBA,
                   0,
                   NULL,
                   nbColors, nbColors, nbColors,
                   buffsize, 
                   m_File, m_Filename);
  }
    break;
  case GLSnap_RGB:
    break;
  }
  return TRUE;
}

// Fonctions annexes pour la sauvegarde
#define PUT_BYTE(outFile,val)  { unsigned char buf[1];    \
                                 buf[0] = (char)(val);            \
                                 fwrite(buf,1,1,outFile); }

#define PUT_SHORT(outFile,val) { unsigned char buf[2];    \
                                 buf[0] = (char)((val) >> 8);       \
                                 buf[1] = (char)((val) >> 0);       \
                                 fwrite(buf,2,1,outFile); }

#define PUT_LONG(outFile,val)  { unsigned char buf[4];    \
                                 buf[0] = (char)((val) >> 24);      \
                                 buf[1] = (char)((val) >> 16);      \
                                 buf[2] = (char)((val) >> 8);       \
                                 buf[3] = (char)((val) >> 0);       \
                                 fwrite(buf,4,1,outFile); } 

int GLSnapshot::Finish(){
  if(m_File!=NULL){
    switch(m_Mode){
    case GLSnap_NONE:
      break;
//     case GLSnap_SVG:
//     case GLSnap_PDF:
    case GLSnap_EPS:
      int ret;
      ret = gl2psEndPage();
      break;
    case GLSnap_RGB: {      
      GLubyte * imDataR=NULL;
      GLubyte * imDataG=NULL;
      GLubyte * imDataB=NULL;

      char iname[80];
      int  i,x,y,z;

      imDataR = (GLubyte*) malloc(m_Width*m_Height*sizeof(GLubyte));
      imDataG = (GLubyte*) malloc(m_Width*m_Height*sizeof(GLubyte));
      imDataB = (GLubyte*) malloc(m_Width*m_Height*sizeof(GLubyte));

      glReadPixels(0,0,m_Width,m_Height, GL_RED,   GL_UNSIGNED_BYTE, imDataR);
      glReadPixels(0,0,m_Width,m_Height, GL_GREEN, GL_UNSIGNED_BYTE, imDataG);
      glReadPixels(0,0,m_Width,m_Height, GL_BLUE,  GL_UNSIGNED_BYTE, imDataB);
  

      PUT_SHORT(m_File,(short)474);       /* MAGIC                       */
      PUT_BYTE (m_File,(char) 0);         /* STORAGE is VERBATIM         */
      PUT_BYTE (m_File,(char) 1);         /* BPC is 1                    */
      PUT_SHORT(m_File,(short)3);         /* DIMENSION is 2              */
      PUT_SHORT(m_File,(short)m_Width);         /* XSIZE                       */
      PUT_SHORT(m_File,(short)m_Height);         /* YSIZE                       */
      PUT_SHORT(m_File,(short)3);         /* ZSIZE                       */
      PUT_LONG (m_File,(long) 0);         /* PIXMIN is 0                 */
      PUT_LONG (m_File,(long) 255);       /* PIXMAX is 255               */
      for(i=0; i<4; i++)       /* DUMMY 4 bytes               */
        PUT_BYTE(m_File,(char) 0);
  
      strcpy(iname,"GLSnapshot");
      fwrite(iname,80,1,m_File);   /* IMAGENAME                   */
      PUT_LONG(m_File,(long) 0);          /* COLORMAP is 0               */
      for(i=0; i<(404+12); i++)
        PUT_BYTE(m_File,(char) 0);        /* DUMMY 404 bytes             */
  
      for(z=0;z<3;z++){
        i=0;
        for(y=0;y<m_Height;y++){
          for(x=0;x<m_Width;x++){	
	          switch(z){
	          case 0:
	            PUT_BYTE(m_File,(char)imDataR[i]);
	              break;
	          case 1:
	            PUT_BYTE(m_File,(char)imDataG[i]);
	            break;
	          case 2:
	            PUT_BYTE(m_File,(char)imDataB[i]);
	            break;
	          }
	          i++;
          }
        }
      }
   
      free(imDataR);
      free(imDataG);
      free(imDataB);
    }
      break;
    }
    fclose(m_File);
    m_File = NULL;
  }
  return TRUE;
}



