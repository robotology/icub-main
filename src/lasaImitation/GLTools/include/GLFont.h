#ifndef __GLFONT_H__
#define __GLFONT_H__

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include <stdlib.h>
#include <stdio.h>

class GLFont
{
protected:
  unsigned int  m_TextureFont;
  bool          m_IsReady;
  unsigned int  m_DiplayListBase;

public:
  GLFont();
  ~GLFont();

  void  LoadFont();
  void  Free();
  void  Print(const char *string, int maxLen = -1);

};

#endif

