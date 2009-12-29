#ifndef __GL2DPLOT_H__
#define __GL2DPLOT_H__

#include <stdlib.h>
//#include <GL/glut.h>
#include <vector>
using namespace std;

#include "GLSubWindow.h"

typedef struct{
  float r;
  float g;
  float b;  
} RGBColor, *pRGBColor;


class GL2DPlot : public GLSubWindow
{
protected:
  int               m_NbPlots;
  vector<int>       m_PlotType;
  vector<int>       m_PlotSize;
  vector<float*>    m_XValues;
  vector<float*>    m_YValues;
  vector<RGBColor>  m_Color;
  vector<int>       m_LineStyle;
  vector<float>     m_LineWidth;

  float             m_MinX, m_MaxX;
  float             m_MinY, m_MaxY;
public:
  bool              m_AxesAuto;

  int               m_Offset;
public:
  GL2DPlot(pGLSubWindow parentWindow = NULL);
  ~GL2DPlot();

          void  Clear();
          void  AddPlot(float* YVal, int size);
          void  AddPlot(float* XVal, float* YVal, int size);
          void  Add2DSurfPlot(float* YVal0, float* YVal1, int size);
          void  Add2DSurfPlot(float* XVal, float* YVal0, float* YVal1, int size);
          void  AddVarSurfPlot(float* YVal0, float* YVal1, int size);
          void  AddVarSurfPlot(float* XVal, float* YVal0, float* YVal1, int size);
          void  SetColor(float r, float g, float b);
          void  SetColor(int no, float r, float g, float b);
          void  SetLineWidth(int no, float w);
          void  SetDataArray(int no, float *YVal, int size);
          void  SetAxes(float minX, float minY, float maxX, float maxY);
          void  GetAxes(float *minX, float *minY, float *maxX, float *maxY);
          void  ClearAxes();
          void  GetDataBoundaries(float *rminX,float *rminY,float*rmraxX,float*rmaxY);
          
  virtual void  Render();

  virtual void  Resize(int w, int h);
  virtual void  OnNormalKey(char key);
  virtual void  OnSpecialKey(int key);

          void  SetOffset(int off);
};
typedef GL2DPlot *pGL2DPlot;

#endif
