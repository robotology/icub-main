
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
using namespace std;

//#include <GL/glut.h>

#include "GLTools.h"


float  GLTools::m_red     = 1.0;
float  GLTools::m_green   = 1.0;
float  GLTools::m_blue    = 1.0;
float  GLTools::m_alpha   = 1.0;

bool   GLTools::m_outline = true;
bool   GLTools::m_solid   = true;

GLFont GLTools::m_Font;

void GLTools::SetColor(float r, float g, float b, float a){
  m_red = r;  m_green = g;   m_blue = b; m_alpha = a;
};

void GLTools::DrawOutline(bool state){
  m_outline = state;
}
void GLTools::DrawSolid(bool state){
  m_solid = state;
}

/*
void GLTools::DrawArc(float min, float max, CMatrix4_t *ref){
  if((!m_outline) && (!m_solid))
    return;
  
  glPushMatrix();
  if(ref!=NULL)
    glMultMatrixf(*ref);

  float stepSize  = pi/32.0;
  int   step      = (int)ceil((max - min)*RAD2DEG / stepSize);
  int   i;
  float currAngle = min*RAD2DEG;

  if(m_solid){
    glColor3f(m_red, m_green, m_blue);
    glBegin(GL_TRIANGLE_FAN);
  		glVertex3f( 0.0f, 0.0f,  0.0f);
      for(i=0;i<step;i++){
        glVertex3f(cos(currAngle), sin(currAngle), 0.0f);
        currAngle += stepSize;
      }
      glVertex3f(cos(max*RAD2DEG),  sin(max*RAD2DEG), 0.0f);
	  glEnd();
  }

  if(m_outline){
    glScalef(1.001,1.001,1.001);
    currAngle = min*RAD2DEG;
    glColor3f(0.0,0.0,0.0);
    glBegin(GL_LINE_LOOP);
  		glVertex3f( 0.0f, 0.0f,  0.0f);
      for(i=0;i<step;i++){
        glVertex3f(cos(currAngle),  sin(currAngle), 0.0f);
        currAngle += stepSize;
      }
      glVertex3f(cos(max*RAD2DEG),  sin(max*RAD2DEG), 0.0f);
	  glEnd();  
  }

  glPopMatrix();
}


void GLTools::Draw3DArc(CVector3_List_t * vl, CMatrix4_t *ref){
  if((!m_outline) && (!m_solid))
    return;
  
  glPushMatrix();
  if(ref!=NULL)
    glMultMatrixf(*ref);


  int   step      = (int)(*vl).size();
  int   i;

  if(m_solid){
    glColor3f(m_red, m_green, m_blue);
    glBegin(GL_TRIANGLE_FAN);
  		glVertex3f( 0.0f, 0.0f,  0.0f);
      for(i=0;i<step;i++){ 
        glVertex3f(((*vl)[i]).m_Vector[0],((*vl)[i]).m_Vector[1],((*vl)[i]).m_Vector[2]);
      }
	  glEnd();
  }

  if(m_outline){
    glScalef(1.001,1.001,1.001);
    glColor3f(0.0,0.0,0.0);
    glBegin(GL_LINE_STRIP);
  		//glVertex3f( 0.0f, 0.0f,  0.0f);
      for(i=0;i<step;i++){
        glVertex3f(((*vl)[i]).m_Vector[0],((*vl)[i]).m_Vector[1],((*vl)[i]).m_Vector[2]);
      }
	  glEnd();  
  }

  glPopMatrix();
}

*/

void GLTools::DrawLines       (Matrix &verticesList,int offset){
  glPushMatrix();
  glLineWidth(2.0f);
  const int length = verticesList.RowSize();
  //printf("%d %d\n",verticesList.RowSize(),verticesList.ColumnSize());
  glColor4f(m_red, m_green, m_blue,m_alpha);
  glBegin(GL_LINE_STRIP);
    for(int i=offset;i<length;i++){
        float x,y,z,n;
        x = verticesList(i,0);
        y = verticesList(i,1);
        z = verticesList(i,2);
        n = sqrt(x*x+y*y+z*z);
        if(n>0.001)
          glNormal3f(x/n,y/n,z/n);
        glVertex3f(x,y,z);
    }
    for(int i=0;i<offset;i++){
        float x,y,z,n;
        x = verticesList(i,0);
        y = verticesList(i,1);
        z = verticesList(i,2);
        n = sqrt(x*x+y*y+z*z);
        if(n>0.001)
          glNormal3f(x/n,y/n,z/n);
        glVertex3f(x,y,z);
    }
  glEnd();  
  glLineWidth(1.0f);
  glPopMatrix();
}

void GLTools::DrawVector(const Vector3 &v, float radius, const Matrix4 *ref){
  Vector3 vec(v);
  vec.Normalize();
  
  float norm = v.Norm();

  float orie = atan2(vec[0],vec[2]);
  float elev = asin(vec[1]);

  glPushMatrix();
  if(ref!=NULL)
    glMultMatrixf(ref->RowOrderForceFloat());
 	
  glRotatef(RAD2DEG(orie),0.0f,1.0f,0.0f);
  glRotatef(-RAD2DEG(elev),1.0f,0.0f,0.0f);

  glScalef(radius,radius,norm);

  glPushMatrix();
  glTranslatef(0.0f,0.0f,0.25f);
  glScalef(0.1f,0.1f,1.0f);
  DrawCube(0.5f);
  glPopMatrix();

  glTranslatef(0.0f,0.0f,0.5f);
  glScalef(0.1f,0.1f,1.0f);
  DrawCone(1.0f,0.5f,4);
  glPopMatrix();
}


void GLTools::DrawSegment(const Vector3 &v, float radius){
  Vector3 vec(v);
  vec.Normalize();
  
  float norm = v.Norm();

  float orie = atan2(vec[0],vec[2]);
  float elev = asin(vec[1]);

  glPushMatrix();
  
  glRotatef(RAD2DEG(orie),0.0f,1.0f,0.0f);
  glRotatef(-RAD2DEG(elev),1.0f,0.0f,0.0f);

  glScalef(1.0f,1.0f,norm);

  glPushMatrix();
  //glScalef(0.1,0.1,1.0);
  DrawCylinder(radius,1.0f);
  glPopMatrix();
/*
  glTranslatef(0.0,0.0,0.5);
  glScalef(0.1,0.1,1.0);
  DrawCone(1.0,0.5,4);*/
  glPopMatrix();  
}

void GLTools::DrawPlane(const Vector3 &v, const Matrix4 *ref){
  if((!m_outline) && (!m_solid))
    return;
  
  Vector3 vec(v);
  vec.Normalize();
  float norm = v.Norm();

  float orie = atan2(vec[0],vec[2]);
  float elev = asin(vec[1]);

  glPushMatrix();
  if(ref!=NULL)
    glMultMatrixf(ref->RowOrderForceFloat());
 	
  glRotatef(RAD2DEG(orie), 0.0f, 1.0f, 0.0f);
  glRotatef(-RAD2DEG(elev), 1.0f, 0.0f, 0.0f);


  glScalef(norm,norm,1.0);
  if(m_solid){
    glColor4f(m_red, m_green, m_blue, m_alpha);
    glBegin(GL_QUADS);
      glNormal3f( 0.0f, 0.0f,  1.0f);
		  glVertex3f(-1.0f,-1.0f,  0.0f);
      glVertex3f( 1.0f,-1.0f,  0.0f);
      glVertex3f( 1.0f, 1.0f,  0.0f);
		  glVertex3f(-1.0f, 1.0f,  0.0f);
      glNormal3f( 0.0f, 0.0f, -1.0f);
      glVertex3f(-1.0f,-1.0f,  0.0f);
      glVertex3f(-1.0f, 1.0f,  0.0f);
      glVertex3f( 1.0f, 1.0f,  0.0f);
      glVertex3f( 1.0f,-1.0f,  0.0f);
	  glEnd();
  }
  if(m_outline){
	  glScalef(1.001f,1.001f,1.0f);
	  glColor4f(0.0f, 0.0f, 0.0f,1.0f);
	  glBegin(GL_LINE_STRIP);
  		glVertex3f(-1.0f,-1.0f,  0.0f);
		  glVertex3f(-1.0f, 1.0f,  0.0f);
		  glVertex3f( 1.0f, 1.0f,  0.0f);
		  glVertex3f( 1.0f,-1.0f,  0.0f);
		  glVertex3f(-1.0f,-1.0f,  0.0f);
	  glEnd();
  }
  glPopMatrix();
}

void GLTools::DisplayText(float x, float y, const char * text, float fontHeight, int maxLen){
  glColor3f(1.0f,1.0f,1.0f);
  
  glPushMatrix();
  glTranslatef(x,y-fontHeight,0.0f);
  glScalef(float(fontHeight),-float(fontHeight),0.0f);
  m_Font.Print(text,maxLen);  
  glPopMatrix();
  
}

void GLTools::DisplayText(int x, int y, const char * text, int fontHeight, int maxLen){
  glColor3f(1.0f,1.0f,1.0f);
  
  glPushMatrix();
  glTranslatef(float(x),float(y)-float(fontHeight)+1,0.0f);
  glScalef(float(fontHeight),float(fontHeight),0.0f);
  m_Font.Print(text,maxLen);  
  glPopMatrix();
  
}

void GLTools::DrawCube(float sideLength){
  glPushMatrix();
  glScalef(sideLength/2.0f,sideLength/2.0f,sideLength/2.0f);
  if(m_solid){
    glColor4f(m_red, m_green, m_blue, m_alpha);
    glBegin(GL_QUADS);
      glNormal3f( 0.0f, 0.0f,-1.0f);
      glVertex3f(-1.0f,-1.0f,-1.0f);
      glVertex3f(-1.0f, 1.0f,-1.0f);
      glVertex3f( 1.0f, 1.0f,-1.0f);
      glVertex3f( 1.0f,-1.0f,-1.0f);
  
      glNormal3f( 0.0f, 0.0f, 1.0f);
      glVertex3f(-1.0f,-1.0f, 1.0f);
      glVertex3f( 1.0f,-1.0f, 1.0f);
      glVertex3f( 1.0f, 1.0f, 1.0f);
      glVertex3f(-1.0f, 1.0f, 1.0f);

      glNormal3f(-1.0f, 0.0f, 0.0f);  
      glVertex3f(-1.0f,-1.0f,-1.0f);
      glVertex3f(-1.0f,-1.0f, 1.0f);
      glVertex3f(-1.0f, 1.0f, 1.0f);
      glVertex3f(-1.0f, 1.0f,-1.0f);
  
      glNormal3f( 1.0f, 0.0f, 0.0f);  
      glVertex3f( 1.0f,-1.0f,-1.0f);
      glVertex3f( 1.0f, 1.0f,-1.0f);
      glVertex3f( 1.0f, 1.0f, 1.0f);
      glVertex3f( 1.0f,-1.0f, 1.0f);
  
      glNormal3f( 0.0f,-1.0f, 0.0f);  
      glVertex3f(-1.0f,-1.0f,-1.0f);
      glVertex3f( 1.0f,-1.0f,-1.0f);
      glVertex3f( 1.0f,-1.0f, 1.0f);
      glVertex3f(-1.0f,-1.0f, 1.0f);
  
      glNormal3f( 0.0f, 1.0f, 0.0f);  
      glVertex3f(-1.0f, 1.0f,-1.0f);
      glVertex3f(-1.0f, 1.0f, 1.0f);
      glVertex3f( 1.0f, 1.0f, 1.0f);
      glVertex3f( 1.0f, 1.0f,-1.0f);
    glEnd();
  }
  if(m_outline){
    glColor3f(0.0f,0.0f,0.0f);
    glScalef(1.001f,1.001f,1.001f);
    glBegin(GL_LINE_LOOP);
      glNormal3f( 0.0f, 0.0f,-1.0f);
      glVertex3f(-1.0f,-1.0f,-1.0f);
      glVertex3f(-1.0f, 1.0f,-1.0f);
      glVertex3f( 1.0f, 1.0f,-1.0f);
      glVertex3f( 1.0f,-1.0f,-1.0f);
    glEnd();
  
    glBegin(GL_LINE_LOOP);
      glNormal3f( 0.0f, 0.0f, 1.0f);
      glVertex3f(-1.0f,-1.0f, 1.0f);
      glVertex3f(-1.0f, 1.0f, 1.0f);
      glVertex3f( 1.0f, 1.0f, 1.0f);
      glVertex3f( 1.0f,-1.0f, 1.0f);
    glEnd();
  
    glBegin(GL_LINE_LOOP);
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(-1.0f,-1.0f,-1.0f);
      glVertex3f(-1.0f,-1.0f, 1.0f);
      glVertex3f(-1.0f, 1.0f, 1.0f);
      glVertex3f(-1.0f, 1.0f,-1.0f);
    glEnd();
  
    glBegin(GL_LINE_LOOP);
      glNormal3f( 1.0f, 0.0f, 0.0f);    
      glVertex3f( 1.0f,-1.0f,-1.0f);
      glVertex3f( 1.0f,-1.0f, 1.0f);
      glVertex3f( 1.0f, 1.0f, 1.0f);
      glVertex3f( 1.0f, 1.0f,-1.0f);
    glEnd();
  
    glBegin(GL_LINE_LOOP);
      glNormal3f( 0.0f,-1.0f, 0.0f);    
      glVertex3f(-1.0f,-1.0f,-1.0f);
      glVertex3f(-1.0f,-1.0f, 1.0f);
      glVertex3f( 1.0f,-1.0f, 1.0f);
      glVertex3f( 1.0f,-1.0f,-1.0f);
    glEnd();
  
    glBegin(GL_LINE_LOOP);
      glNormal3f( 0.0f, 1.0f, 0.0f);    
      glVertex3f(-1.0f, 1.0f,-1.0f);
      glVertex3f(-1.0f, 1.0f, 1.0f);
      glVertex3f( 1.0f, 1.0f, 1.0f);
      glVertex3f( 1.0f, 1.0f,-1.0f);
      glVertex3f(-1.0f, 1.0f,-1.0f);
    glEnd();    
  }  
  glPopMatrix();
}

void GLTools::DrawCone(float radius, float height, int slices){
  if(slices<2)
    return;
  
  glPushMatrix();  
  float dangle = 2.0f*PIf/float(slices);    
  
  if(m_solid){
    glColor4f(m_red, m_green, m_blue, m_alpha);
    glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0f,0.0f,1.0f);
      glVertex3f(0.0f,0.0f,height);
      float x,y,h,c,s;
      h = sqrtf(height*height+radius*radius); 
      c = radius/h;
      s = height/h;
      for(int i=0;i<slices+1;i++){
        x = cosf(float(i)*dangle)*radius;
        y = sinf(float(i)*dangle)*radius;

        glNormal3f(cosf((float(i))*dangle),sinf((float(i))*dangle),0.0f);
        //glNormal3f(s * cosf((float(i))*dangle),
        //           s * sinf((float(i))*dangle),
        //           c);
        glVertex3f(x,y,0.0f);
      }      
    glEnd();
    glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0f,0.0f,-1.0f);
      glVertex3f(0.0f,0.0f,0.0f);
      for(int i=slices;i>=0;i--){
        float x = cosf(float(i)*dangle)*radius;
        float y = sinf(float(i)*dangle)*radius;
        glVertex3f(x,y,0.0f);
      }      
    glEnd();
  }
  if(m_outline){
    glColor3f(0.0f,0.0f,0.0f);
    glScalef(1.001f,1.001f,1.001f);
    glBegin(GL_LINES);
      for(int i=0;i<slices+1;i++){
        glVertex3f(0.0f,0.0f,height);
        float x = cosf(float(i)*dangle)*radius;
        float y = sinf(float(i)*dangle)*radius;
        glVertex3f(x,y,0.0f);
      }      
    glEnd();  
    glBegin(GL_LINE_LOOP);
      for(int i=0;i<slices;i++){
        float x = cosf(float(i)*dangle)*radius;
        float y = sinf(float(i)*dangle)*radius;
        glVertex3f(x,y,0.0f);
      }      
    glEnd();  
  }  
  glPopMatrix();  
}

void GLTools::DrawHalfSphere  (float radius,int slices, int stacks){
  DrawSphere(radius,slices,stacks,PIf/2.0f); 
}


void GLTools::DrawSphere(float radius, int slices, int stacks, float radHeight){
  if((slices<2)||(stacks<2))
    return;
  
  glPushMatrix();  
  float dangle1 = 2.0f*PIf/float(slices);    
  float dangle2 = radHeight/float(stacks);  

  if(m_solid){
    glColor4f(m_red, m_green, m_blue, m_alpha);

  
    for(int j=0;j<stacks;j++){
      glBegin(GL_TRIANGLE_STRIP);
        float x,y,z;
        for(int i=0;i<slices+1;i++){
          x=sinf(float(j)*dangle2) * cosf(float(i)*dangle1);
          y=sinf(float(j)*dangle2) * sinf(float(i)*dangle1);
          z=cosf(float(j)*dangle2);
          glNormal3f(x,y,z);
          glVertex3f(radius* x,radius* y,radius* z);
          x=sinf(float(j+1)*dangle2) * cosf(float(i)*dangle1);
          y=sinf(float(j+1)*dangle2) * sinf(float(i)*dangle1);
          z=cosf(float(j+1)*dangle2);
          glNormal3f(x,y,z);
          glVertex3f(radius* x,radius* y,radius* z);
        }
      glEnd();
    }
  }
  if(m_outline){
    glColor3f(0.0f,0.0f,0.0f);
    glScalef(1.001f,1.001f,1.001f);
    for(int j=1;j<stacks;j++){
      glBegin(GL_LINE_LOOP);
        float x,y,z;
        for(int i=0;i<slices;i++){
          x=sinf(float(j)*dangle2) * cosf(float(i)*dangle1);
          y=sinf(float(j)*dangle2) * sinf(float(i)*dangle1);
          z=cosf(float(j)*dangle2);
          glNormal3f(x,y,z);
          glVertex3f(radius*x,radius*y,radius*z);
        }
      glEnd();
    }    
    for(int i=0;i<slices;i++){
      glBegin(GL_LINE_STRIP);
        for(int j=0;j<stacks+1;j++){
          float x,y,z;
          x=sinf(float(j)*dangle2) * cosf(float(i)*dangle1);
          y=sinf(float(j)*dangle2) * sinf(float(i)*dangle1);
          z=cosf(float(j)*dangle2);
          glNormal3f(x,y,z);
          glVertex3f(radius* x,radius* y,radius* z);
        }
      glEnd();
    }    
  }
  glPopMatrix();
}

void GLTools::DrawCylinder(float radius, float height, int slices, int stacks){
  if((slices<2)||(stacks<1))
    return;
  
  glPushMatrix();  
  float dangle1 = 2.0f*PIf/float(slices);    
  float dheight = height/float(stacks);
  
  if(m_solid){
    glColor4f(m_red, m_green, m_blue, m_alpha);

    for(int j=0;j<stacks;j++){
      glBegin(GL_TRIANGLE_STRIP);
        float x,y,z;
        for(int i=0;i<slices+1;i++){
          x=cosf((float(i))*dangle1);
          y=sinf((float(i))*dangle1);
          glNormal3f(x,y,0.0f);
          x*=radius;
          y*=radius;
          z=dheight*float(j+1);
          glVertex3f(x,y,z);
          z=dheight*float(j);
          glVertex3f(x,y,z);
        }
      glEnd();
    }
  }
  if(m_outline){
    glColor3f(0.0f,0.0f,0.0f);
    glScalef(1.001f,1.001f,1.001f);
    for(int j=0;j<stacks+1;j++){
      glBegin(GL_LINE_LOOP);
        float x,y,z;
        for(int i=0;i<slices;i++){
          x=cosf(float(i)*dangle1);
          y=sinf(float(i)*dangle1);
          z=dheight*float(j);
          glNormal3f(x,y,0.0f);
          glVertex3f(x*radius,y*radius,z);
        }
      glEnd();
    }    
    for(int i=0;i<slices;i++){
      glBegin(GL_LINE_STRIP);
        for(int j=0;j<stacks+1;j++){
          float x,y,z;
          x=cosf(float(i)*dangle1);
          y=sinf(float(i)*dangle1);
          z=dheight*float(j);
          glNormal3f(x,y,0.0f);
          glVertex3f(x*radius,y*radius,z);
        }
      glEnd();
    }    
  }
  glPopMatrix();  
}

void GLTools::DrawFullCylinder(float radius, float height, int slices, int stacks){
  if((slices<2)||(stacks<1))
    return;
  
  glPushMatrix();  
  float dangle1 = 2.0f*PIf/float(slices);    
  float dheight = height/float(stacks);
  
  if(m_solid){
    glColor4f(m_red, m_green, m_blue, m_alpha);

    for(int j=0;j<stacks;j++){
      glBegin(GL_TRIANGLE_STRIP);
        float x,y,z;
        for(int i=0;i<slices+1;i++){
          x=cosf((float(i))*dangle1);
          y=sinf((float(i))*dangle1);
          glNormal3f(x,y,0.0f);
          x*=radius;
          y*=radius;
          z=dheight*float(j+1);
          glVertex3f(x,y,z);
          z=dheight*float(j);
          glVertex3f(x,y,z);
        }
      glEnd();
    }
    float x,y,z;
    glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0f,0.0f,-1.0f);
      glVertex3f(0.0f,0.0f,0.0f);
      for(int i=slices;i>=0;i--){
        x=cosf((float(i))*dangle1);
        y=sinf((float(i))*dangle1);        
        x*=radius;
        y*=radius;
        z=0.0f;
        glVertex3f(x,y,z);
      }
    glEnd();
    glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0f,0.0f,1.0f);
      glVertex3f(0.0f,0.0f,height);
      for(int i=0;i<slices+1;i++){
        x=cosf((float(i))*dangle1);
        y=sinf((float(i))*dangle1);        
        x*=radius;
        y*=radius;
        z=height;
        glVertex3f(x,y,z);
      }
    glEnd();
  }
  if(m_outline){
    glColor3f(0.0f,0.0f,0.0f);
    glScalef(1.001f,1.001f,1.001f);
    for(int j=0;j<stacks+1;j++){
      glBegin(GL_LINE_LOOP);
        float x,y,z;
        for(int i=0;i<slices;i++){
          x=cosf(float(i)*dangle1);
          y=sinf(float(i)*dangle1);
          z=dheight*float(j);
          glNormal3f(x,y,0.0f);
          glVertex3f(x*radius,y*radius,z);
        }
      glEnd();
    }    
    for(int i=0;i<slices;i++){
      glBegin(GL_LINE_STRIP);
        for(int j=0;j<stacks+1;j++){
          float x,y,z;
          x=cosf(float(i)*dangle1);
          y=sinf(float(i)*dangle1);
          z=dheight*float(j);
          glNormal3f(x,y,0.0f);
          glVertex3f(x*radius,y*radius,z);
        }
      glEnd();
    }    
  }
  glPopMatrix();  
}

void GLTools::DrawRef(float scale, const Matrix4 *ref){
  float r = m_red, g = m_green, b = m_blue;
  glPushMatrix();
  if(ref!=NULL){
    glMultMatrixf(ref->RowOrderForceFloat());
  }
    
  glScalef(scale,scale,scale);  
  SetColor(1,0,0);  
  DrawVector(Vector3(1,0,0));
  SetColor(0,1,0);  
  DrawVector(Vector3(0,1,0));
  SetColor(0,0,1);  
  DrawVector(Vector3(0,0,1));  
  SetColor(0.8f,0.8f,0.8f);
  DrawSphere(0.15f);
  SetColor(r,g,b);

  glPopMatrix();
}
