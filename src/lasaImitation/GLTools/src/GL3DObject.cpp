#include "GL3DObject.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
using namespace std;



#define GL3DOBJ_MAX_TRIANGLES   32768
#define GL3DOBJ_MAX_VERTICES    32768

float *GL3DObject::verData  = NULL;
int   *GL3DObject::triData  = NULL;
int    GL3DObject::objCount = 0;

GL3DObject::GL3DObject(){
  
  objCount++;
  if(verData==NULL) verData = new float[GL3DOBJ_MAX_VERTICES*3];     
  if(triData==NULL) triData = new   int[GL3DOBJ_MAX_TRIANGLES*3];  
  
  callListId =  0;
  nbVertices  = 0;
  nbTriangles = 0;
  
  triangles   = NULL;
  vertices    = NULL;
  normals     = NULL; 
  invNormals  = false; 
}

GL3DObject::~GL3DObject(){
  Free();
  
  objCount--;
  if(objCount<=0){
    delete [] verData;
    delete [] triData;
    verData = NULL;
    triData = NULL;
  }
      
  
}
   
void GL3DObject::Free(){
  if(triangles!=NULL) delete [] triangles; triangles = NULL;
  if(vertices!=NULL) delete [] vertices; vertices = NULL;
  if(normals!=NULL) delete [] normals; normals = NULL;
  if(callListId>0)
    glDeleteLists(callListId,1);    
}
   
int   GL3DObject::LoadFromObjFile(const char *filename, bool invNormals){
  
  this->invNormals = invNormals;
  
  ifstream ifile;
  ifile.open(filename);
  if(!ifile.is_open()){
    printf("Error while opening file <%s>\n",filename);
    return FALSE;    
  }
    
  char cline[512];
  char c;
  
  float *verPos = verData;
  int   *triPos = triData;
  
  nbVertices  = 0;
  nbTriangles = 0;
  
  ifile.getline(cline,512);
  while(!ifile.eof()){
    if(cline[0]=='v'){
      sscanf(cline,"%c %f %f %f",&c,verPos+0,verPos+1,verPos+2);
      verPos+=3;  
      nbVertices++;
            
    }else if (cline[0]=='f'){
      if(invNormals)
        sscanf(cline,"%c %d %d %d",&c,triPos+2,triPos+1,triPos+0);
      else
        sscanf(cline,"%c %d %d %d",&c,triPos+0,triPos+1,triPos+2);
        
      (*(triPos+0))--;
      (*(triPos+1))--;
      (*(triPos+2))--;
      triPos+=3;  
      nbTriangles++;      
    }
    
    ifile.getline(cline,512);    
  }  
  ifile.close();
  
  vertices  = new float[nbVertices*3];
  triangles = new int[nbTriangles*3];
  normals   = NULL;
  
  memcpy(vertices, verData,nbVertices*3*sizeof(float));
  memcpy(triangles,triData,nbTriangles*3*sizeof(int));
  
  
  
  return TRUE;
}

void GL3DObject::CalcNormals(){
  
  if(normals!=NULL)
    return;
  
  normals   = new float[nbTriangles*3];    
  
  int cnt = 0;
  float *v1,*v2,*v3;
  Vector3 a,b,n;
  for(int i=0;i<nbTriangles;i++){
    v1 = vertices+(triangles[cnt+0])*3;
    v2 = vertices+(triangles[cnt+1])*3;
    v3 = vertices+(triangles[cnt+2])*3;
    a.x() =  v2[0]-v1[0]; 
    a.y() =  v2[1]-v1[1]; 
    a.z() =  v2[2]-v1[2]; 
    b.x() =  v3[0]-v1[0]; 
    b.y() =  v3[1]-v1[1]; 
    b.z() =  v3[2]-v1[2];
    //if(invNormals)  
    b.Cross(a,n);
    //else            a.Cross(b,n);
    n.Normalize();
    normals[cnt+0] = n.x();
    normals[cnt+1] = n.y();
    normals[cnt+2] = n.z();
    cnt+=3;
  }  
}

void GL3DObject::AddOffset(Vector3& offset){
  float *ver = vertices;
  for(int i=0;i<nbVertices;i++){
    *(ver++)+=offset.cx();
    *(ver++)+=offset.cy();
    *(ver++)+=offset.cz();
  }
}

void GL3DObject::Transform(Matrix3& trans){
  float *ver = vertices;
  Vector3 vec,vec2;
  for(int i=0;i<nbVertices;i++){
    vec.x() = *(ver+0);
    vec.y() = *(ver+1);
    vec.z() = *(ver+2);
    trans.Mult(vec,vec2); 
    *(ver++)=vec2.x();
    *(ver++)=vec2.y();
    *(ver++)=vec2.z();
  }  
}

int GL3DObject::BuildDisplayList(){
  if(callListId>0)
    return TRUE;
    
  CalcNormals();
    
  callListId = glGenLists(1);
    
//  printf("GLError Gen: %d\n",glGetError());
  if(callListId>0)
    glNewList(callListId,GL_COMPILE);
//  printf("GLError New: %d\n",glGetError());
  
  glBegin(GL_TRIANGLES);
  int cnt = 0;
  float *v;
  for(int i=0;i<nbTriangles;i++){
    v = normals+i*3;
    glNormal3f(*v,*(v+1),*(v+2));
    v = vertices+(triangles[cnt+0])*3;
    glVertex3f(*v,*(v+1),*(v+2));
    v = vertices+(triangles[cnt+2])*3;
    glVertex3f(*v,*(v+1),*(v+2));
    v = vertices+(triangles[cnt+1])*3;
    glVertex3f(*v,*(v+1),*(v+2));
    cnt+=3;  
  }
  glEnd();
  
  if(callListId>0)    
    glEndList();
//  printf("GLError End: %d\n",glGetError());
//  printf("ISLIST : %d\n",glIsList(callListId));
  
  if(callListId>0)
    return TRUE;
  
  return FALSE; 
}

void  GL3DObject::Render(){
  if(BuildDisplayList()){
    glCallList(callListId);
  }
  /*glBegin(GL_TRIANGLES);
  glColor3f(1.0f,1.0f,0.0f);
  int cnt = 0;
  float *v;
  for(int i=0;i<nbTriangles;i++){
    v = normals+i*3;
    glNormal3f(*v,*(v+1),*(v+2));
    v = vertices+(triangles[cnt+0])*3;
    glVertex3f(*v,*(v+1),*(v+2));
    v = vertices+(triangles[cnt+2])*3;
    glVertex3f(*v,*(v+1),*(v+2));
    v = vertices+(triangles[cnt+1])*3;
    glVertex3f(*v,*(v+1),*(v+2));
    cnt+=3;  
  }
  glEnd();
*/}
  