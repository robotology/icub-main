/*
 * mesh.h
 */

#ifndef ICUBMESH_H
#define ICUBMESH_H

#include <qstring.h>
#include <qmessagebox.h>
#include <qfile.h>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

inline QStringList tokenize(QString in,char sep)
{
    QStringList out;
    while (in.length())
    {
        int pos=in.find(sep);
        if (pos==-1)
        { 
            out.append(in);
            return out;
        }
        if (pos) out.append(in.left(pos));
        in.remove(0,pos+1);
    }
    return out;    
}

class iCubMesh
{
public:
    iCubMesh(QString fileName,double rz=0.0,double ry=0.0,double rx=0.0,double tx=0.0,double ty=0.0,double tz=0.0)
    {        
        nFaces=0;
        
        QFile objFile(fileName);
        if(!objFile.open(IO_ReadOnly))
        {
            QMessageBox::critical(0,QObject::tr("Missing Obj File"),
                                  QObject::tr("<qt>Obj file not found at:<br>%1</qt>").arg(fileName));
            return;
        }
        
        int v=0,n=0;

        char buffer[4096];
        
        while(!objFile.atEnd())
        {
            objFile.readLine(buffer,4096);
            QString line(buffer);
            QStringList parameters=tokenize(line.simplifyWhiteSpace(),' ');
            
            if (parameters[0]=="v")       ++v;
            else if (parameters[0]=="vn") ++n;
            else if (parameters[0]=="f")  ++nFaces;   
        }
        objFile.close();
        
        vx=new float[v]; vy=new float[v]; vz=new float[v];
        nx=new float[n]; ny=new float[n]; nz=new float[n];        
        
        av=new short[nFaces]; bv=new short[nFaces]; cv=new short[nFaces];
        an=new short[nFaces]; bn=new short[nFaces]; cn=new short[nFaces];
                 
        double grad2rad=M_PI/180.0;
        rz*=grad2rad;
        ry*=grad2rad;
        rx*=grad2rad; 
        double cz=cos(rz),sz=sin(rz);
        double cy=cos(ry),sy=sin(ry);         
        double cx=cos(rx),sx=sin(rx);         
                 
        nFaces=v=n=0;
        QStringList num;
        objFile.open(IO_ReadOnly);
        
        double x1,y1,z1;
        double x2,y2,z2;
        double x3,y3,z3;
        
        while(!objFile.atEnd())
        {
            objFile.readLine(buffer,4096);
            QString line(buffer);
            QStringList parameters=tokenize(line.simplifyWhiteSpace(),' ');
            
            if (parameters[0]=="v")
            {
                x1=parameters[1].toDouble()+tx;
                y1=parameters[2].toDouble()+ty;
                z1=parameters[3].toDouble()+tz;
            
                x2=cz*x1-sz*y1;
                y2=sz*x1+cz*y1;
                z2=z1;
            
                x3= cy*x2+sy*z2;    
                y3= y2;
                z3=-sy*x2+cy*z2;
                
                vx[v]=x3;
                vy[v]=cx*y3-sx*z3;
                vz[v]=sx*y3+cx*z3;
                
                ++v;
            }
            else if (parameters[0]=="vn")
            {
                x1=(cz*parameters[1].toDouble()-sz*parameters[2].toDouble());
                y1=(sz*parameters[1].toDouble()+cz*parameters[2].toDouble());
                z1=parameters[3].toDouble();
            
                x2= cy*x1+sy*z1;    
                y2=y1;
                z2=-sy*x1+cy*z1;
                
                nx[n]=x2;
                ny[n]=cx*y2-sx*z2;
                nz[n]=sx*y2+cx*z2;
                
                ++n;
            }
            else if (parameters[0]=="f")
            {
                num=tokenize(parameters[1].simplifyWhiteSpace(),'/');
                av[nFaces]=num[0].toShort()-1;
                an[nFaces]=num[1].toShort()-1;
                
                num=tokenize(parameters[2].simplifyWhiteSpace(),'/');
                bv[nFaces]=num[0].toShort()-1;
                bn[nFaces]=num[1].toShort()-1;                
                
                num=tokenize(parameters[3].simplifyWhiteSpace(),'/');
                cv[nFaces]=num[0].toShort()-1;
                cn[nFaces]=num[1].toShort()-1;
                                              
                ++nFaces; 
            }   
        }
        objFile.close();
    }

    ~iCubMesh()
    {
        delete [] vx; delete [] vy; delete [] vz;
        delete [] nx; delete [] ny; delete [] nz;
        delete [] av; delete [] bv; delete [] cv;
        delete [] an; delete [] bn; delete [] cn;        
    }

    void Draw()
    {
        short v,n;
        glBegin(GL_TRIANGLES);
        for(int f=0; f<nFaces; ++f)
        {
            v=av[f]; n=an[f];
            glNormal3f(nx[n],ny[n],nz[n]);
            glVertex3f(vx[v],vy[v],vz[v]);
            
            v=bv[f]; n=bn[f];
            glNormal3f(nx[n],ny[n],nz[n]);
            glVertex3f(vx[v],vy[v],vz[v]);
            
            v=cv[f]; n=cn[f];
            glNormal3f(nx[n],ny[n],nz[n]);
            glVertex3f(vx[v],vy[v],vz[v]);
        }
        glEnd();
    }
    
protected:
    int nFaces;
    float *vx,*vy,*vz;
    float *nx,*ny,*nz;
    short *av,*bv,*cv;
    short *an,*bn,*cn;
};

#endif
