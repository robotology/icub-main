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
#include "Shape.h"
#include "Rotation.h"


#define FRANCIS_DIST

Shape::Shape(){
  v_clear(position);
  v_clear(orientation);
  highlight=0;
}


float Shape::Distance(Shape& sh, RigidTransfo& transfo){
  CVector3_t pos,pos2;
  sh.GetPosition(pos);
  transfo.Transform(pos,pos2);
  v_sub(position,pos2,pos);
  return v_length(pos);
}


// void Capsule::SetRadius(float r){
//   radius = r;
// }

 Capsule::Capsule(){
  radius =10;
  v_clear(axis);
  cyl = gluNewQuadric();
}

Capsule::Capsule(pTree config):Shape(){
  //  Capsule();
  unsigned int i;
  float f;
  CVector3_t v;
  cyl = gluNewQuadric();
  Tree_List *subTrees = config->GetSubTrees();
  for(i=0;i<(*subTrees).size();i++){
     if((*subTrees)[i]->GetName().compare("Axis")==0){
       istringstream s((*subTrees)[i]->GetData());
       s >> v[0]>>v[1]>>v[2];
       SetAxis(v);
       cout<<"setting axis "<<v<<endl;
     }
      if((*subTrees)[i]->GetName().compare("Radius")==0){
	istringstream s((*subTrees)[i]->GetData());
	s >> f;
	SetRadius(f);
       cout<<"setting radius "<<f<<endl;
    }
     if((*subTrees)[i]->GetName().compare("Position")==0){
       istringstream s((*subTrees)[i]->GetData());
       s >> v[0]>>v[1]>>v[2];
       SetPosition(v);
       cout<<"setting position "<<v<<endl;
     }
  }
}

Capsule::~Capsule(){
  gluDeleteQuadric(cyl);
}

void Capsule::Stream(ostream& out) const{
  out<<"<Type> Capsule </Type>"<<endl;
  out<<"<Params>"<<endl;
  out<<"<Radius> "<<radius<<" </Radius>"<<endl; 
  out<<"<Axis> "<< axis <<" </Axis>"<<endl;
  out<<"<Position> "<<position<<" </Position>"<<endl;
  out<<"</Params>"<<endl;
}

void Capsule::Render(){
  Rotation rot;
  CVector3_t v,z; //z axis
  CMatrix4_t mat;
  float length,angle;
  length = v_length(axis);
  if(length>epsilon){
    v_set(0,0,1,z);
    v_cross(z,axis,v);
    
    if(!rot.SetRotationAxis(v)){
      m_identity(mat);
    }
    else{
      angle = acos(v_dot(z,axis)/length);
      rot.SetAngle(angle);
      rot.RotToMatrix(mat);
    }
    if(highlight){
      glColor3f(1.0,0.6,0.4);
    }
    else{
      glColor3f(1.0,0.8,0.7);
    }
    glPushMatrix();
    glTranslatef(position[0],position[1],position[2]);
    glMultMatrixf(mat);
    gluCylinder(cyl,radius,radius,length,8,5);
  //  glutSolidCylinder(radius,length, 10,10);
  }
  glColor3f(1.0,0,0);
  glutSolidSphere(radius,10,10);
  glPushMatrix();
  glTranslatef(0,0,length);
  glutSolidSphere(radius,10,10);
  glPopMatrix();
  if(length>epsilon){
    glPopMatrix();
  }
  //  Unhighlight();
}


void Capsule::Update(CVector3_t *pos,int nbpos){
    if(nbpos==1){
        v_copy(pos[0],axis);
    }
}

/*
float Capsule::Distance(Capsule& cap, RigidTransfo& transfo){ 
    
    CVector3_t vec1, vec2, p1g1,p1g2,p2g1,p2g2;
    CVector3_t point[4];
    float k1,k2,dist,radius2,min;
    
    cap.GetParam(p1g2,vec2,radius2); 
    
        //conversion de referentiel
    transfo.InverseTransform(position,p1g1);
    transfo.Rotation::InverseTransform(axis,vec1);
  
    v_add(p1g2,vec2,p2g2);
    v_add(p1g1,vec1,p2g1);
    
    if (segments_nearest_points(p1g1, vec1, p1g2, vec2, &k1 , &k2, &dist)==1){
      return (dist-radius-radius2);
    }
    else{      
      v_sub(p1g2,p1g1,point[0]);
      v_sub(p1g2,p2g1,point[1]);
      v_sub(p2g2,p1g1,point[2]);  
      v_sub(p2g2,p2g1,point[3]);
      //for (int i=0;i<4;i++)
      //cout<<"Distance"<<i<<"=  "<<v_length(point[i])<<endl;
      min=MAX_FLT;
      for(int i=0;i<4;i++){
	if (v_length(point[i])<min){
	  min=v_length(point[i]);
	}
      }
      dist=min-radius-radius2;
      return dist;
    }
}

*/


void Capsule::GetParam(CVector3_t point, CVector3_t dir, float& radius_cap){
  v_copy(position,point);
  v_copy(axis,dir);
  radius_cap=radius;
}

float Capsule::Distance(Capsule& cap, RigidTransfo& transfo){ 
    
    CVector3_t vec1, vec2;
    CVector3_t point[4]; // p1,p2 : gelule 1 , p2,p3 :gelule 2
    float k1,k2,dist,radius2,min_dist=FLT_MAX;
    

    
    radius2 = cap.GetRadius();
    cap.GetPosition(point[2]);
    cap.GetAxis(vec2);

        //conversion de referentiel
    transfo.InverseTransform(position,point[0]);
    transfo.Rotation::InverseTransform(axis,vec1);
  
    v_add(point[2],vec2,point[3]);
    v_add(point[0],vec1,point[1]);
    
    if (segments_nearest_points(point[0], vec1, point[2], vec2, &k1 , &k2, &dist)==1){
      return (dist-radius-radius2);
    }
    else{
      for(int i=0;i<2;i++){
	for(int j=0;j<2;j++){
	  v_sub(point[i],point[2+j],vec1);
	  dist = v_squ_length(vec1);
	  if(dist<min_dist){
	    min_dist=dist;
	  }
	}
      }
      return sqrt(dist)-radius-radius2;
    }     
}





float Capsule::Distance(Shape& sh, RigidTransfo& transfo){
  
  Shape *psh;
  Capsule *cap;
  Sphere *sp;
  Parallelipiped *para;
 
  psh = &sh;
  
  cap = dynamic_cast<Capsule *>(psh);
  if(cap){
    return Distance(*cap,transfo);
  }
  sp = dynamic_cast<Sphere *>(psh);
  if(sp){
    return Distance(*sp,transfo);
  }
  para = dynamic_cast<Parallelipiped *>(psh);
  if(para){
    return Distance(*para,transfo);
  }
  
  return 0;
}

/*
float Capsule::Distance(Sphere& sph, RigidTransfo& transfo){

	CVector3_t intersection, vec_dir, new_vec, point1, point2, centre;
	float norme_carre, prod_vec, multi, k=0, rayon_gelule, rayon_sphere;


	sph.GetParam(centre,rayon_sphere);

	v_copy(position,point1);
	v_add(point1,axis,point2);
	rayon_gelule=radius;	 

	v_copy(axis,vec_dir); //axis=point2[j]-point1[j];
	
	
	//conversion de referentiel
	if (sph.GetNumshape() == 1){
	  transfo.InverseTransform(centre,centre);
	}
	else{
	  transfo.Transform(centre,centre);
	}

	norme_carre= pow(vec_dir[0],2)+pow(vec_dir[1],2)+pow(vec_dir[2],2);
	prod_vec=(centre[0]-point1[0])*vec_dir[0]+(centre[1]-point1[1])*vec_dir[1]+(centre[2]-point1[2])*vec_dir[2];

	multi=prod_vec/norme_carre;

	for (int l=0;l<3;l++){
		intersection[l]=point1[l]+multi*vec_dir[l];
		new_vec[l]=intersection[l]-point1[l];
		if (new_vec[l]!=0. && vec_dir[l]!=0.)
			k=new_vec[l]/vec_dir[l];
	}


	if (k<0.)
		return (sqrt(pow(point1[0]-centre[0],2)+pow(point1[1]-centre[1],2)+pow(point1[2]-centre[2],2))-rayon_gelule-rayon_sphere);
	else{
		if(k>1.)
			return (sqrt(pow(point2[0]-centre[0],2)+pow(point2[1]-centre[1],2)+pow(point2[2]-centre[2],2))-rayon_gelule-rayon_sphere);
		else
			return (sqrt(pow(intersection[0]-centre[0],2)+pow(intersection[1]-centre[1],2)+pow(intersection[2]-centre[2],2))-rayon_gelule-rayon_sphere);

	}
}

*/


float Capsule::Distance(Sphere& sph, RigidTransfo& transfo){

  CVector3_t centre,centreR1,naxis;
  float f,f1,f2,srad;

  sph.GetPosition(centre);
  srad = sph.GetRadius();
  transfo.Transform(centre,centreR1);
  v_sub(centreR1,position,centreR1);
  f1 = v_normalize(axis,naxis);
  f2 = v_dot(centreR1,naxis);
  f =min(f1,max(0,f2)); 
  v_scale(naxis,f,naxis);
  v_sub(centreR1,naxis,centre);
  return v_length(centre)-radius-srad;
}	




float Capsule::Distance(Parallelipiped& par, RigidTransfo& transfo){
  RigidTransfo rt(transfo);
  rt.Invert();
  return par.Distance(*this,rt);
}


Sphere::Sphere(pTree config):Shape(){
  //  Capsule();
  unsigned int i;
  float f;
  CVector3_t v;
  Tree_List *subTrees = config->GetSubTrees();
  for(i=0;i<(*subTrees).size();i++){
    if((*subTrees)[i]->GetName().compare("Radius")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> f;
      SetRadius(f);
    }
    if((*subTrees)[i]->GetName().compare("Position")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> v[0]>>v[1]>>v[2];
      SetPosition(v);
    }
  }
}



void Sphere::Render(){
  glPushMatrix();
  glTranslatef(position[0],position[1],position[2]);
  if(highlight){
    glColor3f(0.8,0,0.8);
  }
  else{
    glColor3f(1.0,0,1.0);
  }
 glutSolidSphere(radius,15,15); 
  glPopMatrix();
  //  Unhighlight();
}


void Sphere::Stream(ostream& out) const{
  out<<"<Type> Sphere </Type>"<<endl;
  out<<"<Params>"<<endl;
  out<<"<Radius> "<<radius<<" </Radius>"<<endl;
  out<<"<Position> "<<position<<" </Position>"<<endl;
   out<<"</Params>"<<endl;
}




float Sphere::Distance(Shape& sh, RigidTransfo& transfo){
  
  Shape *psh;
  Capsule *cap;
  Sphere *sp;
  Parallelipiped *para;
 
  psh = &sh;
  
  cap = dynamic_cast<Capsule *>(psh);
  if(cap){
    return Distance(*cap, transfo);
  }
  sp = dynamic_cast<Sphere *>(psh);
  if(sp){
    return Distance(*sp, transfo);
  }
  para = dynamic_cast<Parallelipiped *>(psh);
  if(para){
    return Distance(*para, transfo);
  }
  
  return 0;
}


void Sphere::GetParam(CVector3_t center_sph, float& radius_sph){
  v_copy(position,center_sph);
  radius_sph=radius;
}

float Sphere::Distance(Sphere& sph, RigidTransfo& transfo){
  CVector3_t v1,v2;
  float r;
  sph.GetPosition(v1);
  r=sph.GetRadius();

  //conversion de referentiel
  transfo.Transform(v1,v2);	    	  
  v_sub(position,v2,v1);
  return v_length(v1)-radius-r;
}   

float Sphere::Distance(Capsule& cap, RigidTransfo& transfo){
  RigidTransfo rt(transfo);
  rt.Invert();
  return cap.Distance(*this,rt);
}

float Sphere::Distance(Parallelipiped& para, RigidTransfo& transfo){
  RigidTransfo rt(transfo);
  rt.Invert(); 
  return para.Distance(*this,rt);
}
Parallelipiped::Parallelipiped():Shape(){
  v_clear(size);
}

Parallelipiped::Parallelipiped(pTree config):Shape(){
  CVector3_t v;
  Tree_List *subTrees = config->GetSubTrees();
  for(unsigned int i=0;i<(*subTrees).size();i++){
    if((*subTrees)[i]->GetName().compare("Size")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> v[0]>>v[1]>>v[2];
      SetSize(v);
    }
    if((*subTrees)[i]->GetName().compare("Position")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> v[0]>>v[1]>>v[2];
      SetPosition(v);
    }
  }

}

void Parallelipiped::Stream(ostream& out) const{
 out<<"<Type> Parallelipiped </Type>"<<endl;
  out<<"<Params>"<<endl;
  out<<"<Size> "<<size<<" </Size>"<<endl; 
  out<<"<Position> "<<position<<" </Position>"<<endl;
  out<<"</Params>"<<endl;
}


void Parallelipiped::Render(){
  Rotation rot;
  CMatrix4_t mat;
  rot.SetRotationParam(orientation);
  rot.RotToMatrix(mat);
  glPushMatrix();

 glTranslatef(position[0],position[1],position[2]);
 glMultMatrixf(mat);//new
 glTranslatef(0.5*size[0],0.5*size[1],0.5*size[2]);
 glScalef(size[0],size[1],size[2]);
 if(highlight){
   glColor3f(0.5,0.7,0.9);
 }
 else{
   glColor3f(0.8,0.9,1.0);
 }
 glutSolidCube(1.0);
 glColor3f(0,0,0);
 glScalef(1.01,1.01,1.01);
 glLineWidth(3.0);
 glutWireCube(1.0);
 glColor3f(1.0,0.8,0.7);
 glPopMatrix();
 // Unhighlight();
}

float Parallelipiped::Distance_point_face(const CVector3_t point, int plan, float longueur, float largeur, float hauteur){

	int i,j;
	float pos_plan[6];

	for (i=0;i<3;i++)
		pos_plan[2*i]=0.0;

	pos_plan[1]=hauteur;
	pos_plan[3]=longueur;
	pos_plan[5]=largeur;

	j=plan/2;

	if (j==0){
		if (point[0]<=longueur && point[0]>=0.0){
			if (point[1]<=largeur && point[1]>=0.0)
				return (abs(point[2]-pos_plan[plan]));
			else{
				if (point[1]<0.0)
					return (sqrt(pow(point[1],2)+pow(point[2]-pos_plan[plan],2)));
				else
					return (sqrt(pow(point[1]-largeur,2)+pow(point[2]-pos_plan[plan],2)));
			}
		}
		else{
			if (point[0]<0.0){
				if (point[1]>largeur)
					return (sqrt(pow(point[0],2)+pow(point[1]-largeur,2)+pow(point[2]-pos_plan[plan],2)));
				else{
					if (point[1]<0.0)
						return (sqrt(pow(point[0],2)+pow(point[1],2)+pow(point[2]-pos_plan[plan],2)));
					else
						return (sqrt(pow(point[0],2)+pow(point[2]-pos_plan[plan],2)));
				}
			}
			else{
				if (point[1]>largeur)
					return (sqrt(pow(point[0]-longueur,2)+pow(point[1]-largeur,2)+pow(point[2]-pos_plan[plan],2)));
				else{
					if (point[1]<0.0)
						return (sqrt(pow(point[0]-longueur,2)+pow(point[1],2)+pow(point[2]-pos_plan[plan],2)));
					else
						return (sqrt(pow(point[0]-longueur,2)+pow(point[2]-pos_plan[plan],2)));
				}
			}
		}
	}
	else{	
		if (j==1){
			if (point[1]<=largeur && point[1]>=0.0){
				if (point[2]<=hauteur && point[2]>=0.0)
					return (abs(point[0]-pos_plan[plan]));
				else{
					if (point[2]<0.0)
						return (sqrt(pow(point[0]-pos_plan[plan],2)+pow(point[2],2)));
					else
						return (sqrt(pow(point[0]-pos_plan[plan],2)+pow(point[2]-hauteur,2)));
				}
			}
			else{
				if (point[1]<0.0){
					if (point[2]>hauteur)
						return (sqrt(pow(point[0]-pos_plan[plan],2)+pow(point[1],2)+pow(point[2]-hauteur,2)));
					else{
						if (point[2]<0.0)
							return (sqrt(pow(point[0]-pos_plan[plan],2)+pow(point[1],2)+pow(point[2],2)));
						else
							return (sqrt(pow(point[0]-pos_plan[plan],2)+pow(point[1],2)));
					}
				}
				else{
					if (point[2]>hauteur)
						return (sqrt(pow(point[0]-pos_plan[plan],2)+pow(point[1]-largeur,2)+pow(point[2]-hauteur,2)));
					else{
						if (point[2]<0.0)
							return (sqrt(pow(point[0]-pos_plan[plan],2)+pow(point[1]-largeur,2)+pow(point[2],2)));
						else
							return (sqrt(pow(point[0]-pos_plan[plan],2)+pow(point[1]-largeur,2)));
					}
				}
			}
		}
		else{
			if (point[0]<=longueur && point[0]>=0.0){
				if (point[2]<=hauteur && point[2]>=0.0)
					return (abs(point[1]-pos_plan[plan]));
				else{
					if (point[2]<0.0)
						return (sqrt(pow(point[1]-pos_plan[plan],2)+pow(point[2],2)));
					else
						return (sqrt(pow(point[1]-pos_plan[plan],2)+pow(point[2]-hauteur,2)));
				}
			}
			else{
				if (point[0]<0.0){
					if (point[2]>hauteur)
						return (sqrt(pow(point[0],2)+pow(point[1]-pos_plan[plan],2)+pow(point[2]-hauteur,2)));
					else{
						if (point[2]<0.0)
							return (sqrt(pow(point[0],2)+pow(point[1]-pos_plan[plan],2)+pow(point[2],2)));
						else
							return (sqrt(pow(point[0],2)+pow(point[1]-pos_plan[plan],2)));
					}
				}
				else{
					if (point[2]>hauteur)
						return (sqrt(pow(point[0]-longueur,2)+pow(point[1]-pos_plan[plan],2)+pow(point[2]-hauteur,2)));
					else{
						if (point[2]<0.0)
							return (sqrt(pow(point[0]-longueur,2)+pow(point[1]-pos_plan[plan],2)+pow(point[2],2)));
						else{
							return (sqrt(pow(point[0]-longueur,2)+pow(point[1]-pos_plan[plan],2)));
						}
					}
				}
			}
		}
	}
}

void Parallelipiped::Point_proche_plan(const CVector3_t point1, const CVector3_t point2, int plan, float longueur, float largeur, float hauteur, CVector3_t out){
	
	float dist1=0, dist2=0, pos_plan[6];
	int i,j;

	for (i=0;i<3;i++)
		pos_plan[2*i]=0.0;

	pos_plan[1]=hauteur;
	pos_plan[3]=longueur;
	pos_plan[5]=largeur;

	j=(plan/2);

	if (j==0){
		dist1=abs(point1[2]-pos_plan[plan]);
		dist2=abs(point2[2]-pos_plan[plan]);
	}
	if (j==1){
		dist1=abs(point1[0]-pos_plan[plan]);
		dist2=abs(point2[0]-pos_plan[plan]);
	}
	if (j==2){
		dist1=abs(point1[1]-pos_plan[plan]);
		dist2=abs(point2[1]-pos_plan[plan]);
	}

	if (dist1<dist2)
		for (i=0;i<3;i++)  
			out[i]=point1[i];
	else{
		for (i=0;i<3;i++)
			out[i]=point2[i];
	}
}

int Parallelipiped::Inter_droite_plan (const CVector3_t point1, const CVector3_t point2, int plan, float dist, CVector3_t out){ 
     
    CVector3_t droite;
    float lambda=0;

    v_sub(point2,point1,droite);
    /*droite[0]=point2[0]-point1[0];
    droite[1]=point2[1]-point1[1];
    droite[2]=point2[2]-point1[2];
    cout<<"vec dir= "<<droite<<endl;*/

	switch(plan/2){
	case 0: if (abs(droite[2])<=0.01){
	           return 0;
	        }
	        out[2]=dist;
		lambda=(out[2]-point1[2])/droite[2];
		break;
	case 1: if (abs(droite[0])<=0.01){
	           return 0;
	        }
	        out[0]=dist;
		lambda=(out[0]-point1[0])/droite[0];
		break;
	case 2:	if (abs(droite[1])<=0.01){
	           return 0;
		  
	        }
	        out[1]=dist;
		lambda=(out[1]-point1[1])/droite[1];
		break;
	}

	
	out[0]=point1[0]+lambda*droite[0];
	out[1]=point1[1]+lambda*droite[1];
	out[2]=point1[2]+lambda*droite[2];


	
	//cout << "lambda = " << lambda << endl;
	return 1;
}







float Parallelipiped::Distance(Sphere& sph, RigidTransfo& transfo){
	  
      CVector3_t para, sphere ;
      float hauteur, longueur, largeur, rayon;    
      
      //cout<<"Paralllelipiped Sphere"<<endl;
      sph.GetParam(sphere,rayon);
      //for (int k=0;k<3;k++)
      // para[k]=position[k];
      
      //conversion de referentiel
   //    if(sph.GetNumshape() == 1){
// 	transfo.InverseTransform(sphere,sphere);
//       }
//       else{
	transfo.Transform(sphere,sphere);
	//      }
      //
	
      
      
      hauteur=size[0];
      longueur=size[1];
      largeur=size[2];

      para[0]=position[0]+largeur/2.;
      para[1]=position[1]+hauteur/2.;
      para[2]=position[2]+longueur/2.;
    
      //cout <<"Longueur = "<<longueur<< "  largeur = " <<largeur<<"  hauteur= "<<hauteur<<endl;
      //cout << "Position de la sphere"<<endl;
      //cout << sphere[0]<<" "<<sphere[1]<<" "<<sphere[2]<<endl;
      //cout << "rayon = " <<rayon<<endl;
      
      float dist[3];
      
      
      //Compute distances to planes
      
      if (sphere[0]>=para[0])
          dist[0]= abs(sphere[0]-para[0]-longueur/2.);             
      else
          dist[0]= abs(sphere[0]-para[0]+longueur/2.);
          
      if (sphere[1]>=para[1])
          dist[1]= abs(sphere[1]-para[1]-largeur/2.);             
      else
          dist[1]= abs(sphere[1]-para[1]+largeur/2.);
             
      if (sphere[2]>=para[2])
          dist[2]= abs(sphere[2]-para[2]-hauteur/2.);             
      else
          dist[2]= abs(sphere[2]-para[2]+hauteur/2.);       
      
      //cout<<"Distance aux plans "<<endl;
      //cout<<dist[0]<<" "<<dist[1]<<" "<<dist[2]<<endl;

       //Compute minimum distance
       
     if (sphere[0]<(para[0]-longueur/2.) || sphere[0]>(para[0]+longueur/2.)){
            if(sphere[1]<(para[1]-largeur/2.) || sphere[1]>(para[1]+largeur/2.)){
                 if (sphere[2]<(para[2]-hauteur/2.) || sphere[2]>(para[2]+hauteur/2.))
                       return (sqrt(pow(dist[0],2)+pow(dist[1],2)+pow(dist[2],2)) - rayon);
                 else
                       return (sqrt(pow(dist[0],2)+pow(dist[1],2)) - rayon);
            }
            else{
                 if (sphere[2]<(para[2]-hauteur/2.) || sphere[2]>(para[2]+hauteur/2.))
                       return (sqrt(pow(dist[0],2)+pow(dist[2],2)) - rayon); 
                 else
                       return (dist[0]-rayon);
            }
      }
      else{
            if(sphere[1]<(para[1]-largeur/2.) || sphere[1]>(para[1]+largeur/2.)){
                 if (sphere[2]<(para[2]-hauteur/2.) || sphere[2]>(para[2]+hauteur/2.)) 
                      return (sqrt(pow(dist[1],2)+pow(dist[2],2)) - rayon); 
                 else
                      return (dist[1]-rayon);
            }  
            else
                 return (dist[2]-rayon);
      }
}

float Parallelipiped::Distance(Shape& sh, RigidTransfo& transfo){
  
  Shape *psh;
  Capsule *cap;
  Sphere *sp;
 
  psh = &sh;
  
  cap = dynamic_cast<Capsule *>(psh);
  if(cap){
    return Distance(*cap, transfo);
  }
  sp = dynamic_cast<Sphere *>(psh);
  if(sp){
    return Distance(*sp, transfo);
  }
  
  return 100;
}


#ifdef FRANCIS_DIST
float Parallelipiped::Distance(Capsule& cap, RigidTransfo& transfo){
   
	int faces[6], arretes[12], index[6], i,autorized=0;
	CVector3_t intersection, point,vec_dir_seg, vec_dir_edge, point_edge, point1, point2, axis;
	float distance[6], pos_plan[6], min=0.0, k1, k2, distance_edge[12], pos, longueur, largeur, hauteur, rayon;
	//struct timeval first,second;
	//static int k=0;
	
	//if (k==0){
	//gettimeofday(&first,NULL);
	//}
	//k++;
	//cout<<"Parallelipiped Capsule"<<endl;
	cap.GetParam(point1,axis,rayon);
       	

// 	cout<<"refpos=  "<<position<<endl;
// 	cout<<"capspos=  "<<point1<<endl
	//conversion de referentiel
	
// 	if (cap.GetNumshape() == 1){
// 	  transfo.InverseTransform(point1,point1);
// 	  transfo.Rotation::InverseTransform(axis,axis);
// 	}
// 	else{
	  transfo.Transform(point1,point1);
	  transfo.Rotation::Transform(axis,axis);
	  //	}

// 	v_copy(point1,tmp);
// 	point1[0]=tmp[2];
// 	point1[1]=tmp[0];
// 	point1[2]=tmp[1];
// 	v_copy(axis,tmp);
// 	axis[0]=tmp[2];
// 	axis[1]=tmp[0];
// 	axis[2]=tmp[1];
// 	longueur=size[2];
//         largeur=size[0];
//         hauteur=size[1];
	//
	
	
	//cout<<"capspos apres transfo=  "<<point1<<endl;
	//cout<<"axespos apres transfo=  "<<axis<<endl;
	

	v_sub(point1,position,point1);
	v_add(axis,point1,point2);
		
	longueur=size[0];
        largeur=size[1];
        hauteur=size[2];
	
	//	cout<<longueur<<" "<<largeur<<" "<<hauteur<<endl;
	v_copy(axis,vec_dir_seg);
	
     
	for (i=0;i<6;i++){
		faces[i]=1;
		distance[i]=0.0;
	}
	for (i=0;i<12;i++){
		arretes[i]=0;
		distance_edge[i]=0.0;
	}
	for (i=0;i<3;i++)
		pos_plan[2*i]=0.0;

	pos_plan[1]=longueur;
	pos_plan[3]=largeur;
	pos_plan[5]=hauteur;
         
	// Control if we can suppress sides
	//cout<<"Suppress  ";
	if (point1[0]>0.0 && point2[0]>0.0){
	  faces[2]=0;//cout<<"faces[2]  "<<endl;
	}
	if (point1[1]>0.0 && point2[1]>0.0){
	  faces[4]=0;//cout<<"faces[4]  ";
	}
	if (point1[2]>0.0 && point2[2]>0.0){
	  faces[0]=0;//cout<<"faces[0]  ";
	}
	if (point1[0]<0.0 && point2[0]<0.0){
	  faces[3]=0;//cout<<"faces[3]  ";
	}
	if (point1[1]<0.0 && point2[1]<0.0){
	  faces[5]=0;//cout<<"faces[5]  ";
	}
	if (point1[2]<0.0 && point2[2]<0.0){
	  faces[1]=0;//cout<<"faces[1]  ";
	}
	//	cout<<endl;
		
	// Control if the intersection is in a plane
     
	for (i=0;i<6;i++){
		autorized=0;
		if (faces[i]==1){
			switch(i){
				case 1: pos=hauteur;
						break;
				case 3: pos=longueur;
						break;
				case 5: pos=largeur;
						break;
				default:pos=0.0;
						break;
			}
			if (Inter_droite_plan ( point1, point2, i, pos, intersection)==0){
			  
			  //	cout << "Pas d'intersection sur face " <<i <<" !!!"<< endl;
			  //	cout << endl;
				autorized=1;
			}
			

			
			if (autorized!=1){
			  //cout<<"point1= "<<point1<<" point2= "<<point2<<" i= "<<i<<" pos= "<<pos<<endl;
			  //cout << "face " << i << endl;
			  //cout << "intersection = " << intersection[0] <<"      "<< intersection[1] <<"         "<< intersection[2] << endl;
			  //cout << endl;
			  if (intersection[(i/2)]>pos_plan[2*(i/2)+1]){
					arretes[2*i]=1;
					//cout<<"1"<<endl;
					if (i>3)
						faces[0]=0;
					else
						faces[2+2*(i/2)]=0;
				}
			  else{
			       	if (intersection[(i/2)]<0.){
						arretes[(2*i)+1]=1;
						//	cout<<"2"<<endl;
						if (i>3)
							faces[1]=0;
						else
							faces[2*(i/2)+3]=0;
					}
			       	else{
			       		if (i>3){
					  //cout<<"3"<<endl;
			       			if (intersection[0]>=0.0 && intersection[0]<= pos_plan[1])
								index[i]=1;
			       			else{
			       				if (intersection[0]<0.0){
			       					if (i==4)
			       						arretes[5]=1;
			       					else
			       						arretes[4]=1;
			       				}
						       	else{
							       	if (intersection[0]>pos_plan[1]){
							       		if (i==4)
								       		arretes[7]=1;
								       	else
									       	arretes[6]=1;
							       	}
						       	}
					       	}
				       	}
				       	else{
					  //cout<<"4"<<endl;
					  if (intersection[(i/2)+1]>=0.0 && intersection[(i/2)+1]<=pos_plan[2*(i/2)+3]){
				       			index[i]=1;
							
					  }
				       	  else{
				       			if (intersection[(i/2)+1]<0.0){
					       			switch(i){
								       	case 0:	arretes[9]=1;
								       			break;
								       	case 1: arretes[8]=1;
								       			break;
								       	case 2:	arretes[1]=1;
								       			break;
								       	case 3:	arretes[0]=1;
									                break;
								       }
						       	}
						       	else{
							       	if(intersection[(i/2)+1]>pos_plan[2*(i/2)+3]){
								       	switch(i){
								       		case 0:	arretes[11]=1;
								       				break;
									       	case 1: arretes[10]=1;
									       			break;
									       	case 2:	arretes[3]=1;
									       			break;
									       	case 3:	arretes[2]=1;
									       			break;
								       	}	
							       	}
						       	}
					       	}
				       	}
			       	}
		       	}
			}
		}
	}



	// Compute the distance from the segment to the face if it's the minimum distance

	for (i=0;i<6;i++){
		if (index[i]==1){
			Point_proche_plan( point1, point2, i, longueur, largeur, hauteur, point);
			distance[i]=Distance_point_face(point,i, longueur, largeur, hauteur); 
		}
	}
	
	// Find the minimum distance to a face
	
	min = FLT_MAX;
	for(i=0;i<6;i++){
		if(distance[i]>0.){
			if(distance[i]<min){
				min=distance[i];
			}
		}
	}

	if(min!=FLT_MAX){
	  //if (k==1000){
	  //k=0;
	  //gettimeofday(&second,NULL);
	  //cout<< second.tv_sec - first.tv_sec<<" sec  "<<second.tv_usec  - first.tv_usec<<" usec"<<endl;
	  //}

	  return (min-rayon);		
	}			

	// Compute the minimal distance if there is no intersection on a face

	for (i=0;i<12;i++){
		if (arretes[i]==1){
			switch(i/4){
				case 0: vec_dir_edge[0]=0.0;
						vec_dir_edge[1]=largeur;
						vec_dir_edge[2]=0.0;
						break;

				case 1: vec_dir_edge[0]=0.0;
						vec_dir_edge[1]=0.0;
						vec_dir_edge[2]=hauteur;
						break;

				case 2: vec_dir_edge[0]=longueur;
						vec_dir_edge[1]=0.0;
						vec_dir_edge[2]=0.0;
						break;
			}

			if (i==0 || i==2 || i==6 || i==7)
				point_edge[0]=longueur;
			else
				point_edge[0]=0.0;

			if (i==4 || i==6 || i==10 || i==11)
				point_edge[1]=largeur;
			else
				point_edge[1]=0.0;
			
			if (i==2 || i==3 || i==8 || i==10)
				point_edge[2]=hauteur;
			else
				point_edge[2]=0.0;

			segments_nearest_points(point_edge, vec_dir_edge, point1, vec_dir_seg, &k1 , &k2, &distance_edge[i]);
		}
	}
	
	// Take the minimum distance

	min = FLT_MAX;
	for(i=0;i<12;i++){
		if(distance_edge[i]>0.){
			if(distance_edge[i]<min){
				min=distance_edge[i];
			}
		}
	}
	//if (k==1000){
	//k=0;
	//gettimeofday(&second,NULL);
	//cout<< second.tv_sec - first.tv_sec<<" sec  "<<second.tv_usec  - first.tv_usec<<" usec"<<endl;
	//}
	return (min-rayon);		
		
}

#else

float Parallelipiped::Distance(Capsule& cap, RigidTransfo& transfo){
  CVector3_t upperLinkExtr, lowerLinkExtr,ax;
  CVector3_t objPos,s1,s2; 
  CVector3_t v1,v2,linkv,u,vertexPos,v_tmp,tmp3,pos;
  CMatrix4_t ref,tmpmat,tmpmat2;
  int i,j,k,dir1,dir2,pindex,min_i;
  float alldists[12]; //closest distance to each edge
  float allpoints[24]; // closest pair of points on each edge
  float min_dist,dist,rad;
  int s3[3];
  CVector3_t contact_vector;
  float point;

  rad=cap.GetRadius(); 
  cap.GetAxis(ax);
  cap.GetPosition(pos);
  transfo.Transform(pos,upperLinkExtr);  
  v_add(pos,ax,pos);
  transfo.Transform(pos,lowerLinkExtr);
 
  //setting the center of parallelipiped
  v_scale(size,0.5,objPos);
  v_add(position,objPos,objPos);
  m_identity(ref);

//   transfo.RotToMatrix(tmpmat);
//   m_transpose(tmpmat,ref);

  v_sub(lowerLinkExtr,objPos,v1);
  v_sub(upperLinkExtr,objPos,v2);
  //  cout<<v1<<" "<<v2<<endl;
  v_clear(Ss3);
  
  //checking when two parallel sides must be checked
  for(i=0;i<3;i++){
    s1[i] = v_dot(v1, &ref[i*4]);
    s2[i] = v_dot(v2, &ref[i*4]);
    if (s1[i]*s2[i]>=0){
      s3[i] = sign(s1[i]+s2[i]);
    }
  }
  //    cout << "s3: ";coutvec(s3);
  m_copy(ref,tmpmat);
  for(i=0;i<3;i++){
    if(s3[i]){
      v_sub(lowerLinkExtr,upperLinkExtr,linkv);
      v_scale(size,-0.5,u);
      u[i] *=-s3[i];
      v_add(objPos,u,vertexPos);
      //      cout<<"vPos "<<i<<": ";coutvec(vertexPos);
      v_scale(&(ref[i*4]),s3[i],&(tmpmat[i*4]));
      // cout<<"norm "<<i<<": ";coutvec((tmpmat+i*4));
      m_inverse(tmpmat,tmpmat2);
      if(v_dot(&(tmpmat[i*4]),linkv)<0){
	v_sub(lowerLinkExtr,vertexPos,v_tmp);
	point = 1;
      }
      else{
	v_sub(upperLinkExtr,vertexPos,v_tmp);
	point = 0;
      }
      //      cout<<"linkv ";coutvec(linkv);
      // cout <<"pt "<<*point<<endl;
	
      
// #ifdef OLD_AV
//       v_copy(linkv,(CVector3_t)&tmpmat[i*4]);
//       m_inverse(tmpmat,tmpmat2);
//       v_sub(upperLinkExtr,vertexPos,tmp2);
      // tmp3 should contain the intersection coordinates in
      // the referential defined by the edges of the surface 
      v_transform_normal(v_tmp,tmpmat2,tmp3); 
      // cout<<"tmp3 ";coutvec(tmp3);
      if(tmp3[(i+1)%3]>=0 && tmp3[(i+2)%3]>=0 // the link points to the rectangle
	 && tmp3[(i+1)%3]<=size[(i+1)%3] && 
	 tmp3[(i+2)%3]<=size[(i+2)%3]){
	if(tmp3[i]<0){
	  dist = tmp3[i]-rad;
	  return dist; // there is a collision
	}
	else{
	  dist = tmp3[i];
	  v_copy(&(tmpmat[i*4]),contact_vector);
	  v_scale(contact_vector,dist,contact_vector);
	  return dist-rad;
	}
      }
    }
  }
  for(i=0;i<12;i++){
    alldists[i]=FLT_MAX;
  }
  // the link does not point to a rectangle -> look for the edges
  v_scale(size,-0.5,u);
  for(i=0;i<3;i++){// each kind of edge
      dir1 = s3[(i+1)%3]; 
      dir2 = s3[(i+2)%3];
    for(j=0;j<2;j++){ 
      if(dir1 == 0 || dir1==2*j-1){ //edges of this face must be computed
	for(k=0;k<2;k++){
	  if(dir2 == 0 || dir2==2*k-1){ //edges of this face must be computed
	    v_copy(u,v_tmp);
	    v_tmp[(i+1)%3]*=-(2*j-1);
	    v_tmp[(i+2)%3]*=-(2*k-1);
	    v_add(objPos,v_tmp,vertexPos);
	    v_scale(&(ref[4*i]),size[i],v1); // edge with the right length
	    pindex = 4*i+2*j+k;
	    segments_nearest_points(vertexPos,v1,upperLinkExtr,linkv,&(allpoints[2*pindex]),&(allpoints[2*pindex+1]),&(alldists[pindex]));
	//     cout<<"i:"<<i<<" j:"<<j<<" k:"<<k<<"dist "<<alldists[pindex]<<endl;
// 	    cout<<"v1:";coutvec(v1);
// 	    cout<<"ref:";coutvec((&(ref[4*i])));
// 	    cout<<"vP:";coutvec(vertexPos);
	  }
	}
      }
    }
  }
  //looking for the min
  min_dist = alldists[0];
  min_i = 0;
  for(i=1;i<12;i++){
    if(alldists[i] < min_dist){
      min_dist = alldists[i];
      min_i = i;
    }
  }
  // returning the min distance
  dist = min_dist;
  //  cout<<"min "<<min_i<<" "<<dist<<endl;
  point = allpoints[2*min_i+1];
  v_scale(linkv,point,v1);
  v_add(upperLinkExtr,v1,v2); // nearest point on link
  k = min_i%2; //retrieving the right edge
  j = ((min_i-k)%4)/2;
  i = min_i/4; 
  //  cout<<"ijk: "<<i<<" "<<j<<" "<<k<<endl;
  v_copy(u,v_tmp);
  v_tmp[(i+1)%3]*=-(2*j-1);
  v_tmp[(i+2)%3]*=-(2*k-1); 
  v_add(objPos,v_tmp,vertexPos); // starting vertex of the edge
  v_scale(&(ref[3*1]),allpoints[2*min_i]*(size[min_i]),v1);
  v_add(vertexPos,v1,v1); // nearest point on solid
  v_sub(v2,v1,contact_vector);
  return dist-rad;
}


#endif
