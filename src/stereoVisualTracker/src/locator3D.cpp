#include <iostream>
#include "public.h"
#include "basicOpenCV.h"
#include "basicMath.h"
#include "locator3D.h"
#include "quatMath.h"


using namespace std;

Locator3D::Locator3D()
{
  isCalibrated = 0;
  nb_points=0;
}

Locator3D::Locator3D(char *filename){
  isCalibrated = 0;
  LoadParams(filename);
  strcpy(paramfile,filename);
}

void Locator3D::AddPoint(Vec2 p1,Vec2 p2){
//   pos2d[nb_points].id =nb_points;
//   pos2d[nb_points].p.x =p1.x;
//   pos2d[nb_points].p.y =p1.y;
//   pos2d[MAX_OBJ+nb_points].id =nb_points;
//   pos2d[MAX_OBJ+nb_points].p.x =p2.x;
//   pos2d[MAX_OBJ+nb_points].p.y =p2.y;

//   cout<<pos2d[nb_points][0].p.x<<" "<<pos2d[nb_points][0].p.y<<" - "
//       <<pos2d[nb_points][1].p.x<<" "<<pos2d[nb_points][1].p.y<<endl;


  pos2d[0][nb_points].id =nb_points;
  pos2d[0][nb_points].p.x =p1.x;
  pos2d[0][nb_points].p.y =p1.y;
  pos2d[1][nb_points].id =nb_points;
  pos2d[1][nb_points].p.x =p2.x;
  pos2d[1][nb_points].p.y =p2.y;

  nb_points++;

  if(nb_points==MAX_OBJ){nb_points--;}
}


int Locator3D::Get3dPoint(int i, Vec3 *point){
  if(i>=MAX_OBJ){
    return 0;
  }
  if(pos3d[i].id == -1){
    return 0;
  }
  //  cout<<pos3d[i].id<<" : "<<pos3d[i].p.x<<endl;
  point->x = pos3d[i].p.x;
  point->y = pos3d[i].p.y;
  point->z = pos3d[i].p.z;
  return 1;
}



int Locator3D::LocateAllPoints(){
  if(isCalibrated){
 //    if(! camInfos[0].valid || !camInfos[1].valid)
//       {cout<<"not valid"<<endl;}else{cout<<"valid"<<endl;}
   

   for(int i=nb_points;i<MAX_OBJ;i++){
      pos2d[0][i].id=-1;
      pos2d[1][i].id=-1;
    }
    int res= cv3dTrackerLocateObjects(2, MAX_OBJ, camInfos,
				      (Cv3dTracker2dTrackedObject *)pos2d, pos3d);
    //    cout<<pos2d[0][0].p.x<<" "<<pos3d[0].p.z<<endl;
    return res;
  }
  else{
    return 0;
  }
}

int Locator3D::Locate(int nbpoints, const Vec2 *p1, const Vec2 *p2, Vec3 *outpos, int *found)
{
  int i;
  int res;
  if(isCalibrated){
    for(i=0;i<nbpoints;i++){
      pos2d[0][i].id=i;//was 0; 
      pos2d[0][i].p.x = p1[i].x;
      pos2d[0][i].p.y = p1[i].y;
      pos2d[1][i].id=i;// was 0;
      pos2d[1][i].p.x = p2[i].x;
      pos2d[1][i].p.y = p2[i].y;
    }

    for(i=nbpoints;i<MAX_OBJ;i++){ //maybe not necessary
      pos2d[0][i].id=-1;
      pos2d[1][i].id=-1;
    }

    res = cv3dTrackerLocateObjects(2, MAX_OBJ, camInfos,
				   (Cv3dTracker2dTrackedObject *)pos2d, pos3d);
    
    for(i=0;i<nbpoints;i++){
      if(pos3d[i].id==-1){
	found[i] = 0;
      }
      else{
	found[i] = 1;
	outpos[i] = pos3d[i].p;
      }
    }
    return res;
  }
  else{
    cout <<"not calibrated"<<endl;
    return 0;
  }
}


int Locator3D::IsCalibrated()const{
  return isCalibrated;
}


void Locator3D::SetIntrinsicsParams(const CvCamera *cam[]){
 for(int i=0;i<2;i++){
   SetIntrinsicsParams(i,cam[i]);
 }
}

void Locator3D::SetIntrinsicsParams(int i, const CvCamera *cam){
  camera_intrinsics[i].principal_point.x = cam->matrix[2];
  camera_intrinsics[i].principal_point.y = cam->matrix[5];
  camera_intrinsics[i].focal_length[0]= cam->matrix[0];
  camera_intrinsics[i].focal_length[1]= cam->matrix[4];
  memcpy(camera_intrinsics[i].distortion,cam->distortion,4*sizeof(float));
}


int Locator3D::Calibrate(IplImage **img, const CvCamera *cam[], CvSize checkerboard_size, float sq_size)
{

  SetIntrinsicsParams(cam);  
  if(cv3dTrackerCalibrateCameras(2,camera_intrinsics,checkerboard_size,sq_size,img,camInfos)){
    isCalibrated=1;
    nb_points =0;
    cout<<"calibration done"<<endl;
    //    PrintParams();//for checking
    return 1;
  }
  else{
    isCalibrated=0;
    return 0;
  }
}

int Locator3D::SaveParams(const char* filename){
  FILE *fp;
  if(IsCalibrated()){
    if(!(fp=fopen(filename,"wb"))){
      cout<<"cannot open file "<<filename<<endl;
      return 0;
    }
    if(2!=fwrite(camInfos,sizeof( Cv3dTrackerCameraInfo),2,fp)){
      cout<<"cannot write data"<<endl;
      fclose(fp);
      return(0);
    }
    fclose(fp);
    return 1;
  }
  else{
    cout<<"no parameters to save";
    return 0;
  }
}

int Locator3D::LoadParams(const char* filename){
  FILE *fp;
   if(!(fp=fopen(filename,"rb"))){
     cout<<"cannot open file "<<filename<<endl;
     return 0;
   }
    if(2!=fread(camInfos,sizeof( Cv3dTrackerCameraInfo),2,fp)){
      cout<<"cannot read data"<<endl;
      fclose(fp);
      return(0);
    }
    isCalibrated=1;
    fclose(fp);
    return 1;
}
    

void Locator3D::PrintParams(int k){
    float rotmat[9];
  if(k>0){
    cout<<"extrinsic paramerters"<<endl;
    for(int i=0;i<2;i++){
      cout<<"camera "<<i<<": "<<(int)camInfos[i].valid<<endl;
      for(int j=0;j<4;j++){
	 for(int k=0;k<4;k++){
	   cout<<camInfos[i].mat[j][k]<<" ";
	   if(j<3 && k<3){
	     rotmat[j*3+k] =camInfos[i].mat[j][k];
	   }
	 }
	 cout<<endl;
      }
      cout<<endl;
      Quat q;
      float angle =q.fromMatrix(rotmat); 
      cout<<"angle "<<angle*180/PIf;
      angle = 1.f/sin(angle*0.5);
      cout<<": "<<q.x*angle<<" "<<q.y*angle<<" "<<q.z*angle<<endl;
      cout<<endl;
    }
  }
}

void Locator3D::SetDefaultTranslation(float dist){
 float trans[2][3];
 trans[0][0] = -0.5*dist; trans[0][1]= 0; trans[0][2]= 0;
  trans[1][0] = 0.5*dist; trans[1][1]= 0; trans[1][2]= 0;
 for(int i=0;i<2;i++){
   camInfos[i].mat[3][0] = -trans[i][0];
   camInfos[i].mat[3][1] = -trans[i][1];
   camInfos[i].mat[3][2] = -trans[i][2];
   camInfos[i].mat[3][3] = 1.f;
 }
}

void Locator3D::SetDefaultParams(float angle0, float angle1, float dist){
  float a[2];
  float trans[2][3];
  float s,c,flx,fly;
  a[0] =  angle0 *PIf/180;
  a[1] =  angle1 *PIf/180;
 
  trans[0][0] = -0.5*dist; trans[0][1]= 0; trans[0][2]= 0;
  trans[1][0] = 0.5*dist; trans[1][1]= 0; trans[1][2]= 0;

  for(int i=0;i<2;i++){
    s= sin(a[i]);
    c= cos(a[i]);
    flx = camera_intrinsics[i].focal_length[0];
    fly = camera_intrinsics[i].focal_length[1];
    camInfos[i].mat[0][0] = -c/flx;
    camInfos[i].mat[0][1] = 0;
    camInfos[i].mat[0][2] = -s/flx;
    camInfos[i].mat[1][0] = 0;
    camInfos[i].mat[1][1] = -1/fly;
    camInfos[i].mat[1][2] = 0;
    camInfos[i].mat[2][0] = -s;
    camInfos[i].mat[2][1] = 0;
    camInfos[i].mat[2][2] = c;
    camInfos[i].mat[3][0] = -trans[i][0];
    camInfos[i].mat[3][1] = -trans[i][1];
    camInfos[i].mat[3][2] = -trans[i][2];
    camInfos[i].mat[0][3] = camInfos[i].mat[1][3] = camInfos[i].mat[2][3]=0.f;
    camInfos[i].mat[3][3] = 1.f;
    camInfos[i].valid = true;   
  }
}
    
void Locator3D::Decalibrate(){
  isCalibrated =0;
  nb_points =0;
}
