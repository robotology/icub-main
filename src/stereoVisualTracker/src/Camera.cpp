#include <iostream>
#include "Camera.h"
#include <stdio.h>
#include <assert.h>
//#include <glob.h>
#include <errno.h>
using namespace std;

Camera::Camera(char* name)
{
  this->name = name;

  YSize.width     = 0;
  YSize.height    = 0;
  CbCrSize.width  = 0;
  CbCrSize.height = 0;

  opened = false;
}

Camera::~Camera()
{
  free (name);
}

void Camera::close()
{
  YSize.width = 0;
  YSize.height = 0;
  CbCrSize.width = 0;
  CbCrSize.height = 0;
  opened = false;
}

bool Camera::open()
{
  opened = true;
  return true;
}

void Camera::setCameraIntrinsics(CvCamera* cam)
{

  intParams.principal_point.x = cam->matrix[2];
  intParams.principal_point.y = cam->matrix[5];
  
  intParams.focal_length[0] = cam->matrix[0];
  intParams.focal_length[1] = cam->matrix[4];
  
  intParams.distortion[0] = cam->distortion[0];
  intParams.distortion[1] = cam->distortion[1];
  intParams.distortion[2] = cam->distortion[2];
  intParams.distortion[3] = cam->distortion[3];
}

void Camera::setCameraExtrinsics(Cv3dTrackerCameraInfo* param)
{
  assert(param != NULL);

  memcpy(&(this->extParams), param, sizeof(Cv3dTrackerCameraInfo));
}

void Camera::saveCalibrationParams(char* filename)
{
  assert(filename != NULL);

  FILE* f = fopen(filename, "w");
  
  if(f == NULL) {
    printf("Problem saving calibration parameters : %s\n", strerror(errno));
    exit(-1);
  }

  assert(sizeof(char) == 1);

  // save the intrinsics parameters
  for (int i=0; i< (int)sizeof(intParams); i++)
    fprintf(f, "%c", ((char*)(&intParams))[i]);

  // save the extrinsics parameters
  for (int i=0; i< (int)sizeof(extParams); i++)
    fprintf(f, "%c", ((char*)(&extParams))[i]);

  fclose(f);
}
  

void Camera::loadCalibrationParams(char* filename)
{
  assert(filename != NULL);

  FILE* f = fopen(filename, "r");
  
  if(f == NULL) {
    printf("Problem loading calibration parameters : %s\n", strerror(errno));
    return;
    //exit(-1);
  }

  assert(sizeof(char) == 1);

  // load the intrinsics parameters
  for (int i=0; i< (int)sizeof(intParams); i++)
    fscanf(f, "%c", &(((char*)(&intParams))[i]));

  // load the extrinsics parameters
  for (int i=0; i< (int)sizeof(extParams); i++)
    fscanf(f, "%c", &(((char*)(&extParams))[i]));

  fclose(f);
}

CameraImage::CameraImage(Camera* cam, int width, int height){

  if(cam==NULL)
    return;

  if(width<=0)
    return;
  if(height<=0)
    return;

  m_Width  = width;
  m_Height = height;
  m_Camera = cam;

  m_Size.width = width;
  m_Size.height = height;


  CvSize y_size    = *(cam->getYSize());
  CvSize cbcr_size = *(cam->getCbCrSize());
  
  YCamImg  = cvCreateImageHeader(y_size,    IPL_DEPTH_8U, 1);
  CbCamImg = cvCreateImageHeader(cbcr_size, IPL_DEPTH_8U, 1);
  CrCamImg = cvCreateImageHeader(cbcr_size, IPL_DEPTH_8U, 1);
  
  YImg     = cvCreateImage( cvSize( m_Width,m_Height), IPL_DEPTH_8U, 1 );
  CbImg    = cvCreateImage( cvSize( m_Width,m_Height), IPL_DEPTH_8U, 1 );
  CrImg    = cvCreateImage( cvSize( m_Width,m_Height), IPL_DEPTH_8U, 1 );
  YCbCrImg = cvCreateImage( cvSize( m_Width,m_Height), IPL_DEPTH_8U, 3 );
  DRGBImg  = cvCreateImage( cvSize( m_Width,m_Height), IPL_DEPTH_8U, 3 );
  RGBImg   = cvCreateImage( cvSize( m_Width,m_Height), IPL_DEPTH_8U, 3 );

  int step;
  CvSize size;
  cvGetImageRawData( DRGBImg, (uchar**)&m_DData, &step, &size );
  cvGetImageRawData( RGBImg, (uchar**)&m_Data, &step, &size );

  DistMap = NULL;
}

void CameraImage::SetCameraParams(const CvCamera* cam){
  memcpy(&m_CameraParams,cam,sizeof(CvCamera));

  if(DistMap==NULL)
    DistMap   = cvCreateImage( cvSize( m_Width*3,m_Height*3), IPL_DEPTH_32S, 1 );

  cvUnDistortInit(RGBImg, DistMap,
                  m_CameraParams.matrix,
                  m_CameraParams.distortion,
                  1 );
  m_Camera->setCameraIntrinsics(&m_CameraParams);
}

void CameraImage::SetCameraReferential(float* transMat, float* rotMat){
  int i,j;
  memcpy(m_TransMat,transMat,4*4*sizeof(float));
  memcpy(m_RotMat,rotMat,4*4*sizeof(float));

  for(i=0;i<3;i++){
    for(j=0;j<3;j++){
      m_CameraParams.rotMatr[i*3+j]=rotMat[i*4+j];
    }
    m_CameraParams.transVect[i]=transMat[i*4+3];
  }
}

void CameraImage::SetCameraReferential(CvCamera* cam){
  int i,j;
  memset(m_TransMat,0,4*4*sizeof(float));
  memset(m_RotMat,0,4*4*sizeof(float));
  for(i=0;i<4;i++){
    m_TransMat[i][i]=1.0;
    m_RotMat[i][i]=1.0;
  }

  for(i=0;i<3;i++){
    for(j=0;j<3;j++){
      m_RotMat[i][j] = cam->rotMatr[i*3+j];
    }
    m_TransMat[i][3] = cam->transVect[i];
  }
}


void  CameraImage::SaveCameraParams(char* filename){
  if(filename==NULL)
    return;

  FILE* f = fopen(filename, "wb");
  if(f!=NULL){
    fwrite(&m_CameraParams,sizeof(CvCamera),1,f);
    fclose(f);
  }
}
void  CameraImage::LoadCameraParams(char* filename){
  if(filename==NULL)
    return;

  FILE* f = fopen(filename, "rb");
  if(f!=NULL){
    CvCamera camTmp;
    fread(&camTmp,sizeof(CvCamera),1,f);
    fclose(f);
    SetCameraParams(&camTmp);
    SetCameraReferential(&camTmp);
  }
}


CameraImage::~CameraImage(){
  cvReleaseImageHeader(&YCamImg);
  cvReleaseImageHeader(&CbCamImg);
  cvReleaseImageHeader(&CrCamImg);

  cvReleaseImage(&YImg);
  cvReleaseImage(&CbImg);
  cvReleaseImage(&CrImg);
  cvReleaseImage(&YCbCrImg);
  cvReleaseImage(&DRGBImg);
  cvReleaseImage(&RGBImg);

  if(DistMap!=NULL)
    cvReleaseImage(&DistMap);

}


void CameraImage::Update(){
  m_Camera->grabImage(YCamImg, CbCamImg, CrCamImg);

  cvResize(YCamImg,  YImg);
  cvResize(CbCamImg, CbImg);
  cvResize(CrCamImg, CrImg);
  cvCvtPlaneToPix(YImg, CrImg, CbImg, NULL, YCbCrImg);
  
  if(DistMap!=NULL){
    cvCvtColor(YCbCrImg, DRGBImg, CV_YCrCb2RGB);
    cvUnDistort( DRGBImg, RGBImg, DistMap, 1 );
  }else{
    cvCvtColor(YCbCrImg, RGBImg, CV_YCrCb2RGB);
    cvCopy(RGBImg,DRGBImg);
  }  
}

void CameraImage::GrabFrame(IplImage **img,u32 index){
  Update();
  *img = DRGBImg;
}



unsigned char *CameraImage::GetDataPtr(){
  return m_DData;
}
unsigned char *CameraImage::GetUnDistortDataPtr(){
  return m_Data;
}

IplImage *CameraImage::GetUnDistortImage(){
  return RGBImg;
}

IplImage *CameraImage::GetImage(){
  return DRGBImg;
}

void CameraImage::GetImageYCbCr(IplImage** _YImg, IplImage** _CbImg, IplImage** _CrImg){

  if(_YImg!=NULL)  *_YImg  = YImg;
  if(_CbImg!=NULL) *_CbImg = CbImg;
  if(_CrImg!=NULL) *_CrImg = CrImg;

}

#ifdef NEW_VERSION
CvSize  CameraImage::GetSize() {
 return m_Size; 
}
#else
CvSize*  CameraImage::GetSize() {
 return &m_Size; 
}
#endif
