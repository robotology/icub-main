#include "public.h"
#include "pd_kalman.h"

static CvRandState gaussian;
static CvMat *pGaussianResult = NULL;

static float A[STATE_VECTOR_SIZE * STATE_VECTOR_SIZE] = {
	1.0, 0.0, 1.0, 0.0,			// x = x + Vx
	0.0, 1.0, 0.0, 1.0,			// y = y + Vy
	0.0, 0.0, 1.0, 0.0,			// Vx = Vx
	0.0, 0.0, 0.0, 1.0,			// Vy = Vy
};

static float B[STATE_VECTOR_SIZE_3D * STATE_VECTOR_SIZE_3D] = {
	1.0, 0.0, 0.0, 1.0, 0.0, 0.0,		// x = x + Vx
	0.0, 1.0, 0.0, 0.0, 1.0, 0.0,		// y = y + Vy
	0.0, 0.0, 1.0, 0.0, 0.0, 1.0,		// z = z + Vz
	0.0, 0.0, 0.0, 1.0, 0.0, 0.0,		// Vy
	0.0, 0.0, 0.0, 0.0, 1.0, 0.0,		// Vy
	0.0, 0.0, 0.0, 0.0, 0.0, 1.0		// Vz
};

float UniModal()
{
	float r;
	cvRand(&gaussian, pGaussianResult);
	r = pGaussianResult->data.fl[0];
	if (r < -1) r = -1;
	if (r > 1) r = 1;
	return r;
}

CKalman::CKalman(){
  Init(false,1.0f);
}



CKalman::CKalman(bool b3d, float noise){
  Init(b3d,noise);
}

CKalman::CKalman(bool b3d, float mnoise,float pnoise){
  Init(b3d,mnoise,pnoise);
}
void CKalman::Init(bool b3d, float measurement_noise, float process_noise)
{
	CKalman::b3d = b3d;
	p_noise = process_noise;
	m_noise = measurement_noise;
	p_noise_incr = 2.0;
	m_noise_incr = 2.0;
	if (!b3d)
	{
		MODEL_STATE initial_state;
		initial_state.model_struct.x = 0;
		initial_state.model_struct.y = 0;
		initial_state.model_struct.Vx = 0;
		initial_state.model_struct.Vy = 0;

		pKalman = cvCreateKalman(STATE_VECTOR_SIZE, MEASUREMENT_SIZE, 0);

		memcpy( pKalman->transition_matrix->data.fl, A, sizeof(A));
		memcpy( pKalman->state_post->data.fl, &(initial_state.state),
sizeof(initial_state.state));
	}
	else
	{
		MODEL_STATE_3D initial_state;
		initial_state.model_struct.x = 0;
		initial_state.model_struct.y = 0;
		initial_state.model_struct.z = 0;
		initial_state.model_struct.Vx = 0;
		initial_state.model_struct.Vy = 0;
		initial_state.model_struct.Vz = 0;

		pKalman = cvCreateKalman(STATE_VECTOR_SIZE_3D, MEASUREMENT_SIZE_3D, 0);

		memcpy( pKalman->transition_matrix->data.fl, B, sizeof(B));
		memcpy( pKalman->state_post->data.fl, &(initial_state.state),
sizeof(initial_state.state));
	}
	cvSetIdentity( pKalman->measurement_matrix, cvRealScalar(1) );	// 1:1 correlation between model and measurement
// 	cvSetIdentity( pKalman->process_noise_cov, cvRealScalar(p_noise) );
// 	cvSetIdentity( pKalman->measurement_noise_cov, cvRealScalar(m_noise));
	SetNoise(m_noise);
	SetProcessNoise(p_noise);
	cvSetIdentity( pKalman->error_cov_post, cvRealScalar(1));
}

void CKalman::Kill()
{
	cvReleaseKalman(&pKalman);
}

CvPoint2D32f CKalman::PredictAndCorrect(CvPoint2D32f measurement)
{
	if (b3d) return cvPoint2D32f(-1, -1);

	// Predict location
	const CvMat *pPrediction = cvKalmanPredict(pKalman);
	CvPoint2D32f predictedPt =
	  cvPoint2D32f(pPrediction->data.fl[0],pPrediction->data.fl[1]);

	// Correct Model
	CvMat *pLocation = cvCreateMat(2,1,CV_32FC1);
	pLocation->data.fl[0] = (float)measurement.x;
	pLocation->data.fl[1] = (float)measurement.y;
	cvKalmanCorrect(pKalman,pLocation);
	//delete pLocation;

	return predictedPt;
}

CvPoint3D32f CKalman::PredictAndCorrect(CvPoint3D32f measurement)
{
	if (!b3d) return cvPoint3D32f(-1, -1, -1);
	// Predict location
	//const CvMat *pPrediction = 
	cvKalmanPredict(pKalman);
	// Correct Model
	CvMat *pLocation = cvCreateMat(3,1,CV_32FC1);
	pLocation->data.fl[0] = (float)measurement.x;
	pLocation->data.fl[1] = (float)measurement.y;
	pLocation->data.fl[2] = (float)measurement.z;
	const CvMat *pPrediction = cvKalmanCorrect(pKalman,pLocation);
	CvPoint3D32f predictedPt = cvPoint3D32f(pPrediction->data.fl[0],
	pPrediction->data.fl[1],pPrediction->data.fl[2]);	

	return predictedPt;
}

// updates measurement noise
void CKalman::SetNoise(float noise)
{
  if(noise>0){
    m_noise = noise;
    cvSetIdentity(pKalman->measurement_noise_cov, cvRealScalar(noise));
  }
}

// updates process noise
void CKalman::SetProcessNoise(float noise)
{
  if(noise>0){
    p_noise = noise;
    cvSetIdentity(pKalman->process_noise_cov, cvRealScalar(noise));
  }
}


float CKalman::DecrementNoise(){
  if(m_noise>m_noise_incr){
    SetNoise(m_noise-m_noise_incr);
  }
  return m_noise;
}

float CKalman::DecrementProcessNoise(){
  if(p_noise>p_noise_incr){
    SetProcessNoise(p_noise-p_noise_incr);
  }
  return p_noise;
}


float CKalman::IncrementNoise(){
  SetNoise(m_noise+m_noise_incr);
  return m_noise;
}

float CKalman::IncrementProcessNoise(){
  SetProcessNoise(p_noise+p_noise_incr);
  return p_noise;
}



void CKalmanList::Init(bool b3d, int count, float noise)
{
	CKalmanList::b3d = b3d;
	pList = new CKalman[count];
	if (!b3d) pPredictions = new CvPoint2D32f[count];
	else pPredictions3D = new CvPoint3D32f[count];
	for(int i=0; i < count; i++)
	{
		pList[i].Init(b3d, noise);
		if (!b3d) pPredictions[i] = cvPoint2D32f(-1, -1);
		else pPredictions3D[i] = cvPoint3D32f(-1, -1, -1);
	}
}

void CKalmanList::Process(CvPoint3D32f *pPoints, int count)
{
	if(!b3d) return;
	for(int i=0; i < count; i++)
	{
		if(pPoints[i].x != -1)
		{
			pPredictions3D[i] = pList[i].PredictAndCorrect(pPoints[i]);
		}
	}
}

void CKalmanList::Process(CvPoint2D32f *pPoints, int count)
{
	if(b3d) return;
	for(int i=0; i < count; i++)
	{
		if(pPoints[i].x != -1)
		{
			pPredictions[i] = pList[i].PredictAndCorrect(pPoints[i]);
		}
	}
}


CvPoint2D32f CKalmanList::GetPrediction(int index)
{
	return pPredictions[index];
}

CvPoint2D32f *CKalmanList::GetPredictions()
{
	return pPredictions;
}

CvPoint3D32f CKalmanList::GetPrediction3D(int index)
{
	return pPredictions3D[index];
}

CvPoint3D32f *CKalmanList::GetPredictions3D()
{
	return pPredictions3D;
}
