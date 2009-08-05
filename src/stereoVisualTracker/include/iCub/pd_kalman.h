#ifndef PD_KALMAN_H
#define PD_KALMAN_H

#define STATE_VECTOR_SIZE 4		// x, y, Vx, Vy
#define MEASUREMENT_SIZE 2		// x, y

#define STATE_VECTOR_SIZE_3D 6  // x, y, z, Vx, Vy, Vz
#define MEASUREMENT_SIZE_3D 3	// x, y, Z

typedef union {
	float state[STATE_VECTOR_SIZE];	// assert sizeof(float) == sizeof(CV_32FC1)
	struct {
		float x,y;		// current position
		float Vx,Vy;	// x and y velocities
	} model_struct;
} MODEL_STATE;

typedef union {
	float state[STATE_VECTOR_SIZE_3D];	// assert sizeof(float) == sizeof(CV_32FC1)
	struct {
		float x,y,z;		// current position
		float Vx,Vy,Vz;	// x and y velocities
	} model_struct;
} MODEL_STATE_3D;

class CKalman
{
protected:
	bool b3d;
	float m_noise;
	float p_noise;
	float m_noise_incr;
	float p_noise_incr;

public:
	CvKalman *pKalman;

	CKalman();
	CKalman(bool b3d, float noise);
	CKalman(bool b3d, float mnoise,float pnoise);
	void Init(bool b3d, float measurement_noise=5,float process_noise=5);
	void Kill();
	CvPoint2D32f PredictAndCorrect(CvPoint2D32f measurement);
	CvPoint3D32f PredictAndCorrect(CvPoint3D32f measurement);
	void SetNoise(float noise);
	void SetProcessNoise(float noise);
	float IncrementNoise();
	float DecrementNoise();
	float IncrementProcessNoise();
	float DecrementProcessNoise();
};

class CKalmanList
{
public:
	bool b3d;
	CKalman *pList;
	CvPoint2D32f *pPredictions;
	CvPoint3D32f *pPredictions3D;

	void Init(bool b3d, int count, float noise);
	void Process(CvPoint2D32f *pPoints, int count);
	void Process(CvPoint3D32f *pPoints, int count);
	CvPoint2D32f GetPrediction(int index);
	CvPoint2D32f *GetPredictions();
	CvPoint3D32f GetPrediction3D(int index);
	CvPoint3D32f *GetPredictions3D();
};

#endif // PD_KALMAN_H
