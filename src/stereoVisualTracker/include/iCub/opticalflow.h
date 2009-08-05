#ifndef _OPTICALFLOW_H_
#define _OPTICALFLOW_H_


int opticalflow(char *filename);


class OpticalFlow : public Filter
{
private:
	u32 count;
	s32 nbFeatures;
	u32 maxFeatures;
	Vec2 *pre_features;
	Vec2 *post_features;
	Vec2 *dist, *ori, *arrival;
	char *of_feature_found;
	float *of_feature_error;
	IplImage *pre, *dst, *eig, *tmp, *pyr1, *pyr2;
	double startpositionx,startpositiony, motionx, motiony;
	int countframe;
	bool externalFeatures;
	u32 *index;

	void ComputeMotion();
	void PruneOutliers();
	void PruneOutliers2(float ratio_small, float ratio_big);

public:
	OpticalFlow(u32 features=400);
	~OpticalFlow();
	void Draw(IplImage *image);
	void Drawold(IplImage *image);
	void Apply(IplImage *image);
	IplImage *Process(IplImage *image){return NULL;};
	CvPoint2D32f GetEstimatedPos();
	CvPoint2D32f GetMotion();
	
	Vec2 *GetOrigins(){return ori;};
	Vec2 *GetVectors(){return dist;};
	int GetCount(){return count;};
	//added by micha
	s32 GetNbFeatures(){return nbFeatures;};
	int SetFeaturesFromNeighbours(IplImage *img,Vec2 *feature, s32 nbFeatures,u32 *ext_index);		
	bool GetPreFeature(int i, Vec2 *feat)
	{if(!of_feature_found[i])return false; *feat=pre_features[i];return true;};
	bool GetPostFeature(int i, Vec2 *feat)	
	{if(!of_feature_found[i])return false; *feat=post_features[i];return true;};	
	void GetOrigin(int i, Vec2 *feat){*feat = ori[i];};
	void GetEnd(int i, Vec2 *feat){*feat = arrival[i];};
	Vec2 GetOrigin(int i){return ori[i];}
	Vec2 GetEnd(int i){return arrival[i];}
	IplImage *GetPreImage(){return pre;};
	IplImage *GetPostImage(){return dst;};
	int CleanFeatures(Vec2 *features, char *valid, s32 size,u32 *ext_index);
	void ComputeEnds();
	Vec2 *GetEnds(){return arrival;}
	IplImage *GetPyrBuffer1(){return pyr1;};
	IplImage *GetPyrBuffer2(){return pyr2;};
	int RetrieveEnd(Vec2 feat,u32 index);
	u32 *GetIndex(){return index;};
};







#endif // _OPTICALFLOW_H_
