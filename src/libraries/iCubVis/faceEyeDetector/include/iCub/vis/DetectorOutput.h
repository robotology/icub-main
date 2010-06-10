#include <yarp/os/Portable.h>
//#include <Portable.h>
#include <iCub/visualobject.h>
#include <iCub/faceboxlist.h>

#include <iCub/faceobject.h>
#include <yarp/sig/Image.h>

using namespace yarp::os;
using namespace yarp::sig;

class DetectorOutput: public Portable{
public:
	FaceBoxList facesOnly;
	list<FaceObject *> facesEyes;
	ImageOf<PixelRgb> *detectImage;
	bool eyeFlag;
	int nFaces;
	DetectorOutput(){
		detectImage = new ImageOf<PixelRgb>();
		//facesEyes = VisualObject();
		//facesOnly = FaceBoxList();
	}
	DetectorOutput(bool myEyeFlag){
		eyeFlag=myEyeFlag;
		/*if ( eyeFlag )
			facesEyes = VisualObject();
		else
			facesOnly = FaceBoxList();*/
	}
	~DetectorOutput(){
		//facesEyes.~VisualObject();
		//facesOnly.~FaceBoxList();
        delete detectImage;
		/*if ( eyeFlag )
		facesEyes.~VisualObject();
		else
		facesOnly.~ObjectList();*/
	}
	virtual bool write(ConnectionWriter& connection) {
		connection.appendInt(eyeFlag);
		if ( eyeFlag ){
			connection.appendInt(facesEyes.size());
			nFaces = facesEyes.size();
			if ( nFaces != 0 ){
				list<FaceObject *>::iterator face = facesEyes.begin();
				list<FaceObject *>::iterator last_face = facesEyes.end();
				for(int i = 0; face != last_face; ++face, ++i){
					// Current face plot
					FaceObject *f = static_cast<FaceObject*>(*face);
					connection.appendInt((int)f->x);
					connection.appendInt((int)f->xSize);
					connection.appendInt((int)f->y);
					connection.appendInt((int)f->ySize);

					connection.appendInt((int)f->eyes.xLeft);
					connection.appendInt((int)f->eyes.yLeft);
					connection.appendInt((int)f->eyes.xRight);
					connection.appendInt((int)f->eyes.yRight);

					int numLeftEyes = f->leftEyes.size();
					connection.appendInt(numLeftEyes);
					// Current face, left eyes plot
					for(int k = 0; k < (int)(f->leftEyes.size()); ++k){
						EyeObject *le = &(f->leftEyes[k]);
						connection.appendInt((int)le->scale);
						connection.appendInt((int)le->x);
						connection.appendInt((int)le->y);
					}

					// Current face, right eyes plot
					int numRightEyes = f->rightEyes.size();
					connection.appendInt(numRightEyes);
					for(int k = 0; k < (int)(f->rightEyes.size()); ++k){
						EyeObject *re = &(f->rightEyes[k]);
						connection.appendInt((int)re->scale);
						connection.appendInt((int)re->x);
						connection.appendInt((int)re->y);
					}
				}
			}
		}
		else{
			int nBoxes = facesOnly.size();
			connection.appendInt(nBoxes);
			int myCont = 0;
			if(nBoxes != 0) {
				//while(!facesOnly.empty( ))
				while ( myCont< nBoxes )
				{
					Square face = facesOnly.front();  
					facesOnly.pop_front();
					connection.appendInt(face.x);
					connection.appendInt(face.y);
					connection.appendInt(face.size);
					myCont++;
				}
			}
		}
		//printf("Before image writing\n");
		detectImage->write(connection);
		//printf("After image writing\n");
		return true;
	}
	virtual bool read(ConnectionReader& connection) {
		//printf("Starting to read input\n");
		int currentListSize;
		//eyeFlag = static_cast<bool> (connection.expectInt());
		eyeFlag = (connection.expectInt()!=0);
		if ( eyeFlag ){
			list<FaceObject *>::iterator currentFace = facesEyes.begin();
			list<FaceObject *>::iterator lastFace = facesEyes.end();
			for(; currentFace != lastFace; ++currentFace){
				//printf("i: %d\n",i);
				FaceObject *hTracker = *currentFace;
				delete hTracker;
			}
			facesEyes.clear();
			//printf("Reading faces and eyes\n");
			nFaces = connection.expectInt();
			//printf("List size: %d\n",currentListSize);
			/*if ( currentListSize != 0)
			facesEyes.clear();*/
			/*if ( currentListSize == 0 )
			if ( !facesEyes.empty() )
			facesEyes.clear();*/
			for ( int i=0; i < nFaces; i++){
				int currentXPos = connection.expectInt();
				int currentXSize = connection.expectInt();
				int currentYPos = connection.expectInt();
				int currentYSize = connection.expectInt();

				FaceObject *currentFace = new FaceObject((float)currentXPos, (float)currentYPos, (float)currentXSize, (float)currentYSize, 1.0f);
				/*connection.appendInt(f->x);
				connection.appendInt(f->xSize);
				connection.appendInt(f->y);
				connection.appendInt(f->ySize);*/

				int curLEXPos = connection.expectInt();
				int curLEYPos = connection.expectInt();
				int curREXPos = connection.expectInt();
				int curREYPos = connection.expectInt();

				currentFace->eyes = bEyesObject((float)curLEXPos, (float)curLEYPos, (float)curREXPos, (float)curREYPos);

				/*connection.appendInt(f->eyes.xLeft);
				connection.appendInt(f->eyes.yLeft);
				connection.appendInt(f->eyes.xRight);
				connection.appendInt(f->eyes.yRight);*/

				int numLeftEyes = connection.expectInt();
				//connection.appendInt(numLeftEyes);
				// Current face, left eyes plot
				for(int k = 0; k < numLeftEyes; ++k){
					/*EyeObject *le = &(f->leftEyes[k]);
					connection.appendInt(le->scale);
					connection.appendInt(le->x);
					connection.appendInt(le->y);*/
					int currentEyeScale = connection.expectInt();
					curLEXPos = connection.expectInt();
					curLEYPos = connection.expectInt();
					EyeObject *le =	new EyeObject(TSquare<float> (0, (float)curLEXPos, (float)curLEYPos, (float)currentEyeScale), lefteye);
					currentFace->leftEyes.push_back(EyeObject(*le));
                    delete le;
					//EyeObject(lefteye, float x_in, float y_in, float xSize_in, float ySize_in, float scale_in);
				}

				// Current face, right eyes plot
				int numRightEyes = connection.expectInt();
				//connection.appendInt(numRightEyes);
				for(int k = 0; k < numRightEyes; ++k){
					/*EyeObject *re = &(f->rightEyes[k]);
					connection.appendInt(re->scale);
					connection.appendInt(re->x);
					connection.appendInt(re->y);*/
					int currentEyeScale = connection.expectInt();
					curREXPos = connection.expectInt();
					curREYPos = connection.expectInt();
					EyeObject *re =	new EyeObject(TSquare<float> (0, (float)curREXPos, (float)curREYPos, (float)currentEyeScale), righteye);
					currentFace->rightEyes.push_back(EyeObject(*re));
                    delete re;
				}

				facesEyes.push_front(new FaceObject(*currentFace));
				delete currentFace;
				//VisualObject(float x_in, float y_in, float xSize_in, float scale_in, feature_type feature_in, float ySize_in = 0.0);
				//enum feature_type{e_face = 1, lefteye, righteye, botheyes};
			}
		}
		else {
			currentListSize = connection.expectInt();
			nFaces = currentListSize;
			if(currentListSize != 0) {
				for ( int k = 0; k < currentListSize; k++ ){
					int curFaceXPos = connection.expectInt();
					int curFaceYPos = connection.expectInt();
					int curFaceSize = connection.expectInt();
					/*connection.appendInt(face.x);
					connection.appendInt(face.y);
					connection.appendInt(face.size);*/
					facesOnly.push_back(TSquare<int> (curFaceSize, curFaceXPos, curFaceYPos, 0));
				}
			}
		}
		detectImage->read(connection);
		return true;
	}
};
