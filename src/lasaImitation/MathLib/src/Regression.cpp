#include "Regression.h"
#include <fstream>
#include "Vector.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

Matrix& Regression::HermitteSplineFit(const Matrix& inData, const Vector& inTimeBase, const Vector& outTimeBase, Matrix& outData){
  const int dataSize  = inData.ColumnSize(); 
  const int inSize    = inData.RowSize();
  const int outSize   = outTimeBase.Size();
   
  outData.Resize(outSize,dataSize);
  const REALTYPE startTime = inTimeBase.At(0);
  const REALTYPE stopTime  = inTimeBase.At(inSize-1);
  for(int i=0;i<outSize;i++){
    // Find the nearest data pair
    const REALTYPE cTime = outTimeBase.At(i);
    int   prev, next;
    REALTYPE prevTime, nextTime;
    if(cTime<=startTime){
      for(int j=0;j<dataSize;j++)
        outData(i,j) = inData.At(0,j);        
    }else if(cTime>=stopTime){
      for(int j=0;j<dataSize;j++)
        outData(i,j) = inData.At(inSize-1,j);
    }else{ 
      prev      = 0;
      prevTime  = startTime; 
      for(int j=1;j<inSize;j++){
        next      = j;
        nextTime  = inTimeBase.At(j);
        if((cTime>=prevTime)&&(cTime<nextTime)) break;        
        prev      = next;
        prevTime  = nextTime; 
      }
      const REALTYPE nphase = (cTime-prevTime)/(nextTime-prevTime);
      const REALTYPE s1 = nphase;
      const REALTYPE s2 = s1*nphase;
      const REALTYPE s3 = s2*nphase; 
      const REALTYPE h1 =  2.0f*s3 - 3.0f*s2 + 1.0f;
      const REALTYPE h2 = -2.0f*s3 + 3.0f*s2;
      const REALTYPE h3 =       s3 - 2.0f*s2 + s1;
      const REALTYPE h4 =       s3 -      s2;
      for(int j=0;j<dataSize;j++){
        const REALTYPE p0 = (prev>0?inData.At(prev-1,j):inData.At(prev,j));
        const REALTYPE p1 = inData.At(prev,j);
        const REALTYPE p2 = inData.At(next,j);      
        const REALTYPE p3 = (next<inSize-1?inData.At(next+1,j):inData.At(next,j));
        const REALTYPE t1 = 0.5f*(p2-p0);
        const REALTYPE t2 = 0.5f*(p3-p1);
        outData(i,j) = p1*h1+p2*h2+t1*h3+t2*h4;
      }      
    }
  }   
  return outData;
}


Matrix& Regression::HermitteSplineFit(const Matrix& inData, int nbSteps, Matrix& outData){
  if(nbSteps<=0)
    return outData;
    
  const int dataSize  = inData.ColumnSize(); 
  const int inSize    = inData.RowSize();
  const int outSize   = nbSteps;
   
  outData.Resize(outSize,dataSize);
  for(int i=0;i<outSize;i++){
    // Find the nearest data pair
    const REALTYPE cTime = REALTYPE(i)/REALTYPE(outSize-1)*REALTYPE(inSize-1);
    int   prev, next;
    REALTYPE prevTime, nextTime;

    prev      = int(floor(cTime));
    next      = prev+1;
    prevTime  = REALTYPE(prev);
    nextTime  = REALTYPE(next); 
    const REALTYPE nphase = (cTime-prevTime)/(nextTime-prevTime);
    const REALTYPE s1 = nphase;
    const REALTYPE s2 = s1*nphase;
    const REALTYPE s3 = s2*nphase; 
    const REALTYPE h1 =  2.0f*s3 - 3.0f*s2 + 1.0f;
    const REALTYPE h2 = -2.0f*s3 + 3.0f*s2;
    const REALTYPE h3 =       s3 - 2.0f*s2 + s1;
    const REALTYPE h4 =       s3 -      s2;
    for(int j=0;j<dataSize;j++){
      const REALTYPE p0 = (prev>0?inData.At(prev-1,j):inData.At(prev,j));
      const REALTYPE p1 = inData.At(prev,j);
      const REALTYPE p2 = inData.At(next,j);      
      const REALTYPE p3 = (next<inSize-1?inData.At(next+1,j):inData.At(next,j));
      const REALTYPE t1 = 0.5f*(p2-p0);
      const REALTYPE t2 = 0.5f*(p3-p1);
      outData(i,j) = p1*h1+p2*h2+t1*h3+t2*h4;
    }      
  }   
  return outData;  
}


Vector& Regression::HermitteSplineFit(const Vector& inData, const Vector& inTimeBase, const Vector& outTimeBase, Vector& outData){
  Matrix inMat(inData.Size(),1);
  Matrix outMat(outData.Size(),1);
  inMat.SetColumn(inData,0);  
  HermitteSplineFit(inMat,inTimeBase,outTimeBase,outMat);
  outMat.GetColumn(0,outData);
  return outData;
}

Vector& Regression::HermitteSplineFit(const Vector& inData, int nbSteps, Vector& outData){
  Matrix inMat(inData.Size(),1);
  Matrix outMat(outData.Size(),1);
  inMat.SetColumn(inData,0);  
  HermitteSplineFit(inMat,nbSteps,outMat);
  outMat.GetColumn(0,outData);
  return outData;
}


void Regression::SaveData(Matrix &data, const char fileName[])
{
	std::ofstream file(fileName);
	file << data.RowSize() << ' ' << data.ColumnSize() << std::endl;
	for (int j = 0; j < (int)data.RowSize(); j++)
	{
		for (int i = 0; i < (int)data.ColumnSize(); i++)
			file << data(j, i) << ' ';
		file << std::endl;
	}
}

bool Regression::GaussProduct(
	Matrix &data0,
	Matrix &data1,
	const int yIndices[],
	int yIndicesCount,
	Matrix *covarianceMatrices0,
	int covarianceMatricesCount0,
	Matrix *covarianceMatrices1,
	int covarianceMatricesCount1,
	Matrix &resultData,
	Vector &resultC,
  bool keepAlignment,bool pdiff,Matrix * eigDiff)
{
  
  REALTYPE maxDiff,minDiff;
  minDiff =  1000000.0f;
  maxDiff = -1000000.0f;


	int rowSize, columnSize;
	Vector xHat, mu, mu0, mu1;
	Matrix sigma, invSigma;
	Matrix invSigma0, invSigma1;
	if (!(
		data0.RowSize() == data1.RowSize() &&
		data1.RowSize() == covarianceMatricesCount0 &&
		covarianceMatricesCount0 == covarianceMatricesCount1 &&
		data0.ColumnSize() == data1.ColumnSize()))
		return false;
	rowSize = data0.RowSize();
	columnSize = data0.ColumnSize();
	Create(yIndicesCount, rowSize);
  if(keepAlignment){
    resultData.Resize(rowSize, columnSize);
  }else{
	  resultData.Resize(rowSize, yIndicesCount);
  }
	resultC.Resize(rowSize);
	xHat.Resize(yIndicesCount, false);
	mu.Resize(yIndicesCount, false);
	mu0.Resize(yIndicesCount, false);
	mu1.Resize(yIndicesCount, false);
	invSigma0.Resize(yIndicesCount, yIndicesCount, false);
	invSigma1.Resize(yIndicesCount, yIndicesCount, false);
	invSigma.Resize(yIndicesCount, yIndicesCount, false);
	for (int k = 0; k < rowSize; k++)
	{
		REALTYPE invDenominator;
		if (!(
			covarianceMatrices0[k].RowSize() == covarianceMatrices0[k].ColumnSize() &&
			covarianceMatrices0[k].ColumnSize() == covarianceMatrices1[k].RowSize() &&
			covarianceMatrices1[k].RowSize() == covarianceMatrices1[k].ColumnSize()))
			return false;
		sigma = covarianceMatrices0[k] + covarianceMatrices1[k];
		sigma.Inverse(invSigma, &invDenominator);
		invDenominator = invDenominator < 0 ? -invDenominator : invDenominator;
		invDenominator = sqrtf(invDenominator);
		invDenominator *= powf(PIf * 2.0f, (REALTYPE)columnSize * 0.5f);
		invDenominator = 1.0f / invDenominator;
		REALTYPE sum0 = 0;
		for (int j = 0; j < yIndicesCount; j++)
			xHat(j) = data1(k, yIndices[j]) - data0(k, yIndices[j]);
		for (int j = 0; j < yIndicesCount; j++)
		{
			REALTYPE sum1 = 0;
			for (int i = 0; i < yIndicesCount; i++)
				sum1 +=  xHat(j) * invSigma(j, i);
			sum0 += sum1 * xHat(j);
		}
		resultC(k) = expf(sum0 * -0.5f) * invDenominator ;
    
    REALTYPE det0,det1,detDiff;
    
		covarianceMatrices0[k].Inverse(invSigma0,&det0,1e10);
		covarianceMatrices1[k].Inverse(invSigma1,&det1,1e10);
    invSigma0.Add(invSigma1,sigma);
		//sigma = invSigma0 + invSigma1;
		sigma.Inverse(covarianceMatrices[k]);
    //if(!Matrix::IsInverseOk())
    //  covarianceMatrices[k].Print();
		for (int j = 0; j < yIndicesCount; j++)
		{
			mu0(j) = data0(k, yIndices[j]);
			mu1(j) = data1(k, yIndices[j]);
		}
		mu = covarianceMatrices[k] * (invSigma0 * mu0 + invSigma1 * mu1);

    det0 = 0.0f;
    det1 = 0.0f;
    REALTYPE cov;
    for (int j = 0; j < yIndicesCount; j++){
      cov=sqrt(covarianceMatrices0[k](j,j));      
      //det0 += cov;
      if(det0<cov) det0=cov; 
      cov=sqrt(covarianceMatrices1[k](j,j));
      //det1 += cov;
      if(det1<cov) det1=cov;
    }
    det0 /= REALTYPE(yIndicesCount);
    det1 /= REALTYPE(yIndicesCount);

    
    detDiff = det0-det1;
    if(maxDiff<detDiff) maxDiff = detDiff;
    if(minDiff>detDiff) minDiff = detDiff;

    // Truncing to objects
    REALTYPE thLow   = 0.004f;
    REALTYPE thHigh  = 0.008f;
    REALTYPE thDelta = thHigh-thLow;
    
    if(eigDiff!=NULL){
      (*eigDiff)(yIndices[0],k) = thLow;         
      (*eigDiff)(yIndices[1],k) = thHigh;         
      (*eigDiff)(yIndices[2],k) = det0;         
      (*eigDiff)(yIndices[3],k) = det1;         
    }
    
    // Truncing to objects
    
    Vector muReg(mu);
    
    REALTYPE lambda0 = (det0-thLow)/thDelta;
    REALTYPE lambda1 = (det1-thLow)/thDelta; 
    lambda0 = TRUNC(lambda0,0.0f,1.0f);
    lambda1 = TRUNC(lambda1,0.0f,1.0f);
    
        mu = ( (muReg*lambda1 + mu1*(1.0f-lambda1))*lambda0 + (mu0*lambda1 + muReg*(1.0f-lambda1))*(1.0f-lambda0) )*0.5f+
             ( (muReg*lambda0 + mu0*(1.0f-lambda0))*lambda1 + (mu1*lambda0 + muReg*(1.0f-lambda0))*(1.0f-lambda1) )*0.5f;

/*    if(det0>=thHigh){
      if(det1>=thHigh){
        mu = muReg;
      }else if(det1>=thLow){
        mu = muReg*lambda1 + mu1*(1.0f-lambda1);          
      }else{
        mu = mu1;
      }      
    }else if(det0>=thLow){
      if(det1>=thHigh){
        mu = muReg*lambda0 + mu0*(1.0f-lambda0);
      }else if(det1>=thLow){
        mu = ( (muReg*lambda1 + mu1*(1.0f-lambda1))*lambda0 + (mu0*lambda1 + muReg*(1.0f-lambda1))*(1.0f-lambda0) )*0.5f+
             ( (muReg*lambda0 + mu0*(1.0f-lambda0))*lambda1 + (mu1*lambda0 + muReg*(1.0f-lambda0))*(1.0f-lambda1) )*0.5f;
      }else{
        mu = mu1*lambda0 + muReg*(1.0f-lambda0);
      }      
    }else{
      if(det1>=thHigh){
        mu = mu0;
      }else if(det1>=thLow){
        mu = mu0*lambda1 + muReg*(1.0f-lambda1);
      }else{
        mu = muReg;
      }            
    }
  */  
/*     ---------------------------------------------
l1=1 |    mu0         | muReg+mu0   |   muReg    |
     ---------------------------------------------
l1   |    mu0+muReg   |             |            |    
     ---------------------------------------------
l1=0 |    muReg       | mu1+muReg   |   mu1      |
     ---------------------------------------------
            l0 = 0       l0             l0=1    
  */  
    
    
    //covarianceMatrices0[k].Print();
    //printf("%1.20f \n",det0);
    
    
    //printf("%1.20f %1.20f %1.20f\n",det0,det1,detDiff);
    
    if(keepAlignment){
      for (int j = 0; j < yIndicesCount; j++)
        resultData(k, yIndices[j]) = mu(j);
    }else{
  		for (int j = 0; j < yIndicesCount; j++)
  			resultData(k, j) = mu(j);
    }
	}
  if(pdiff)
    printf("Regr: MinDiff %1.20f MaxDiff %1.20f\n",minDiff,maxDiff); 
	return true;
}

bool Regression::LoadData(const char fileName[], Matrix &result)
{
	int rowSize, columnSize;
	std::ifstream file(fileName);
	if (!file.is_open())
		return false;
	file >> rowSize >> columnSize;
	result.Resize(rowSize, columnSize);
	for (int j = 0; j < rowSize; j++)
	{
		for (int i = 0; i < columnSize; i++)
			file >> result(j, i);
	}
	return true;
}

inline bool Regression::Create(int matrixSize, int covarianceMatricesCount)
{
	covarianceMatrices =  new Matrix [covarianceMatricesCount];
	this->covarianceMatricesCount = covarianceMatricesCount;
	for (int i = 0; i < covarianceMatricesCount; i++)
		covarianceMatrices[i].Resize(matrixSize, matrixSize, false);
  smoothCovMatrix.Resize(matrixSize, matrixSize, false);
  covMatInRow.Resize(covarianceMatricesCount,matrixSize*matrixSize);
	return false;
}

inline void Regression::Release()
{
	if (covarianceMatrices != NULL)
	{
		delete [] covarianceMatrices;
		covarianceMatrices = NULL;
	}
  covarianceMatricesCount = 0;
}

bool Regression::LocallyWeightedRegression(Matrix &inData, Matrix &outData, int xIndices[], int yIndices[], int xIndicesCount, int yIndicesCount, const Matrix &spam, bool covSquare)
{
	REALTYPE invDenominator;
	Matrix invSpam, xHat, sigmaTmp;
	Matrix sigmaXX, sigmaXY, sigmaYX, sigmaYY, invSigmaXX;
	Vector probabilityVector, muTmp, muHat;
	if (outData.ColumnSize() != xIndicesCount + yIndicesCount || spam.ColumnSize() != spam.RowSize() || spam.ColumnSize() != xIndicesCount)
		return false;
	Release();
	Create(outData.ColumnSize(), outData.RowSize());
	xHat.Resize(inData.RowSize(), xIndicesCount, false);
	sigmaTmp.Resize(inData.ColumnSize(), inData.ColumnSize(), false);
	sigmaXX.Resize(xIndicesCount, xIndicesCount, false);
	sigmaXY.Resize(xIndicesCount, yIndicesCount, false);
	sigmaYX.Resize(yIndicesCount, xIndicesCount, false);
	sigmaYY.Resize(yIndicesCount, yIndicesCount, false);
	invSigmaXX.Resize(xIndicesCount, xIndicesCount, false);
	probabilityVector.Resize(inData.RowSize(), false);
	muTmp.Resize(inData.ColumnSize(), false);
	muHat.Resize(inData.ColumnSize(), false);
	spam.Inverse(invSpam, &invDenominator);
	invDenominator = invDenominator < 0 ? -invDenominator : invDenominator;
	invDenominator = sqrtf(invDenominator);
	invDenominator *= powf(PIf * 2.0f, (REALTYPE)xIndicesCount * 0.5f);
	invDenominator = 1.0f / invDenominator;
	for (int k = 0; k < (int)outData.RowSize(); k++)
	{
		for (int j = 0; j < (int)inData.RowSize(); j++)
		{
			REALTYPE sum0 = 0;
			for (int i = 0; i < xIndicesCount; i++)
				xHat(j, i) = outData(k, i) - inData(j, xIndices[i]);
			for (int i = 0; i < xIndicesCount; i++)
			{
				REALTYPE sum1 = 0;
				for (int h = 0; h < xIndicesCount; h++)
					sum1 +=  xHat(j, i) * invSpam(h, i);
				sum0 += sum1 * xHat(j, i);
			}
			probabilityVector(j) = expf(sum0 * -0.5f) * invDenominator ;
		}
		probabilityVector /= probabilityVector.Sum();
		muTmp.Zero();
		for (int j = 0; j < (int)inData.RowSize(); j++)
		{
			for (int i = 0; i < (int)inData.ColumnSize(); i++)
				muTmp(i) += inData(j, i) * probabilityVector(j);
		}
		for (int j = 0; j < yIndicesCount; j++)
			outData(k, xIndicesCount + j) = muTmp(yIndices[j]);

		sigmaTmp.Zero();
		for (int j = 0; j < (int)inData.RowSize(); j++)
		{
			for (int i = 0; i < (int)inData.ColumnSize(); i++)
				muHat(i) = inData(j,i) - muTmp(i);
			for (int i = 0; i < (int)inData.ColumnSize(); i++)
			{
				for (int h = 0; h < (int)inData.ColumnSize(); h++)
					sigmaTmp(i,h) += muHat(i) * muHat(h) * probabilityVector(j);
			}
		}
		for (int j = 0; j < yIndicesCount; j++)
		{
			for (int i = 0; i < yIndicesCount; i++)
				sigmaYY(j, i) = sigmaTmp(yIndices[j], yIndices[i]);
			for (int i = 0; i < xIndicesCount; i++)
			{
				sigmaXY(i, j) = sigmaTmp(xIndices[i], yIndices[j]);
				sigmaYX(j, i) = sigmaTmp(yIndices[j], xIndices[i]);
			}
		}
		for (int j = 0; j < xIndicesCount; j++)
		{
			for (int i = 0; i < xIndicesCount; i++)
				sigmaXX(j, i) = sigmaTmp(xIndices[j], xIndices[i]);
		}
		sigmaXX.Inverse(invSigmaXX);
		covarianceMatrices[k] = sigmaYY - (sigmaYX * invSigmaXX * sigmaXY);
    if(covSquare){
      Matrix tmpCovMatrix(covarianceMatrices[k]);
      Matrix tmpCovMatrix2(covarianceMatrices[k]);
      const int msize = tmpCovMatrix.RowSize()-1;
      if(msize>=0){ 
        for(int kk=0;kk<msize;kk++){
          tmpCovMatrix(kk,msize) = 0.0f;
          tmpCovMatrix(msize,kk) = 0.0f;
        }
        tmpCovMatrix(msize,msize) = 1.0f;
      }
      tmpCovMatrix2.Mult(tmpCovMatrix,covarianceMatrices[k]);
      covarianceMatrices[k] *= 100000.0f;
      //covarianceMatrices[k].Sqrt();
    }
    Matrix id(covarianceMatrices[k]);
    id.Identity();
    id*=0.00001;
    covarianceMatrices[k] += id;
    
    Vector matVec(covarianceMatrices[k].Array(),yIndicesCount*yIndicesCount);
    covMatInRow.SetRow(matVec,k);
    
	}
	return true;
}

void Regression::SaveCovarianceMatrices(const char fileName[])
{
	std::ofstream file(fileName);
	file << covarianceMatrices[0].RowSize() << ' ' << covarianceMatrices[0].ColumnSize() << ' ' << covarianceMatricesCount << std::endl;
	for (int k = 0; k < covarianceMatricesCount; k++)
	{
		for (int j = 0; j < (int)covarianceMatrices[k].RowSize(); j++)
		{
			for (int i = 0; i < (int)covarianceMatrices[k].ColumnSize(); i++)
				file  << covarianceMatrices[k](j, i) << ' ';
			file << std::endl;
		}
		file << std::endl;
	}
	
}


void Regression::SmoothCovarianceMatrices(int window){
  if(covarianceMatricesCount<=0) return;
  if(window<=1) return;

  Matrix *tmpMat =  new Matrix[covarianceMatricesCount];
  for (int i = 0; i < covarianceMatricesCount; i++)
    tmpMat[i] = covarianceMatrices[i];
  


  const int sx = covarianceMatrices[0].ColumnSize();
  const int sy = covarianceMatrices[0].RowSize();
  const int hw = window/2;
  const int ws = hw*2; 
  for(int x=0;x<sx;x++){
    for(int y=0;y<sy;y++){    
      for(int i=hw;i<covarianceMatricesCount-hw;i++){
        REALTYPE mean = 0.0f;
        for(int j=-hw;j<hw-1;j++){
          mean += tmpMat[i+j](y,x);
        }
        mean /= REALTYPE(ws);
        covarianceMatrices[i](y,x) = mean; 
      }
    }
  }
  
  delete [] tmpMat;
}

void Regression::SmoothCovarianceMatrix(REALTYPE alpha){
  if(covarianceMatricesCount<=0) return;

  //covarianceMatrices[0] *= alpha; 
  //covarianceMatrices[0] += smoothCovMatrix*(1.0f-alpha);
  //smoothCovMatrix = covarianceMatrices[0];  
  covarianceMatrices[0] = smoothCovMatrix;
}

void Regression::SetSmoothCovarianceMatrix(){
  if(covarianceMatricesCount<=0) return;
  smoothCovMatrix = covarianceMatrices[0];
}


Matrix4& Regression::GetBestPlaneFromPoints(const Matrix &points, const Vector3& zero, const Vector3& firAxis, const Vector3& secAxis, Matrix4 &out){
  
  Matrix pointsT;
  points.Transpose(pointsT);
  
  Matrix AtA;
  pointsT.Mult(points,AtA);
    
  Vector Id(points.RowSize());
  Id+=1.0f;
  
  Vector off(3);
  pointsT.Mult(Id,off);
  
  Matrix AtAI;
  AtA.Inverse(AtAI);
  if(Matrix::IsInverseOk()){
    Vector resV;
    AtAI.Mult(off,resV);
    Vector3 res(resV.GetArray());
    
    Vector3 e1(res);
    Vector3 e2(secAxis);
    Vector3 e3;
    e1.Normalize();    
    if(e1.Dot(firAxis)<0.0f) e1 = -e1;
    e2.Normalize();
    e1.Cross(e2,e3);

    Matrix3 orient;
    orient.SetColumns(e1,e2,e3);
    orient.Normalize(0);
    
    Vector3 eZero(zero);
    REALTYPE lambda = (1.0f-zero.Dot(res))/res.Dot(res);
    eZero += res*lambda; 

    res*= lambda;
    out.Transformation(orient,eZero);    
  }else{
    out.Identity();
  }      
  return out;
}

  
  
