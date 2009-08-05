#ifndef ONLINE_SVR_CPP
#define ONLINE_SVR_CPP

#include "OnlineSVR.h"


namespace onlinesvr
{

	// Initialization
	OnlineSVR::OnlineSVR ()
	{
		this->C = 30;
		this->Epsilon = 0.1;
		this->KernelType = this->KERNEL_RBF;
		this->KernelParam = 30;
		this->KernelParam2 = 0;
		this->AutoErrorTollerance = true;
		this->ErrorTollerance = this->Epsilon/10;
		this->StabilizedLearning = true;
		this->SaveKernelMatrix = true;
		this->Verbosity = 1;
		this->X = new Matrix<double>();
		this->Y = new Vector<double>();	
		this->Weights = new Vector<double>();
		this->Bias = 0;
		this->SupportSetIndexes = new Vector<int>();
		this->ErrorSetIndexes = new Vector<int>();
		this->RemainingSetIndexes = new Vector<int>();
		this->R = new Matrix<double>();
		this->KernelMatrix = new Matrix<double>();
		this->SamplesTrainedNumber = 0;		
	}

	OnlineSVR::~OnlineSVR ()
	{
		this->Clear();
	}

	void OnlineSVR::Clear ()
	{
		this->X->Clear();
		this->Y->Clear();
		this->Weights->Clear();
		this->Bias = 0;
		this->SupportSetIndexes->Clear();
		this->ErrorSetIndexes->Clear();
		this->RemainingSetIndexes->Clear();
		this->R->Clear();
		this->KernelMatrix->Clear();
	}

	OnlineSVR* OnlineSVR::Clone ()
	{
		OnlineSVR* SVR = new OnlineSVR();
		SVR->C = this->C;
		SVR->Epsilon = this->Epsilon;
		SVR->KernelType = this->KernelType;
		SVR->KernelParam = this->KernelParam;
		SVR->KernelParam2 = this->KernelParam2;
		SVR->AutoErrorTollerance = this->AutoErrorTollerance;
		SVR->ErrorTollerance = this->ErrorTollerance;
		SVR->StabilizedLearning = this->StabilizedLearning;
		SVR->SaveKernelMatrix = this->SaveKernelMatrix;
		SVR->Verbosity = this->Verbosity;
		SVR->X = this->X->Clone();
		SVR->Y = this->Y->Clone();
		SVR->Weights = this->Weights->Clone();
		SVR->Bias = this->Bias;
		SVR->SupportSetIndexes = this->SupportSetIndexes->Clone();
		SVR->ErrorSetIndexes = this->ErrorSetIndexes->Clone();
		SVR->RemainingSetIndexes = this->RemainingSetIndexes->Clone();
		SVR->R = this->R->Clone();
		SVR->KernelMatrix = this->KernelMatrix->Clone();
		SVR->SamplesTrainedNumber = this->SamplesTrainedNumber;
		return SVR;
	}
		

	// Attributes Operations
	double OnlineSVR::GetC ()
	{
		return this->C;
	}

	void OnlineSVR::SetC (double C)
	{
		this->C = C;
	}

	double OnlineSVR::GetEpsilon ()
	{
		return this->Epsilon;
	}

	void OnlineSVR::SetEpsilon (double Epsilon)
	{
		this->Epsilon = Epsilon;
	}

	int OnlineSVR::GetKernelType ()
	{
		return this->KernelType;
	}

	void OnlineSVR::SetKernelType (int KernelType)
	{
		this->KernelType = KernelType;
		if (this->SaveKernelMatrix) {
			this->BuildKernelMatrix();
		}
	}

	double OnlineSVR::GetKernelParam ()
	{
		return this->KernelParam;
	}

	void OnlineSVR::SetKernelParam (double KernelParam)
	{
		this->KernelParam = KernelParam;
		if (this->SaveKernelMatrix) {
			this->BuildKernelMatrix();
		}
	}

	double OnlineSVR::GetKernelParam2 ()
	{
		return this->KernelParam2;
	}

	void OnlineSVR::SetKernelParam2 (double KernelParam2)
	{
		this->KernelParam2 = KernelParam2;
		if (this->SaveKernelMatrix) {
			this->BuildKernelMatrix();
		}
	}

	bool OnlineSVR::GetAutoErrorTollerance ()
	{
		return this->AutoErrorTollerance;
	}

	void OnlineSVR::SetAutoErrorTollerance (bool AutoErrorTollerance)
	{
		this->AutoErrorTollerance = AutoErrorTollerance;
	}

	double OnlineSVR::GetErrorTollerance ()
	{
		return this->ErrorTollerance;
	}

	void OnlineSVR::SetErrorTollerance (double ErrorTollerance)
	{
		this->ErrorTollerance = ErrorTollerance;
	}

	int OnlineSVR::GetSamplesTrainedNumber ()
	{
		return this->SamplesTrainedNumber;
	}

	int OnlineSVR::GetSupportSetElementsNumber ()
	{
		return this->SupportSetIndexes->GetLength();
	}

	int OnlineSVR::GetErrorSetElementsNumber ()
	{
		return this->ErrorSetIndexes->GetLength();
	}

	int OnlineSVR::GetRemainingSetElementsNumber ()
	{
		return this->RemainingSetIndexes->GetLength();
	}

	int OnlineSVR::GetVerbosity ()
	{
		return this->Verbosity;
	}

	void OnlineSVR::SetVerbosity (int Verbosity)
	{
		this->Verbosity = Verbosity;
	}

	bool OnlineSVR::GetStabilizedLearning ()
	{
		return StabilizedLearning;
	}

	void OnlineSVR::SetStabilizedLearning (bool StabilizedLearning)
	{
		this->StabilizedLearning = StabilizedLearning;
	}
	bool OnlineSVR::GetSaveKernelMatrix ()
	{
		return SaveKernelMatrix;
	}

	void OnlineSVR::SetSaveKernelMatrix (bool SaveKernelMatrix)
	{
		if (!this->SaveKernelMatrix && SaveKernelMatrix) {
			this->BuildKernelMatrix();
		}
		else if (!SaveKernelMatrix) {
			this->KernelMatrix->Clear();
		}
		this->SaveKernelMatrix = SaveKernelMatrix;
	}

	Vector<int>* OnlineSVR::GetSupportSetIndexes()
	{
		return this->SupportSetIndexes;
	}

	Vector<int>* OnlineSVR::GetErrorSetIndexes()
	{
		return this->ErrorSetIndexes;
	}
	Vector<int>* OnlineSVR::GetRemainingSetIndexes()
	{
		return this->RemainingSetIndexes;
	}

		

	// Predict/Margin Operations
	double OnlineSVR::Predict (int Index)
	{		
		double PredictedValue = 0;
		for (int i=0; i<this->GetSamplesTrainedNumber(); i++) {		
			PredictedValue += this->Weights->GetValue(i) * this->KernelMatrix->GetValue(i,Index);
		}

		// Bias
		PredictedValue += this->Bias;
		return PredictedValue;
	}

	double OnlineSVR::Predict (Vector<double>* V)
	{	
		// Trained Elements
		double PredictedValue = 0;
		for (int i=0; i<this->GetSamplesTrainedNumber(); i++) {		
			PredictedValue += this->Weights->GetValue(i) * this->Kernel(this->X->GetRowRef(i),V);
		}

		// Bias
		PredictedValue += this->Bias;
		return PredictedValue;
	}

	Vector<double>* OnlineSVR::Predict (Matrix<double>* X)
	{
		if (this->X==X && this->SaveKernelMatrix) {
			Vector<double>* V = new Vector<double>(X->GetLengthRows());
			for (int i=0; i<X->GetLengthRows(); i++) {
				V->Add(this->Predict(i));
			}
			return V;
		}
		else {
			Vector<double>* V = new Vector<double>(X->GetLengthRows());
			for (int i=0; i<X->GetLengthRows(); i++) {
				V->Add(this->Predict(X->GetRowRef(i)));
			}
			return V;
		}
	}

	double OnlineSVR::Margin (Vector<double>* X, double Y)
	{
		return this->Predict(X)-Y;
	}

	Vector<double>* OnlineSVR::Margin (Matrix<double>* X, Vector<double>* Y)
	{
		Vector<double>* V = this->Predict(X);	
		V->SubtractVector(Y);
		return V;
	}

	double OnlineSVR::Predict (double* X, int ElementsSize)
	{
		Vector<double>* V = new Vector<double>(X, ElementsSize);
		double PredictedValue = this->Predict(V);
		return PredictedValue;
	}

	double* OnlineSVR::Predict (double** X, int ElementsNumber, int ElementsSize)
	{
		Matrix<double>* M = new Matrix<double>(X,ElementsNumber,ElementsSize);
		Vector<double>* V = this->Predict(M);
		delete M;
		return V->Values;
	}

	double OnlineSVR::Margin (double* X, double Y, int ElementsSize)
	{
		return this->Predict(X,ElementsSize)-Y;
	}

	double* OnlineSVR::Margin (double** X, double* Y, int ElementsNumber, int ElementsSize)
	{
		Vector<double>* V1 = new Vector<double>(this->Predict(X,ElementsNumber,ElementsSize),ElementsNumber);
		Vector<double>* V2 = new Vector<double>(Y,ElementsNumber);
		V1->SubtractVector(V2);
		delete V2;
		return V1->Values;
	}

		
	// Other Kernel Operations
	Matrix<double>* OnlineSVR::Q (Vector<int>* V1, Vector<int>* V2)
	{
		if (!SaveKernelMatrix) {
			Matrix<double>* M = new Matrix<double>();
			for (int i=0; i<V1->GetLength(); i++) {
				Vector<double>* V = new Vector<double>(V1->GetLength());
				for (int j=0; j<V1->GetLength(); j++) {
					V->Add(this->Kernel(this->X->GetRowRef(V1->GetValue(i)),this->X->GetRowRef(V2->GetValue(j))));
				}
				M->AddRowRef(V);
			}
			return M;
		}
		else {
			Matrix<double>* M = new Matrix<double>();
			for (int i=0; i<V1->GetLength(); i++) {
				Vector<double>* V = new Vector<double>(V1->GetLength());
				for (int j=0; j<V1->GetLength(); j++) {
					V->Add(this->KernelMatrix->GetValue(V1->GetValue(i), V1->GetValue(j)));
				}
				M->AddRowRef(V);
			}
			return M;
		}
	}

	Matrix<double>* OnlineSVR::Q (Vector<int>* V)
	{
		if (!SaveKernelMatrix) {
			Matrix<double>* M = new Matrix<double>();
			for (int i=0; i<this->GetSamplesTrainedNumber(); i++) {
				Vector<double>* V2 = new Vector<double>(V->GetLength());
				for (int j=0; j<V->GetLength(); j++) {
					V2->Add(this->Kernel(this->X->GetRowRef(i),this->X->GetRowRef(V->GetValue(j))));
				}
				M->AddRowRef(V2);
			}
			return M;
		}
		else {
			Matrix<double>* M = new Matrix<double>();
			for (int i=0; i<this->GetSamplesTrainedNumber(); i++) {
				Vector<double>* V2 = new Vector<double>(V->GetLength());
				for (int j=0; j<V->GetLength(); j++) {
					V2->Add(this->KernelMatrix->GetValue(i,V->GetValue(j)));
				}
				M->AddRowRef(V2);
			}
			return M;
		}
	}

	Vector<double>* OnlineSVR::Q (Vector<int>* V, int Index)
	{
		if (!SaveKernelMatrix) {
			Vector<double>* V2 = new Vector<double>(V->GetLength());
			for (int i=0; i<V->GetLength(); i++) {
				V2->Add(this->Kernel(this->X->GetRowRef(V->GetValue(i)),this->X->GetRowRef(Index)));
			}
			return V2;
		}
		else {
			Vector<double>* V2 = new Vector<double>(V->GetLength());
			for (int i=0; i<V->GetLength(); i++) {
				V2->Add(this->KernelMatrix->GetValue(V->GetValue(i),Index));
			}
			return V2;
		}
	}

	Vector<double>* OnlineSVR::Q (int Index)
	{
		if (!SaveKernelMatrix) {
			Vector<double>* V = new Vector<double>(this->GetSamplesTrainedNumber());
			for (int i=0; i<this->GetSamplesTrainedNumber(); i++) {
				V->Add(this->Kernel(this->X->GetRowRef(i),this->X->GetRowRef(Index)));
			}
			return V;
		}
		else {
			Vector<double>* V = new Vector<double>(this->GetSamplesTrainedNumber());
			for (int i=0; i<this->GetSamplesTrainedNumber(); i++) {
				V->Add(this->KernelMatrix->GetValue(i,Index));
			}
			return V;
		}
	}

	double OnlineSVR::Q (int Index1, int Index2)
	{
		if (!SaveKernelMatrix) {
			return this->Kernel(this->X->GetRowRef(Index1),this->X->GetRowRef(Index2));
		}
		else {
			return this->KernelMatrix->GetValue(Index1,Index2);
		}
	}

		
	// Matrix R Operations
	void OnlineSVR::AddSampleToR (int SampleIndex, int SampleOldSet, Vector<double>* Beta, Vector<double>* Gamma)
	{
		if (this->R->GetLengthRows()==0) {
			Vector<double>* V1 = new Vector<double>(2);
			V1->Add(-this->Q(SampleIndex,SampleIndex));
			V1->Add(1);
			this->R->AddRowRef(V1);
			Vector<double>* V2 = new Vector<double>(2);
			V2->Add(1);
			V2->Add(0);
			this->R->AddRowRef(V2);
		}
		else {
			Vector<double>* NewBeta; 
			Vector<double>* NewGamma;
			if (SampleOldSet==this->ERROR_SET || SampleOldSet==this->REMAINING_SET) {
				// TODO: We only need of beta and Gamma(SampleIndex)
				int LastSupport = this->SupportSetIndexes->GetValue(this->GetSupportSetElementsNumber()-1);
				this->SupportSetIndexes->RemoveAt(this->GetSupportSetElementsNumber()-1);
				NewBeta = this->FindBeta(SampleIndex);
				NewGamma = Gamma;
				NewGamma->SetValue(SampleIndex, this->FindGammaSample(NewBeta, SampleIndex));
				this->SupportSetIndexes->Add(LastSupport);			
			} 
			else {
				NewBeta = Beta->Clone();
				NewGamma = Gamma;
			}
			Vector<double>* Zeros = new Vector<double>(this->R->GetLengthCols()+1);
			for (int i=0; i<this->R->GetLengthCols(); i++) {
				Zeros->Add(0);
			}
			this->R->AddColCopy(Zeros);
			Zeros->Add(0);
			this->R->AddRowRef(Zeros);
			if (NewGamma->GetValue(SampleIndex)!=0) {			
				NewBeta->Add(1);
				Matrix<double>* BetaMatrix = BetaMatrix->ProductVectorVector(NewBeta,NewBeta);			
				BetaMatrix->DivideScalar(NewGamma->GetValue(SampleIndex));
				this->R->SumMatrix(BetaMatrix);
				delete BetaMatrix;
			}
			delete NewBeta;
 		}
	}

	void OnlineSVR::RemoveSampleFromR (int SampleIndex)
	{
		Vector<double>* Row = this->R->GetRowCopy(SampleIndex+1);	
		Row->RemoveAt(SampleIndex+1);
		Vector<double>* Col = this->R->GetColCopy(SampleIndex+1);
		Col->RemoveAt(SampleIndex+1);
		double Rii = R->GetValue(SampleIndex+1,SampleIndex+1);
		this->R->RemoveRow(SampleIndex+1);
		this->R->RemoveCol(SampleIndex+1);
		if (Rii!=0) {
			Matrix<double>* RVariations = RVariations->ProductVectorVector(Col,Row);
			RVariations->DivideScalar(Rii);
			this->R->SubtractMatrix(RVariations);
			delete RVariations;
		}
		delete Row;
		delete Col;
		if (this->R->GetLengthRows()==1) {
			this->R->Clear();
		}
	}

	Vector<double>* OnlineSVR::FindBeta (int SampleIndex)
	{
		Vector<double>* Qsi = this->Q(this->SupportSetIndexes,SampleIndex);
		Qsi->AddAt(1,0);
		Vector<double>* Beta = this->R->ProductVector(Qsi);
		Beta->ProductScalar(-1);
		delete Qsi;
		return Beta;
	}

	Vector<double>* OnlineSVR::FindGamma (Vector<double>* Beta, int SampleIndex)
	{	
		if (this->GetSupportSetElementsNumber()==0) {		
			Vector<double>* Gamma = new Vector<double>(this->GetSamplesTrainedNumber());
			for (int i=0; i<this->GetSamplesTrainedNumber(); i++) {
				Gamma->Add(1);
			}
			return Gamma;
		}
		else {
			Vector<double>* Qxi = this->Q(SampleIndex);
			Matrix<double>* Qxs = this->Q(this->SupportSetIndexes);
			Vector<double>* Ones = new Vector<double>(this->GetSamplesTrainedNumber());
			for (int i=0; i<this->GetSamplesTrainedNumber(); i++) {
				Ones->Add(1);
			}
			Qxs->AddColCopyAt(Ones,0);
			Vector<double>* Gamma = Qxs->ProductVector(Beta);
			Gamma->SumVector(Qxi);
			delete Ones;
			delete Qxi;
			delete Qxs;
			return Gamma;
		}
	}

	double OnlineSVR::FindGammaSample (Vector<double>* Beta, int SampleIndex)
	{	
		if (this->GetSupportSetElementsNumber()==0) {		
			return 1;
		}
		else {
			double Qii = this->Q(SampleIndex,SampleIndex);
			Vector<double>* Qsi = this->Q(this->SupportSetIndexes,SampleIndex);
			Qsi->AddAt(1,0);
			double Gamma = Qii + Beta->ProductVectorScalar(Qsi);
			delete Qsi;
			return Gamma;
		}
	}

		
	// KernelMatrix Operations
	void OnlineSVR::AddSampleToKernelMatrix (Vector<double>* X)
	{
		Vector<double>* V = new Vector<double>();
		if (this->SamplesTrainedNumber>1) {
			for (int i=0; i<this->KernelMatrix->GetLengthRows(); i++)
				V->Add(this->Kernel(this->X->GetRowRef(i),X));
			this->KernelMatrix->AddColCopy(V);
		}
		V->Add(this->Kernel(X,X));
		this->KernelMatrix->AddRowRef(V);
	}

	void OnlineSVR::RemoveSampleFromKernelMatrix (int SampleIndex)
	{
		if (this->KernelMatrix->GetLengthRows()>1) {
			this->KernelMatrix->RemoveRow(SampleIndex);	
			this->KernelMatrix->RemoveCol(SampleIndex);
		}
		else {
			this->KernelMatrix->RemoveRow(SampleIndex);	
		}
	}

	void OnlineSVR::BuildKernelMatrix ()
	{
		if (this->KernelMatrix->GetLengthRows()!=this->SamplesTrainedNumber) {
			KernelMatrix->Clear();
			for (int i=0; i<this->SamplesTrainedNumber; i++)
				this->AddSampleToKernelMatrix(this->X->GetRowRef(i));
		}
		else {
			for (int i=0; i<this->SamplesTrainedNumber; i++) {
				for (int j=0; j<=i; j++) {
					double Value = this->Kernel(this->X->GetRowRef(i),this->X->GetRowRef(j));
					this->KernelMatrix->SetValue(i, j, Value);
					this->KernelMatrix->SetValue(j, i, Value);
				}
			}
		}
	}

}
	
#endif
