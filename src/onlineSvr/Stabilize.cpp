#ifndef STABILIZE_CPP
#define STABILIZE_CPP

#include <iostream>
#include "time.h"
#include "OnlineSVR.h"


namespace onlinesvr
{

	// Learning Operations
	int OnlineSVR::Stabilize ()
	{
		// Initialization
		time_t StartTime = time(NULL);
		int Flops = 0;
		this->ShowMessage("Starting Stabilize...\n",1);
		
		// Stabilizing
		int CurrentSampleIndex = 0;
		int TrueSampleIndex = 0;
		int LastSampleIndex = this->GetSamplesTrainedNumber()-1;
		while (CurrentSampleIndex<=LastSampleIndex) {
			if (! this->VerifyKKTConditions(CurrentSampleIndex)) {
				// Show Informations
				this->ShowMessage(" ",2);
				this->ShowMessage(" ",3);
				char Line[80];		
				sprintf(Line,"Stabilizing %d/%d",TrueSampleIndex+1,X->GetLengthRows());
				this->ShowMessage(Line,1);
				// Stabilizing
				Vector<double>* X = this->X->GetRowCopy(CurrentSampleIndex);
				double Y = this->Y->GetValue(CurrentSampleIndex);
				Flops += this->Unlearn(CurrentSampleIndex);
				Flops += this->Learn(X,Y);
				delete X;
				LastSampleIndex --;
			}
			else {
				CurrentSampleIndex ++;
			}
			TrueSampleIndex ++;
		}
		
		// Show Execution Time
		time_t EndTime = time(NULL);
		long LearningTime = static_cast<long>(EndTime-StartTime);
		this->ShowMessage(" ",2);
		this->ShowMessage(" ",3);
		char Line[256];
		if (this->VerifyKKTConditions())
			sprintf(Line, "\nStabilized %d elements correctly in %s.\n", X->GetLengthRows(), this->TimeToString(LearningTime));	
		else
			sprintf(Line, "\nStabilized %d elements in %s, but some cannot be stabilized.\n", X->GetLengthRows(), this->TimeToString(LearningTime));	
		this->ShowMessage(Line,1);

		return Flops;
	}

	bool OnlineSVR::VerifyKKTConditions() 
	{	
		Vector<double>* H = this->Margin(this->X,this->Y);
		bool ris = this->VerifyKKTConditions(H);
		delete H;
		return ris;
	}

	bool OnlineSVR::VerifyKKTConditions(Vector<double>* H) 
	{	
		int SampleIndex, SetName, SampleSetIndex;
		return this->VerifyKKTConditions(H, &SampleIndex,&SetName,&SampleSetIndex);	
	}

	bool OnlineSVR::VerifyKKTConditions(Vector<double>* H, int* SampleIndex, int* SetName, int* SampleSetIndex) 
	{	
		double Error;
		double C = this->C;
		double Epsilon = this->Epsilon;

		// Find Error
		if (this->AutoErrorTollerance)
			Error = Epsilon/10;
		else
			Error = this->ErrorTollerance;

		// Support Set
		int i;
		for (i=0; i<this->GetSupportSetElementsNumber(); i++) {
			(*SampleIndex) = this->SupportSetIndexes->GetValue(i);
			double Weightsi = this->Weights->GetValue(*SampleIndex);
			double Hi = H->GetValue(*SampleIndex);
			if (! ((OnlineSVR::IsContained(Weightsi, -C, 0, Error) && OnlineSVR::IsEquals(Hi, Epsilon, Error)) ||
				((OnlineSVR::IsContained(Weightsi, 0, C, Error) && OnlineSVR::IsEquals(Hi, -Epsilon, Error))))) {
				(*SetName) = this->SUPPORT_SET;			
				(*SampleSetIndex) = i;
				return false;
			}
		}

		// Error Set	
		for (i=0; i<this->GetErrorSetElementsNumber(); i++) {
			(*SampleIndex) = this->ErrorSetIndexes->GetValue(i);
			double Weightsi = this->Weights->GetValue(*SampleIndex);
			double Hi = H->GetValue(*SampleIndex);
			if (! ((OnlineSVR::IsEquals(Weightsi, -C, Error) && OnlineSVR::IsBigger(Hi, Epsilon, Error)) ||
				((OnlineSVR::IsEquals(Weightsi, C, Error) && OnlineSVR::IsLesser(Hi, -Epsilon, Error))))) {
				(*SetName) = this->ERROR_SET;			
				(*SampleSetIndex) = i;
				return false;
			}
		}

		// Remaining Set	
		for (i=0; i<this->GetRemainingSetElementsNumber(); i++) {
			(*SampleIndex) = this->RemainingSetIndexes->GetValue(i);
			double Weightsi = this->Weights->GetValue(*SampleIndex);
			double Hi = H->GetValue(*SampleIndex);
			if (! (OnlineSVR::IsEquals(Weightsi, 0, Error) && OnlineSVR::IsContained(Hi, -Epsilon, +Epsilon, Error))) {
				(*SetName) = this->REMAINING_SET;			
				(*SampleSetIndex) = i;
				return false;
			}
		}
		
		return true;
	}

	bool OnlineSVR::VerifyKKTConditions(int SampleIndex) 
	{	
		double Error;
		double C = this->C;
		double Epsilon = this->Epsilon;
		double Hi = this->Margin(this->X->GetRowRef(SampleIndex),this->Y->GetValue(SampleIndex));
		double Weightsi = this->Weights->GetValue(SampleIndex);

		// Find Error
		if (this->AutoErrorTollerance)
			Error = Epsilon/10;
		else
			Error = this->ErrorTollerance;

		// Support Set		
		if (this->SupportSetIndexes->Find(SampleIndex)>=0) {
			if (! ((OnlineSVR::IsContained(Weightsi, -C, 0, Error) && OnlineSVR::IsEquals(Hi, Epsilon, Error)) ||
			      ((OnlineSVR::IsContained(Weightsi, 0, C, Error) && OnlineSVR::IsEquals(Hi, -Epsilon, Error)))))
				return false;
			else
				return true;
		}

		// Error Set
		if (this->ErrorSetIndexes->Find(SampleIndex)>=0) {		
			if (! ((OnlineSVR::IsEquals(Weightsi, -C, Error) && OnlineSVR::IsBigger(Hi, Epsilon, Error)) ||
				  ((OnlineSVR::IsEquals(Weightsi, C, Error) && OnlineSVR::IsLesser(Hi, -Epsilon, Error)))))
				return false;
			else
				return true;
		}

		// Remaining Set	
		if (this->RemainingSetIndexes->Find(SampleIndex)>=0) {
			if (! (OnlineSVR::IsEquals(Weightsi, 0, Error) && OnlineSVR::IsContained(Hi, -Epsilon, +Epsilon, Error)))
				return false;
			else
				return true;
		}
		
		return true;
	}
	
	bool OnlineSVR::IsEquals (double Value1, double Value2, double Error)
	{
		if (fabs(Value1-Value2)<=Error)
			return true;
		else
			return false;
	}

	bool OnlineSVR::IsLesser (double Value1, double Value2, double Error)
	{
		if (Value1-Error<=Value2)
			return true;
		else
			return false;
	}

	bool OnlineSVR::IsBigger (double Value1, double Value2, double Error)
	{
		if (Value1+Error>=Value2)
			return true;
		else
			return false;
	}	
	bool OnlineSVR::IsContained (double Value, double From, double To, double Error)
	{
		if (From-Error<=Value && Value<=To+Error)
			return true;
		else
			return false;
	}

}
	
#endif
