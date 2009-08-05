#ifndef TRAIN_CPP
#define TRAIN_CPP

#include "OnlineSVR.h"


namespace onlinesvr
{

	// Variation Operations
	double OnlineSVR::FindVariationLc1 (Vector<double>* H, Vector<double>* Gamma, int SampleIndex, int Direction)
	{
		double Weightc = this->Weights->GetValue(SampleIndex);
		double Hc = H->GetValue(SampleIndex);
		double Gammac = Gamma->GetValue(SampleIndex);
		double Epsilon = this->Epsilon;

		if (Gammac==0)
			return Direction*INF;
		else if ((Weightc>0) || ((Weightc==0) && (Hc<0)))
			return (-Hc -Epsilon) / Gammac;
		else
			return (-Hc +Epsilon) / Gammac;		

	}

	double OnlineSVR::FindVariationLc2 (int SampleIndex, int Direction)
	{
		double Weightsc = this->Weights->GetValue(SampleIndex);
		double C = this->C;

		if (this->GetSupportSetElementsNumber()==0)
			return Direction * INF;
		else if (Direction>0)
			return -Weightsc +C;
		else
			return -Weightsc -C;
	}

	double OnlineSVR::FindVariationLc (int SampleIndex)
	{
		return -this->Weights->GetValue(SampleIndex);
	}

	Vector<double>* OnlineSVR::FindVariationLs (Vector<double>* H, Vector<double>* Beta, int Direction)
	{
		Vector<double>* Ls = new Vector<double>(this->GetSupportSetElementsNumber());
		C = this->C;		

		for (int i=0; i<this->GetSupportSetElementsNumber(); i++) {			
			
			double Weightsi = this->Weights->GetValue(this->SupportSetIndexes->GetValue(i));
			double Hi = H->GetValue(this->SupportSetIndexes->GetValue(i));
			double Betai = Beta->GetValue(i+1);
			double Lsi = Direction * INF;

			if (Betai==0)
				Lsi = Direction*INF;
			else if (Direction*Betai>0) {
				if (Hi>0) { // Hi == Epsilon
					if (Weightsi<-C)
						Lsi = (-Weightsi -C) / Betai;
					else if (Weightsi <=0)
						Lsi = -Weightsi / Betai;
					else
						Lsi = Direction*INF;
				} else { // Hi == -Epsilon
					if (Weightsi<0)
						Lsi = -Weightsi / Betai;
					else if (Weightsi <=C)
						Lsi = (-Weightsi +C) / Betai;						
					else
						Lsi = Direction*INF;
				}
			} else {
				if (Hi>0) { // Hi == Epsilon
					if (Weightsi<-C)
						Lsi = Direction*INF;						
					else if (Weightsi <=0)
						Lsi = (-Weightsi -C) / Betai;						
					else
						Lsi = -Weightsi / Betai;
				} else { // Hi == -Epsilon
					if (Weightsi<0)
						Lsi = Direction*INF;						
					else if (Weightsi <=C)
						Lsi = -Weightsi / Betai;						
					else
						Lsi = (-Weightsi +C) / Betai;
				}
			}
				
			
			
			Ls->Add(Lsi);
		}

		return Ls;
	}

	Vector<double>* OnlineSVR::FindVariationLe (Vector<double>* H, Vector<double>* Gamma, int Direction)
	{
		Vector<double>* Le = new Vector<double>(this->GetErrorSetElementsNumber());
		Epsilon = this->Epsilon;

		for (int i=0; i<this->GetErrorSetElementsNumber(); i++) {
		
			double Weightsi = this->Weights->GetValue(this->ErrorSetIndexes->GetValue(i));
			double Hi = H->GetValue(this->ErrorSetIndexes->GetValue(i));
			double Gammai = Gamma->GetValue(this->ErrorSetIndexes->GetValue(i));
			double Lei = Direction * INF;	

			if (Gammai == 0)
				Lei = Direction * INF;
			else if (Direction*Gammai>0) {
				if (Weightsi>0) { // Weightsi == C
					if (Hi<-Epsilon)
						Lei = (-Hi -Epsilon) / Gammai;
					else
						Lei = Direction * INF;
				} else { // Weightsi == -C
					if (Hi<Epsilon)
						Lei = (-Hi +Epsilon) / Gammai;
					else
						Lei = Direction * INF;					
				}
			} else {
				if (Weightsi>0) { // Weightsi == C
					if (Hi>-Epsilon)
						Lei = (-Hi -Epsilon) / Gammai;
					else
						Lei = Direction * INF;
				} else { // Weightsi == -C
					if (Hi>Epsilon)
						Lei = (-Hi +Epsilon) / Gammai;
					else
						Lei = Direction * INF;					
				}
			}
			Le->Add(Lei);
		}

		return Le;
	}

	Vector<double>* OnlineSVR::FindVariationLr (Vector<double>* H, Vector<double>* Gamma, int Direction)
	{
		Vector<double>* Lr = new Vector<double>(this->GetRemainingSetElementsNumber());
		Epsilon = this->Epsilon;

		for (int i=0; i<this->GetRemainingSetElementsNumber(); i++) {
		
			double Hi = H->Values[this->RemainingSetIndexes->Values[i]];
			double Gammai = Gamma->Values[this->RemainingSetIndexes->Values[i]];
			double Lri = Direction * INF;

			if (Gammai == 0) {
				Lri = Direction * INF;
			}
			else if (Direction*Gammai>0) {
				if (Hi < -Epsilon) {
					Lri = (-Hi -Epsilon) / Gammai;
				}
				else if (Hi < +Epsilon) {
					Lri = (-Hi +Epsilon) / Gammai;
				}
				else {
					Lri = Direction * INF;
				}
			}
			else { 
				if (Hi > +Epsilon) {
					Lri = (-Hi +Epsilon) / Gammai;
				}
				else if (Hi > -Epsilon) {
					Lri = (-Hi -Epsilon) / Gammai;
				}
				else {
					Lri = Direction * INF;
				}
			}

			Lr->Add(Lri);
		}

		return Lr;
	}

	void OnlineSVR::FindLearningMinVariation (Vector<double>* H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double* MinVariation, int* MinIndex, int* Flag)
	{

		// Find Samples Variations
		int Direction = SIGN(-H->Values[SampleIndex]);
		double Lc1 = this->FindVariationLc1 (H, Gamma, SampleIndex, Direction);
		double Lc2 = this->FindVariationLc2 (SampleIndex, Direction);
		Vector<double>* Ls = this->FindVariationLs (H, Beta, Direction);
		Vector<double>* Le = this->FindVariationLe (H, Gamma, Direction);
		Vector<double>* Lr = this->FindVariationLr (H, Gamma, Direction);
		if (this->Verbosity>2) {
			this->ShowVariations(H, Beta, Gamma, SampleIndex, Lc1, Lc2, Ls, Le, Lr, this->OPERATION_LEARNING);
		}

		// Find Min Variation
		double MinLsValue, MinLeValue, MinLrValue;
		int MinLsIndex, MinLeIndex, MinLrIndex;
		if (this->GetSupportSetElementsNumber()>0)
			Ls->MinAbs (&MinLsValue, &MinLsIndex);
		else
			MinLsValue = Direction * INF;
		if (this->GetErrorSetElementsNumber()>0)
			Le->MinAbs (&MinLeValue, &MinLeIndex);
		else
			MinLeValue = Direction * INF;
		if (this->GetRemainingSetElementsNumber()>0)
			Lr->MinAbs (&MinLrValue, &MinLrIndex);
		else
			MinLrValue = Direction * INF;
		Vector<double>* Variations = new Vector<double>(5);
		Variations->Add(Lc1);
		Variations->Add(Lc2);
		Variations->Add(MinLsValue);
		Variations->Add(MinLeValue);
		Variations->Add(MinLrValue);
		Variations->MinAbs(MinVariation, Flag);

		// Find Sample Index Variation
		(*MinVariation) *= Direction;
		(*Flag) ++;
		switch (*Flag) {
			case 1:
			case 2:
				(*MinIndex) = 0;
				 break;
			case 3:
				(*MinIndex) = MinLsIndex;
				break;
			case 4:
				(*MinIndex) = MinLeIndex;
				break;
			case 5:
				(*MinIndex) = MinLrIndex;
				break;
		}

		// Clear
		delete Ls;
		delete Le;
		delete Lr;
		delete Variations;
	}

	void OnlineSVR::FindUnlearningMinVariation (Vector<double>* H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double* MinVariation, int* MinIndex, int* Flag)
	{

		// Find Samples Variations
		int Direction = SIGN(-this->Weights->Values[SampleIndex]);
		double Lc = this->FindVariationLc (SampleIndex);
		Vector<double>* Ls = this->FindVariationLs (H, Beta, Direction);
		Vector<double>* Le = this->FindVariationLe (H, Gamma, Direction);
		Vector<double>* Lr = this->FindVariationLr (H, Gamma, Direction);
		if (this->Verbosity>2) {
			this->ShowVariations(H, Beta, Gamma, SampleIndex, Lc, 0, Ls, Le, Lr, this->OPERATION_UNLEARNING);
		}

		// Find Min Variation
		double MinLsValue, MinLeValue, MinLrValue;
		int MinLsIndex, MinLeIndex, MinLrIndex;
		if (this->GetSupportSetElementsNumber()>0)
			Ls->MinAbs (&MinLsValue, &MinLsIndex);
		else
			MinLsValue = Direction * INF;
		if (this->GetErrorSetElementsNumber()>0)
			Le->MinAbs (&MinLeValue, &MinLeIndex);
		else
			MinLeValue = Direction * INF;
		if (this->GetRemainingSetElementsNumber()>0)
			Lr->MinAbs (&MinLrValue, &MinLrIndex);
		else
			MinLrValue = Direction * INF;
		Vector<double>* Variations = new Vector<double>(5);
		Variations->Add(Lc);
		Variations->Add(Direction*INF);
		Variations->Add(MinLsValue);
		Variations->Add(MinLeValue);
		Variations->Add(MinLrValue);
		Variations->MinAbs(MinVariation, Flag);

		// Find Sample Index Variation
		(*MinVariation) *= Direction;
		(*Flag) ++;
		switch (*Flag) {
			case 1:
			case 2:
				(*MinIndex) = 0;
				 break;
			case 3:
				(*MinIndex) = MinLsIndex;
				break;
			case 4:
				(*MinIndex) = MinLeIndex;
				break;
			case 5:
				(*MinIndex) = MinLrIndex;
				break;
		}

		// Clear
		delete Ls;
		delete Le;
		delete Lr;
		delete Variations;
	}

		
	// Matrix R Operations
	void OnlineSVR::UpdateWeightsAndBias (Vector<double>** H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double MinVariation)
	{

		// Update Weights and Bias
		if (this->GetSupportSetElementsNumber()>0) {

			// Update Weights New Sample
			this->Weights->Values[SampleIndex] += MinVariation;

			// Update Bias
			Vector<double>* DeltaWeights = Beta->Clone();
			DeltaWeights->ProductScalar(MinVariation);
			this->Bias += DeltaWeights->Values[0];			
			
			// Update Weights Support Set
			int i;
			for (i=0; i<this->GetSupportSetElementsNumber(); i++) {
				this->Weights->Values[this->SupportSetIndexes->Values[i]] += DeltaWeights->Values[i+1];
			}
			delete DeltaWeights;
			
			// Update H
			Vector<double>* DeltaH = Gamma->Clone();
			DeltaH->ProductScalar(MinVariation);
 			for (i=0; i<this->GetSamplesTrainedNumber(); i++) {
				(*H)->Values[i] += DeltaH->Values[i];
			}
			delete DeltaH;
		}
		else {
			
			// Update Bias
			this->Bias += MinVariation;
			
			// Update H
			(*H)->SumScalar(MinVariation);
		}
	}

	void OnlineSVR::AddSampleToRemainingSet (int SampleIndex)
	{
		this->ShowMessage("> Case 0 : the sample has been classified correctly",2);
		this->RemainingSetIndexes->Add(SampleIndex);  
	}

		
	// Set Operations
	void OnlineSVR::AddSampleToSupportSet (Vector<double>** H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double MinVariation)
	{
		// Message
		char Line[100];	
		sprintf(Line, "> Case 1 : sample %d is a support sample", SampleIndex);	
		for (int i=strlen(Line); i<62; i++)		
			Line[i] = ' ';
		Line[62] = 0;
		sprintf(Line, "%s(Var= %f)", Line, MinVariation);
		this->ShowMessage(Line,2);
		// Update H and Sets
		(*H)->Values[SampleIndex] = SIGN((*H)->Values[SampleIndex]) * this->Epsilon;
		this->SupportSetIndexes->Add (SampleIndex);
		this->AddSampleToR (SampleIndex, this->SUPPORT_SET, Beta, Gamma);
	}

	void OnlineSVR::AddSampleToErrorSet (int SampleIndex, double MinVariation)
	{
		// Message
		char Line[100];
		sprintf(Line, "> Case 2 : sample %d is an error sample", SampleIndex);
		for (int i=strlen(Line); i<62; i++)		
			Line[i] = ' ';
		Line[62] = 0;
		sprintf(Line, "%s(Var= %f)", Line, MinVariation);
		this->ShowMessage(Line,2);
		// Update H and Sets
		this->Weights->Values[SampleIndex] = SIGN(this->Weights->Values[SampleIndex]) * this->C;
		this->ErrorSetIndexes->Add(SampleIndex);
	}

	void OnlineSVR::MoveSampleFromSupportSetToErrorRemainingSet (int MinIndex, double MinVariation)
	{
		int Index = this->SupportSetIndexes->Values[MinIndex];
		double Weightsi = this->Weights->Values[Index];

		if (ABS(Weightsi)<ABS(this->C-ABS(Weightsi))) {
			this->Weights->Values[Index] = 0;
		}
		else {
			this->Weights->Values[Index] = SIGN(Weightsi) * this->C;
		}

		if (this->Weights->Values[Index] == 0) {
        
			// CASE 3a: Move Sample from SupportSet to RemainingSet                   	
			// Message
			char Line[100];
			sprintf(Line, "> Case 3a: move sample %d from support to remaining set", Index);
			for (int i=strlen(Line); i<62; i++)		
				Line[i] = ' ';
			Line[62] = 0;
			sprintf(Line, "%s(Var= %f)", Line, MinVariation);
			this->ShowMessage(Line,2);
			// Update H and Sets
			this->RemainingSetIndexes->Add(Index);
			this->SupportSetIndexes->RemoveAt(MinIndex);
			this->RemoveSampleFromR(MinIndex);
		}	
		else {
			// CASE 3b: Move Sample from SupportSet to ErrorSet            
			// Message
			char Line[100];
			sprintf(Line, "> Case 3b: move sample %d from support to error set", Index);
			for (int i=strlen(Line); i<62; i++)		
				Line[i] = ' ';
			Line[62] = 0;
			sprintf(Line, "%s(Var= %f)", Line, MinVariation);
			this->ShowMessage(Line,2);
			// Update H and Sets
			this->ErrorSetIndexes->Add(Index);
			this->SupportSetIndexes->RemoveAt(MinIndex);
			this->RemoveSampleFromR(MinIndex);
		}
	}

	void OnlineSVR::MoveSampleFromErrorSetToSupportSet (Vector<double>** H, Vector<double>* Beta, Vector<double>* Gamma, int MinIndex, double MinVariation)
	{
		int Index = this->ErrorSetIndexes->Values[MinIndex];
		// Message
		char Line[100];
		sprintf(Line, "> Case 4 : move sample %d from error to support set", Index);
		for (int i=strlen(Line); i<62; i++)		
			Line[i] = ' ';
		Line[62] = 0;
		sprintf(Line, "%s(Var= %f)", Line, MinVariation);
		this->ShowMessage(Line,2);
		// Update H and Sets	
		(*H)->Values[Index] = SIGN((*H)->Values[Index]) * this->Epsilon;
		this->SupportSetIndexes->Add(Index);
		this->ErrorSetIndexes->RemoveAt(MinIndex);
		this->AddSampleToR (Index, this->ERROR_SET, Beta, Gamma);
	}

	void OnlineSVR::MoveSampleFromRemainingSetToSupportSet (Vector<double>** H, Vector<double>* Beta, Vector<double>* Gamma, int MinIndex, double MinVariation)
	{
		int Index = this->RemainingSetIndexes->Values[MinIndex];
		// Message
		char Line[100];
		sprintf(Line, "> Case 5 : move sample %d from remaining to support set", Index);
		for (int i=strlen(Line); i<62; i++)		
			Line[i] = ' ';
		Line[62] = 0;
		sprintf(Line, "%s(Var= %f)", Line, MinVariation);
		this->ShowMessage(Line,2);
		// Update H and Sets	
		(*H)->Values[Index] = SIGN((*H)->Values[Index]) * this->Epsilon;
		this->SupportSetIndexes->Add(Index);
		this->RemainingSetIndexes->RemoveAt(MinIndex);
		this->AddSampleToR (Index, this->REMAINING_SET, Beta, Gamma);
	}

	void OnlineSVR::RemoveSampleFromSupportSet (int SampleSetIndex)
	{
		this->SupportSetIndexes->RemoveAt(SampleSetIndex);
		this->RemoveSampleFromR(SampleSetIndex);
	}

	void OnlineSVR::RemoveSampleFromErrorSet (int SampleSetIndex)
	{
		this->ErrorSetIndexes->RemoveAt(SampleSetIndex);
	}

	void OnlineSVR::RemoveSampleFromRemainingSet (int SampleSetIndex)
	{
		int SampleIndex = this->RemainingSetIndexes->Values[SampleSetIndex];
		this->ShowMessage("> Case 0 : the sample was removed from the remaining set",2);
		this->RemainingSetIndexes->RemoveAt(SampleSetIndex);
		this->X->RemoveRow(SampleIndex);
		this->Y->RemoveAt(SampleIndex);
		this->Weights->RemoveAt(SampleIndex);
		if (this->SaveKernelMatrix) {
			this->RemoveSampleFromKernelMatrix(SampleIndex);
		}
		int i;
		for (i=0; i<this->GetSupportSetElementsNumber(); i++) {
			if (this->SupportSetIndexes->Values[i]>SampleIndex) {
				this->SupportSetIndexes->Values[i] --;
			}
		}
		for (i=0; i<this->GetErrorSetElementsNumber(); i++) {
			if (this->ErrorSetIndexes->Values[i]>SampleIndex) {
				this->ErrorSetIndexes->Values[i] --;
			}
		}
		for (i=0; i<this->GetRemainingSetElementsNumber(); i++) {
			if (this->RemainingSetIndexes->Values[i]>SampleIndex) {
				this->RemainingSetIndexes->Values[i] --;
			}
		}
		this->SamplesTrainedNumber --;
		if (this->SamplesTrainedNumber==0) {
			this->Bias = 0;
		}
	}

	void OnlineSVR::RemoveSample (int SampleIndex)
	{
		this->ShowMessage("> Case 1 : the sample Weights becomes 0",2);	
		this->X->RemoveRow(SampleIndex);
		this->Y->RemoveAt(SampleIndex);
		this->Weights->RemoveAt(SampleIndex);
		if (this->SaveKernelMatrix) {
			this->RemoveSampleFromKernelMatrix(SampleIndex);
		}

		int i;
		for (i=0; i<this->GetSupportSetElementsNumber(); i++) {
			if (this->SupportSetIndexes->Values[i]>SampleIndex) {
				this->SupportSetIndexes->Values[i] --;
			}
		}
		for (i=0; i<this->GetErrorSetElementsNumber(); i++) {
			if (this->ErrorSetIndexes->Values[i]>SampleIndex) {
				this->ErrorSetIndexes->Values[i] --;
			}
		}
		for (i=0; i<this->GetRemainingSetElementsNumber(); i++) {
			if (this->RemainingSetIndexes->Values[i]>SampleIndex) {
				this->RemainingSetIndexes->Values[i] --;
			}
		}	

		this->SamplesTrainedNumber --;
		if (this->SamplesTrainedNumber==1 && this->GetErrorSetElementsNumber()>0) {
			this->ErrorSetIndexes->RemoveAt(0);
			this->RemainingSetIndexes->Add(0);
			this->Weights->Values[0] = 0;
			this->Bias = this->Margin(this->X->Values->Values[0],this->Y->Values[0]);
		}
		if (this->SamplesTrainedNumber==0) {
			this->Bias = 0;
		}
	}

}
	
#endif
