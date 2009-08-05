#ifndef FORGET_CPP
#define FORGET_CPP

#include <iostream>
#include "time.h"
#include "OnlineSVR.h"


namespace onlinesvr
{

	// Learning Operations
	int OnlineSVR::Forget (Vector<int>* Indexes)
	{
		// Initialization
		time_t StartTime = time(NULL);
		int Flops = 0;
		this->ShowMessage("Starting Forget...\n",1);
		
		// Check the indexes
		Indexes->Sort();
		Indexes->RemoveDuplicates();
		if (!(Indexes->GetValue(0)>=0 && Indexes->GetValue(Indexes->GetLength()-1)<SamplesTrainedNumber)) {
			cerr << "Error. The indexes of the samples to remove are not valid." << endl;
			return -1;
		}

		// Unlearning
		for (int i=Indexes->GetLength()-1; i>=0; i--) {
			// Show Informations
			this->ShowMessage(" ",2);
			this->ShowMessage(" ",3);
			char Line[80];
			sprintf(Line,"Forgetting %d/%d",Indexes->GetLength()-i,Indexes->GetLength());
			this->ShowMessage(Line,1);
			// Forgetting
			Flops += this->Unlearn(Indexes->GetValue(i));
		}
		
		// Stabilize the results
		if (this->StabilizedLearning) {	
			int StabilizationNumber = 0;
			while (!this->VerifyKKTConditions()) {
				Flops += this->Stabilize();
				StabilizationNumber ++;
				if (StabilizationNumber>this->GetSamplesTrainedNumber()) {
					cerr << "Error: it's impossible to stabilize the OnlineSVR. Please add or remove some samples." << endl;		
					break;
				}
			}
		}

		if (this->Verbosity>=3)
			this->ShowDetails();

		// Show Execution Time
		time_t EndTime = time(NULL);
		long LearningTime = static_cast<long>(EndTime-StartTime);
		this->ShowMessage(" ",2);
		this->ShowMessage(" ",3);
		char Line[80];
		sprintf(Line, "\nForgetted %d elements correctly in %s.\n", Indexes->GetLength(), this->TimeToString(LearningTime));	
		this->ShowMessage(Line,1);

		return Flops;
	}

	int OnlineSVR::Forget (int* Indexes, int ElementsNumber)
	{	
		Vector<int>* NewIndexes = new Vector<int>(Indexes,ElementsNumber);
		int Flops = Forget(NewIndexes);
		delete NewIndexes;
		return Flops;
	}

	int OnlineSVR::Forget (int Index)
	{
		int Flops;
		Vector<int>* Indexes = new Vector<int>();		
		Indexes->Add(Index);		
		Flops = this->Forget(Indexes);
		delete Indexes;
		return Flops;
	}
		
	int OnlineSVR::Unlearn (int SampleIndex)
	{
		// Inizializations			
		int Flops = 0;
		bool SampleRemoved = false;


		// CASE 0: RemainingSet Sample
		int SampleSetIndex = this->RemainingSetIndexes->Find(SampleIndex);
		if (SampleSetIndex>=0) {
			this->RemoveSampleFromRemainingSet(SampleSetIndex);
			SampleRemoved = true;
			Flops ++;
			return Flops;
		}
		else {
			SampleSetIndex = this->SupportSetIndexes->Find(SampleIndex);
			if (SampleSetIndex>=0)
				this->RemoveSampleFromSupportSet(SampleSetIndex);
			else {
				SampleSetIndex = this->ErrorSetIndexes->Find(SampleIndex);
				this->RemoveSampleFromErrorSet(SampleSetIndex);
			}
		}


		// Find the margin
		Vector<double>* H = this->Margin(this->X,this->Y);

		// Main Loop
		while (!SampleRemoved) {
					
			// Check Iterations Number
			Flops ++;
			if (Flops > (this->GetSamplesTrainedNumber()+1)*100) {
				this->ShowDetails(H,SampleIndex);
				cerr << endl << "Unlearning Error. Infinite Loop." << endl;
				system("pause");
				exit(1);
			}
			
			// KKT CONDITION CHECKING - TODO

			// Find Beta and Gamma
			Vector<double>* Beta = this->FindBeta(SampleIndex);
			Vector<double>* Gamma = this->FindGamma(Beta,SampleIndex);
					
			// Find Min Variation
			double MinVariation = 0;
			int Flag = -1;
			int MinIndex = -1;		
			FindUnlearningMinVariation (H, Beta, Gamma, SampleIndex, &MinVariation, &MinIndex, &Flag);		

			// Update Weights and Bias		
			this->UpdateWeightsAndBias (&H, Beta, Gamma, SampleIndex, MinVariation);

			// Move the Sample with Min Variaton to the New Set
			switch (Flag) {
				
				// CASE 1: Remove the sample
				case 1:
					this->RemoveSample(SampleIndex);
					SampleRemoved = true;
					break;			
				
				// CASE 2: Not used
				case 2:
					break;

				// CASE 3: Move Sample from SupportSet to ErrorSet/RemainingSet
				case 3:
					this->MoveSampleFromSupportSetToErrorRemainingSet (MinIndex, MinVariation);
					break;

				// CASE 4: Move Sample from ErrorSet to SupportSet
				case 4:				
					this->MoveSampleFromErrorSetToSupportSet (&H, Beta, Gamma, MinIndex, MinVariation);								
					break;

				// CASE 5: Move Sample from RemainingSet to SupportSet
				case 5:
					this->MoveSampleFromRemainingSetToSupportSet (&H, Beta, Gamma, MinIndex, MinVariation);
					break;
			}

			// Clear
			delete Beta;
			delete Gamma;
		}

		// Clear
		delete H;

		return Flops;
	}

}	
	
#endif
