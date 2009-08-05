#ifndef TRAIN_CPP
#define TRAIN_CPP

#include <iostream>
#include "time.h"
#include "OnlineSVR.h"


namespace onlinesvr
{

	// Learning Operations
	int OnlineSVR::Train (Matrix<double>* X, Vector<double>* Y)
	{
		// Initialization
		time_t StartTime = time(NULL);
		int Flops = 0;
		this->ShowMessage("Starting Training...\n",1);
		
		// Learning
		for (int i=0; i<X->GetLengthRows(); i++) {
			// Show Informations
			this->ShowMessage(" ",2);
			this->ShowMessage(" ",3);
			char Line[80];
			sprintf(Line,"Training %d/%d",i+1,X->GetLengthRows());			
			this->ShowMessage(Line,1);
			// Training
			Flops += this->Learn(X->GetRowRef(i),Y->GetValue(i));
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
		sprintf(Line, "\nTrained %d elements correctly in %s.\n", X->GetLengthRows(), this->TimeToString(LearningTime));	
		this->ShowMessage(Line,1);

		return Flops;
	}

	int OnlineSVR::Train (double**X, double *Y, int ElementsNumber, int ElementsSize)
	{	
		Matrix<double>* NewX = new Matrix<double>(X, ElementsNumber, ElementsSize);
		Vector<double>* NewY = new Vector<double>(Y, ElementsNumber);
		int Flops = Train(NewX,NewY);
		delete NewX;
		delete NewY;
		return Flops;
	}

	int OnlineSVR::Train (Vector<double>* X, double Y)
	{
		int Flops;
		Matrix<double>* X1 = new Matrix<double>();
		Vector<double>* Y1 = new Vector<double>();
		X1->AddRowCopy(X);
		Y1->Add(Y);
		Flops = this->Train(X1,Y1);
		delete X1;
		delete Y1;
		return Flops;
	}

	int OnlineSVR::Learn (Vector<double>* X, double Y)
	{
		// Inizializations
		this->X->AddRowCopy(X);
		this->Y->Add(Y);
		this->Weights->Add(0);
		this->SamplesTrainedNumber ++;
		if (this->SaveKernelMatrix) {
			this->AddSampleToKernelMatrix(X);
		}	
		int Flops = 0;
		double Epsilon = this->Epsilon;
		bool NewSampleAdded = false;
		int SampleIndex = this->SamplesTrainedNumber-1;

		// CASE 0: Right classified sample
		if (ABS(this->Margin(X,Y))<=Epsilon) {
			this->AddSampleToRemainingSet(SampleIndex);
			NewSampleAdded = true;
			Flops ++;
			return Flops;
		}

		// Find the Margin
		Vector<double>* H = this->Margin(this->X,this->Y);	

		// Main Loop
		while (!NewSampleAdded) {

			// Check Iterations Number
			Flops ++;
			if (Flops > (this->GetSamplesTrainedNumber()+1)*100) {
				cerr << endl << "Learning Error. Infinite Loop." << endl;
				system("pause");
				exit(1);
			}
			
			// KKT CONDITION CHECKING - TODO
			//if (!this->VerifyKKTConditions(H)) {
			//	this->ShowDetails(H,SampleIndex);
			//	int x = 0;
			//}

			// Find Beta and Gamma
			Vector<double>* Beta = this->FindBeta(SampleIndex);
			Vector<double>* Gamma = this->FindGamma(Beta,SampleIndex);
					
			// Find Min Variation
			double MinVariation = 0;
			int Flag = -1;
			int MinIndex = -1;		
			FindLearningMinVariation (H, Beta, Gamma, SampleIndex, &MinVariation, &MinIndex, &Flag);

			// Update Weights and Bias		
			this->UpdateWeightsAndBias (&H, Beta, Gamma, SampleIndex, MinVariation);

			// Move the Sample with Min Variaton to the New Set
			switch (Flag) {
				
				// CASE 1: Add the sample to the support set
				case 1:
					this->AddSampleToSupportSet (&H, Beta, Gamma, SampleIndex, MinVariation);
					NewSampleAdded = true;
					break;			
				
				// CASE 2: Add the sample to the error set
				case 2:
					this->AddSampleToErrorSet (SampleIndex, MinVariation);
					NewSampleAdded = true;
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
