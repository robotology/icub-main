#ifndef KERNEL_CPP
#define KERNEL_CPP

#include "OnlineSVR.h"


namespace onlinesvr
{

	// Learning Operations
	void OnlineSVR::SelfLearning (Matrix<double>* TrainingSetX, Vector<double>* TrainingSetY, Matrix<double>* ValidationSetX, Vector<double>* ValidationSetY, double ErrorTollerance) 
	{
		// Parameters
		int MaxC = 500;
		int MaxKernelParam = 500;
		int Step = 5;

		// OnlineSVR Initialization	
		this->SetEpsilon(ErrorTollerance);
		this->SetKernelType(this->KERNEL_RBF);
		this->SetC(100);
		this->SetKernelParam(100);
		this->SetVerbosity(0);
		this->SetStabilizedLearning(true);
		this->SetSaveKernelMatrix(true);


		// Error Matrix Initialization
		int RowsNumber = MaxC + 2;
		int ColsNumber = MaxKernelParam + 2;    
		// Mean Errors
		Matrix<double>* MeanErrors = MeanErrors->ZeroMatrix(RowsNumber,ColsNumber);	
		MeanErrors->GetRowRef(0)->SumScalar(INF);
		MeanErrors->GetRowRef(RowsNumber-1)->SumScalar(INF);
		int i;
		for (i=0; i<RowsNumber; i++) {
			MeanErrors->SetValue(i, 0, INF);
			MeanErrors->SetValue(i, ColsNumber-1, INF);
		}
		// Max Errors
		Matrix<double>* MaxErrors = MaxErrors->ZeroMatrix(RowsNumber,ColsNumber);
		MaxErrors->GetRowRef(0)->SumScalar(INF);
		MaxErrors->GetRowRef(RowsNumber-1)->SumScalar(INF);
		for (i=0; i<RowsNumber; i++) {
			MaxErrors->SetValue(i, 0, INF);
			MaxErrors->SetValue(i, ColsNumber-1, INF);
		}

		// Start the process
		int IndexC = static_cast<int>(this->GetC());
		int IndexKernelParam = static_cast<int>(this->GetKernelParam());
		cout << "OnlineSVR Self-Learning Process" << endl;

		// Phase 1 - Learning
		int IterationNumber = 1;
		double MinError, MeanError, MaxError;
		cout << IterationNumber << ") Trying with C=" << this->GetC() << " and KernelParam=" << this->GetKernelParam() << endl;	
		this->Train(TrainingSetX,TrainingSetY);
		FindError (ValidationSetX, ValidationSetY, &MinError, &MeanError, &MaxError);
		MeanErrors->SetValue(IndexC+1, IndexKernelParam+1, MeanError);
		MaxErrors->SetValue(IndexC+1, IndexKernelParam+1, MaxError);

		// Phase 2 - Calibration of the parameters
		while (MaxErrors->GetValue(IndexC+1,IndexKernelParam+1)>ErrorTollerance) {

			// Check the nearest positions

			// Left
			if (!MeanErrors->GetValue(IndexC+1,IndexKernelParam+1-Step)) {
				this->SetC(IndexC);
				this->SetKernelParam(IndexKernelParam-Step);
				IterationNumber ++;
				cout << IterationNumber << ") Trying with C=" << this->GetC() << " and KernelParam=" << this->GetKernelParam() << endl;	
				this->Stabilize();
				this->FindError (ValidationSetX, ValidationSetY, &MinError, &MeanError, &MaxError);
				MeanErrors->SetValue(IndexC+1, IndexKernelParam+1-Step, MeanError);
				MaxErrors->SetValue(IndexC+1, IndexKernelParam+1-Step, MaxError);
			}

			// Rigth
			if (!MeanErrors->GetValue(IndexC+1,IndexKernelParam+1+Step)) {
				this->SetC(IndexC);
				this->SetKernelParam(IndexKernelParam+Step);
				IterationNumber ++;
				cout << IterationNumber << ") Trying with C=" << this->GetC() << " and KernelParam=" << this->GetKernelParam() << endl;	
				this->Stabilize();
				FindError (ValidationSetX, ValidationSetY, &MinError, &MeanError, &MaxError);
				MeanErrors->SetValue(IndexC+1, IndexKernelParam+1+Step, MeanError);
				MaxErrors->SetValue(IndexC+1, IndexKernelParam+1+Step, MaxError);
			}

			// Up
			if (!MeanErrors->GetValue(IndexC+1-Step, IndexKernelParam+1)) {
				this->SetC(IndexC-Step);
				this->SetKernelParam(IndexKernelParam);
				IterationNumber ++;
				cout << IterationNumber << ") Trying with C=" << this->GetC() << " and KernelParam=" << this->GetKernelParam() << endl;	
				this->Stabilize();
				FindError (ValidationSetX, ValidationSetY, &MinError, &MeanError, &MaxError);
				MeanErrors->SetValue(IndexC+1-Step, IndexKernelParam+1, MeanError);
				MaxErrors->SetValue(IndexC+1-Step, IndexKernelParam+1, MaxError);
			}

			// Down
			if (!MeanErrors->GetValue(IndexC+1+Step, IndexKernelParam+1)) {
				this->SetC(IndexC+Step);
				this->SetKernelParam(IndexKernelParam);
				IterationNumber ++;
				cout << IterationNumber << ") Trying with C=" << this->GetC() << " and KernelParam=" << this->GetKernelParam() << endl;	
				this->Stabilize();
				FindError (ValidationSetX, ValidationSetY, &MinError, &MeanError, &MaxError);
				MeanErrors->SetValue(IndexC+1+Step, IndexKernelParam+1, MeanError);
				MaxErrors->SetValue(IndexC+1+Step, IndexKernelParam+1, MaxError);
			}

			// Find the next Move
			double MinValue;
			int MinIndex;
			Vector<double>* Errors = new Vector<double>(5);
			Errors->Add(MeanErrors->GetValue(IndexC+1, IndexKernelParam+1));
			Errors->Add(MeanErrors->GetValue(IndexC+1, IndexKernelParam+1+Step));
			Errors->Add(MeanErrors->GetValue(IndexC+1, IndexKernelParam+1-Step));
			Errors->Add(MeanErrors->GetValue(IndexC+1+Step, IndexKernelParam+1));
			Errors->Add(MeanErrors->GetValue(IndexC+1-Step, IndexKernelParam+1));
			Errors->Min(&MinValue,&MinIndex);
			delete Errors;
			switch (MinIndex) 
			{
			case 0:
				// The current solution is the best			
				if (Step==5) 
					Step = 1;
				else
					Step = 0;
				break;
			case 1:
				// The rigth solution is better
				IndexC += Step;
				break;
			case 2:
				// The left solution is better
				IndexC -= Step;
				break;
			case 3:
				// The down solution is better
				IndexKernelParam += Step;
				break;
			case 4:
				// The up solution is better
				IndexKernelParam -= Step;
				break;
			}

			// If there aren't more steps, finish
			if (!Step) {
				break;
			}
		}

		// Finish
		cout << "Optimizing the solution..." << endl << endl;
		this->Stabilize();
		if (MaxErrors->GetValue(IndexC+1, IndexKernelParam+1)<=ErrorTollerance)
			cout << "The OnlineSVR has found an acceptable solution at the problem." << endl;
		else
			cout << "The OnlineSVR hasn't found an acceptable solution at the problem." << endl;

		// Clear
		delete MeanErrors;
		delete MaxErrors;

	}

	void OnlineSVR::FindError(Matrix<double>* ValidationSetX, Vector<double>* ValidationSetY, double* MinError, double* MeanError, double* MaxError)
	{
		Vector<double>* Errors = this->Margin(ValidationSetX, ValidationSetY);
		(*MinError) = Errors->MinAbs();
		(*MeanError) = Errors->MeanAbs();
		(*MaxError) = Errors->MaxAbs();
		delete Errors;
		cout << "  > Min  Error: " << *MinError << endl;
		cout << "  > Mean Error: " << *MeanError << endl;
		cout << "  > Max  Error: " << *MaxError << endl;	
	}

}
	
#endif
