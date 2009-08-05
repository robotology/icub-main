#ifndef CROSS_VALIDATION_CPP
#define CROSS_VALIDATION_CPP

#include "OnlineSVR.h"
#include <iostream>
#include <fstream>
#include <time.h>


namespace onlinesvr
{

	void OnlineSVR::CrossValidation (Matrix<double>* TrainingSetX, Vector<double>* TrainingSetY, Vector<double>* EpsilonList, Vector<double>* CList, Vector<double>* KernelParamList, int SetNumber, char* ResultsFileName)
	{
		int i, j, k;

		// Partition of the training set
		int SamplesPerSet = static_cast<int>(TrainingSetX->GetLengthRows()/SetNumber);
		
		// Build the Sets
		Vector<Matrix<double>*>* SetX = new Vector<Matrix<double>*>();
		Vector<Vector<double>*>* SetY = new Vector<Vector<double>*>();		
		for (i=0; i<SetNumber; i++) {
			SetX->Add(new Matrix<double>());
			SetY->Add(new Vector<double>());
			for (j=0; j<SamplesPerSet; j++) {
				SetX->GetValue(i)->AddRowCopy(TrainingSetX->GetRowRef(j*SetNumber+i));
				SetY->GetValue(i)->Add(TrainingSetY->GetValue(j*SetNumber+i));
			}
		}
		
		// Open the file
		ofstream File (ResultsFileName, ios::out);
		if (!File) {
			cerr << "Error. It's impossible to create the file." << endl;
			return;
		}		
		File << "Epsilon \t C \t KernelParam \t Error" << endl;

		// Cross Validation
		int IterationsNumber = EpsilonList->GetLength() * CList->GetLength() * KernelParamList->GetLength();
		int CurrentIteration = 0;
		double MinError = 10000;
		double MinEpsilon = 0;
		double MinC = 0;
		double MinKernelParam = 0;
		for (i=0; i<EpsilonList->GetLength(); i++) {
			for (j=0; j<CList->GetLength(); j++) {		
				for (k=0; k<KernelParamList->GetLength(); k++) {
					if(1==0)//if (CurrentIteration==33 || CurrentIteration==34 || CurrentIteration==42)
					{CurrentIteration++; printf("%d\n",CurrentIteration);}
					else {
					double Error = CrossValidation (SetX, SetY, EpsilonList->GetValue(i), CList->GetValue(j), KernelParamList->GetValue(k));
					File << EpsilonList->GetValue(i) << "\t" << CList->GetValue(j) << "\t" << KernelParamList->GetValue(k) << " \t" << Error << endl;
					cout << "Test " << ++CurrentIteration << "/" << IterationsNumber << "\t" << "Epsilon=" << EpsilonList->GetValue(i) << "\t" << "C=" << CList->GetValue(j) << "\t" << "KernelParam=" << KernelParamList->GetValue(k) << " \t" << "Error=" << Error   << endl;
					if (Error<MinError) {
						MinError = Error;
						MinEpsilon = EpsilonList->GetValue(i);
						MinC = CList->GetValue(j);
						MinKernelParam = KernelParamList->GetValue(k);
					}
					}
				}
			}
		}

		// Best Result		
		File << endl << "Best Solution:" << endl;
		File << MinEpsilon << "\t" << MinC << "\t" << MinKernelParam << " \t" << MinError << endl;
		cout << endl << "Best Solution:" << endl;
		cout << MinEpsilon << "\t" << MinC << "\t" << MinKernelParam << " \t" << MinError << endl;

		// Close the file
		File.close();
		
		// Free the memory
		for (i=0; i<SetNumber; i++) {
			delete SetX->GetValue(i);
			delete SetY->GetValue(i);
		}
		delete SetX;
		delete SetY;
	}

	double OnlineSVR::CrossValidation (Vector<Matrix<double>*>* SetX, Vector<Vector<double>*>* SetY, double Epsilon, double C, double KernelParam)
	{
		// Variables
		Vector<double>* Errors = new Vector<double>();
		int i, j;

		// Subdivide TrainingSet into SubSets
		for (i=0; i<SetX->GetLength(); i++)
		{
			// New SVR
			OnlineSVR* SVR = new OnlineSVR();
			SVR->SetEpsilon(Epsilon);
			SVR->SetC(C);
			SVR->SetKernelType(OnlineSVR::KERNEL_RBF);
			SVR->SetKernelParam(KernelParam);
			SVR->SetVerbosity(OnlineSVR::VERBOSITY_NO_MESSAGES);
			// Train with N-1 SubSet
			for (j=0; j<SetX->GetLength(); j++) {
				if (i != j) {					
					SVR->Train(SetX->GetValue(j), SetY->GetValue(j));
				}
			}
			// Test with the other SubSet
			Vector<double>* ErrorList = SVR->Margin(SetX->GetValue(i),SetY->GetValue(i));			
			double MeanError = ErrorList->AbsSum() / static_cast<double>(SetX->GetValue(i)->GetLengthRows());
			delete ErrorList;
			// Free
			Errors->Add(MeanError);
			delete SVR;
		}
		
		// Free the memory
		double MeanError = Errors->AbsSum() / Errors->GetLength();		
		delete Errors;
		return MeanError;
	}

	void OnlineSVR::LeaveOneOut (Matrix<double>* TrainingSetX, Vector<double>* TrainingSetY, Vector<double>* EpsilonList, Vector<double>* CList, Vector<double>* KernelParamList, char* ResultsFileName)
	{
		int i, j, k;
		
		// Open the file
		ofstream File (ResultsFileName, ios::out);
		if (!File) {
			cerr << "Error. It's impossible to create the file." << endl;
			return;
		}
		File << "Epsilon \t C \t KernelParam \t Error" << endl;

		// Leave one out
		int IterationsNumber = EpsilonList->GetLength() * CList->GetLength() * KernelParamList->GetLength();
		int CurrentIteration = 0;
		double MinError = 10000;
		double MinEpsilon = 0;
		double MinC = 0;
		double MinKernelParam = 0;
		for (i=0; i<EpsilonList->GetLength(); i++) {
			for (j=0; j<CList->GetLength(); j++) {		
				for (k=0; k<KernelParamList->GetLength(); k++) {
					double Error = LeaveOneOut (TrainingSetX, TrainingSetY, EpsilonList->GetValue(i), CList->GetValue(j), KernelParamList->GetValue(k));
					File << EpsilonList->GetValue(i) << "\t" << CList->GetValue(j) << "\t" << KernelParamList->GetValue(k) << " \t" << Error << endl;
					cout << "Test " << ++CurrentIteration << "/" << IterationsNumber << "\t" << "Epsilon=" << EpsilonList->GetValue(i) << "\t" << "C=" << CList->GetValue(j) << "\t" << "KernelParam=" << KernelParamList->GetValue(k) << " \t" << "Error=" << Error   << endl;
					if (Error<MinError) {
						MinError = Error;
						MinEpsilon = EpsilonList->GetValue(i);
						MinC = CList->GetValue(j);
						MinKernelParam = KernelParamList->GetValue(k);
					}
				}
			}
		}

		// Best Result		
		File << endl << "Best Solution:" << endl;
		File << MinEpsilon << "\t" << MinC << "\t" << MinKernelParam << " \t" << MinError << endl;
		cout << endl << "Best Solution:" << endl;
		cout << MinEpsilon << "\t" << MinC << "\t" << MinKernelParam << " \t" << MinError << endl;

		// Close the file
		File.close();		
	}


	double OnlineSVR::LeaveOneOut (Matrix<double>* SetX, Vector<double>* SetY, double Epsilon, double C, double KernelParam)
	{
		// Variables
		Vector<double>* Errors = new Vector<double>();
		int i;

		// Train the SVR
		OnlineSVR* SVR = new OnlineSVR();
		SVR->SetEpsilon(Epsilon);
		SVR->SetC(C);
		SVR->SetKernelType(OnlineSVR::KERNEL_RBF);
		SVR->SetKernelParam(KernelParam);
		SVR->SetVerbosity(OnlineSVR::VERBOSITY_NO_MESSAGES);
		SVR->Train(SetX, SetY);

		// Forget a sample and test the errors
		for (i=0; i<SetX->GetLengthRows(); i++)
		{
			OnlineSVR* SVR2 = SVR->Clone();
			SVR2->Forget(i);
			double Error = SVR2->Margin(SetX->GetRowRef(i), SetY->GetValue(i));
			Errors->Add(Error);
			delete SVR2;
		}
		
		// Free the memory
		double MeanError = Errors->AbsSum() / Errors->GetLength();		
		delete SVR;
		delete Errors;
		return MeanError;
	}
}

#endif
