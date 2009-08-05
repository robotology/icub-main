#ifndef FILE_CPP
#define FILE_CPP

#include "OnlineSVR.h"
#include <iostream>
#include <fstream>
#include <time.h>


namespace onlinesvr
{

	// I/O Operations
	void OnlineSVR::LoadOnlineSVR(char* Filename)
	{	
		// Open the file
		ifstream File (Filename, ios::in);
		if (!File) {
			cerr << "Error. File not found." << endl;
			return;
		}

		// Clear the old OnlineSVR
		this->Clear();

		try {

			// Title
			char X1[80];	
			double X;
			int X2;
			int SamplesDimension;
			bool SaveKernelMatrix;
			File >> X1 >> X1 >> X1 >> X1 >> X1 >> X1;
			
			// Reading the parameters
			File >> X1 >> X1 >> X1 >> X1 >> X1;
			File >> X1 >> X1 >> this->SamplesTrainedNumber;	
			File >> X1 >> X1 >> SamplesDimension;
			File >> X1 >> X1 >> this->C;
			File >> X1 >> X1 >> this->Epsilon;
			File >> X1 >> X1 >> this->KernelType;
			File >> X1 >> X1 >> this->KernelParam;
			File >> X1 >> X1 >> this->KernelParam2;
			File >> X1 >> X1 >> this->Verbosity;
			File >> X1 >> X1 >> this->AutoErrorTollerance;
			File >> X1 >> X1 >> this->ErrorTollerance;
			File >> X1 >> X1 >> this->StabilizedLearning;
			File >> X1 >> X1 >> SaveKernelMatrix;
			
			// Reading Training Set
			int i;
			File >> X1 >> X1 >> X1 >> X1 >> X1 >> X1 >> X1;	
			for (i=0; i<this->SamplesTrainedNumber; i++) {
				Vector<double>* Sample = new Vector<double>(SamplesDimension);
				for (int j=0; j<SamplesDimension; j++) {
					File >> X;
					Sample->Add(X);
				}
				this->X->AddRowRef(Sample);
			}
			File >> X1 >> X1 >> X1 >> X1 >> X1 >> X1 >> X1;
			for (i=0; i<this->SamplesTrainedNumber; i++) {
				File >> X;
				this->Y->Add(X);
			}

			// Reading Set Indexes
			int SupportSetElementsNumber;
			int ErrorSetElementsNumber;
			int RemainingSetElementsNumber;
			File >> X1 >> X1 >> X1 >> X1 >> X1 >> X1;	
			File >> X1 >> X1 >> SupportSetElementsNumber;
			File >> X1 >> X1 >> ErrorSetElementsNumber;
			File >> X1 >> X1 >> RemainingSetElementsNumber;
			File >> X1 >> X1;
			for (i=0; i<SupportSetElementsNumber; i++) {
				File >> X2;
				this->SupportSetIndexes->Add(X2);
			}	
			File >> X1 >> X1;
			for (i=0; i<ErrorSetElementsNumber; i++) {
				File >> X2;
				this->ErrorSetIndexes->Add(X2);
			}	
			File >> X1 >> X1;
			for (i=0; i<RemainingSetElementsNumber; i++) {
				File >> X2;
				this->RemainingSetIndexes->Add(X2);
			}

			// Weights and Bias
			File >> X1 >> X1 >> X1 >> X1 >> X1 >> X1 >> X1 >> X1 >> X1;	
			for (i=0; i<this->SamplesTrainedNumber; i++) {
				File >> X;
				this->Weights->Add(X);
			}	
			File >> X1 >> X1 >> this->Bias;

			// R Matrix
			File >> X1 >> X1 >> X1 >> X1 >> X1 >> X1;	
			for (i=0; i<this->GetSupportSetElementsNumber()+1; i++) {
				Vector<double>* Sample = new Vector<double>();
				for (int j=0; j<this->GetSupportSetElementsNumber()+1; j++) {
					File >> X;
					Sample->Add(X);
				}
				this->R->AddRowRef(Sample);
			}

			// Kernel Matrix
			this->SaveKernelMatrix = SaveKernelMatrix;
			if (SaveKernelMatrix)
				this->BuildKernelMatrix();
		}
		catch (...) {
			cerr << "Error. The file is probably corrupted." << endl;
			this->Clear();
		}

		// Close the file
		File.close();
	}

	void OnlineSVR::SaveOnlineSVR(char* Filename)
	{
		// Open the file
		ofstream File (Filename, ios::out);
		if (!File) {
			cerr << "Error. It's impossible to create the file." << endl;
			return;
		}
		File.precision(30);
		//int Precision = 50;

		try {
			
			// Title
			File << "------------------" << endl;
			File << "-   ONLINE SVR   -" << endl;
			File << "------------------" << endl;
			File << endl;

			// Parameters
			File << "------------------" << endl;
			File << "-   PARAMETERS   -" << endl;
			File << "------------------" << endl;
			File << "SamplesTrainedNumber = " << this->SamplesTrainedNumber << endl;
			File << "SamplesDimension = " << this->X->GetLengthCols() << endl;
			File << "C = " << this->C << endl;
			File << "Epsilon = " << this->Epsilon << endl;
			File << "KernelType = " << this->KernelType << endl;
			File << "KernelParam = " << this->KernelParam << endl;
			File << "KernelParam2 = " << this->KernelParam2 << endl;
			File << "Verbosity = " << this->Verbosity << endl;
			File << "AutoErrorTollerance = " << this->AutoErrorTollerance << endl;
			File << "ErrorTollerance = " << this->ErrorTollerance << endl;
			File << "StabilizedLearning = " << this->StabilizedLearning << endl;
			File << "SaveKernelMatrix = " << this->SaveKernelMatrix << endl;
			File << endl;

			// Training Set
			File << "-----------------------" << endl;
			File << "-   TRAINING SET X    -" << endl;
			File << "-----------------------" << endl;
			int i;
			for (i=0; i<this->X->GetLengthRows(); i++) {
				for (int j=0; j<this->X->GetLengthCols(); j++) {
					File << this->X->GetValue(i,j) << " ";
				}
				File << endl;
			}
			File << endl;
			File << "-----------------------" << endl;
			File << "-   TRAINING SET Y    -" << endl;
			File << "-----------------------" << endl;
			for (i=0; i<this->Y->GetLength(); i++) {			
				File << this->Y->GetValue(i) << " ";
			}
			File << endl << endl;

			// Set Indexes
			File << "-------------------" << endl;
			File << "-   SET INDEXES   -" << endl;
			File << "-------------------" << endl;
			File << "SupportSetElementsNumber = " << this->GetSupportSetElementsNumber() << endl;
			File << "ErrorSetElementsNumber = " << this->GetErrorSetElementsNumber() << endl;
			File << "RemainingSetElementsNumber = " << this->GetRemainingSetElementsNumber() << endl;
			File << "SupportSet = ";		
			for (i=0; i<this->GetSupportSetElementsNumber(); i++) {			
				File << this->SupportSetIndexes->GetValue(i) << " ";
			}
			File << endl;
			File << "ErrorSet = ";		
			for (i=0; i<this->GetErrorSetElementsNumber(); i++) {			
				File << this->ErrorSetIndexes->GetValue(i) << " ";
			}
			File << endl;
			File << "RemainingSet = ";		
			for (i=0; i<this->GetRemainingSetElementsNumber(); i++) {			
				File << this->RemainingSetIndexes->GetValue(i) << " ";
			}
			File << endl << endl;

			// Weights & Bias
			File << "--------------------" << endl;
			File << "-   Weights & BIAS   -" << endl;
			File << "--------------------" << endl;
			File << "Weights = ";
			for (i=0; i<this->Weights->GetLength(); i++) {			
				File << this->Weights->GetValue(i) << " ";
			}
			File << endl;
			File << "Bias = " << this->Bias << endl;
			File << endl;

			// R Matrix
			File << "----------------" << endl;
			File << "-   R MATRIX   -" << endl;
			File << "----------------" << endl;
			for (i=0; i<this->R->GetLengthRows(); i++) {
				for (int j=0; j<this->R->GetLengthCols(); j++) {
					File << this->R->GetValue(i,j) << " ";
				}
				File << endl;
			}
			File << endl;

		}
		catch (...) {
			cerr << "Error. It's impossible to complete the save." << endl;
		}
		
		// Close the file
		File.close();
	}

	void OnlineSVR::Import(char* Filename, Matrix<double>** X, Vector<double>** Y)
	{	
		// Open the file
		ifstream File (Filename, ios::in);
		if (!File) {
			cerr << "Error. File not found." << endl;
			return;
		}

		// Time
		time_t StartTime = time(NULL);
		cout << "Starting import new data..." << endl;
		int RowsNumber, ColsNumber;

		try {

			// Reading the parameters		
			File >> RowsNumber >> ColsNumber;
			
			// Import the data
			(*X) = new Matrix<double>();
			(*Y) = new Vector<double>(RowsNumber);
			double Value;
			for (int i=0; i<RowsNumber; i++) {
				// Add Y
				File >> Value;
				(*Y)->Add(Value);
				// Add X
				Vector<double>* Line = new Vector<double>(ColsNumber-1);
				for (int j=0; j<ColsNumber-1; j++) {
					File >> Value;
					Line->Add(Value);
				}
				(*X)->AddRowRef(Line);
			}

		}
		catch (...) {
			cerr << "Error. The file is probably corrupted." << endl;
		}

		// Close the file
		File.close();

		// Final Message
		time_t EndTime = time(NULL);
		long ImportTime = static_cast<long>(EndTime-StartTime);
		char Line[80];
		sprintf(Line, "\nImported %d samples correctly in %s.\n", RowsNumber, TimeToString(ImportTime));	
		cout << Line << endl;
	}

	void OnlineSVR::Import(char* Filename, Matrix<double>** AngularPositions, Matrix<double>** MotorCurrents, Matrix<double>** AppliedVoltages)
	{	
		// Open the file
		ifstream File (Filename, ios::in);
		if (!File) {
			cerr << "Error. File not found." << endl;
			return;
		}

		// Time
		time_t StartTime = time(NULL);
		cout << "Starting import new data..." << endl;
		int RowsNumber = 0;

		try {

			(*AngularPositions) = new Matrix<double>();
			(*MotorCurrents) = new Matrix<double>();
			(*AppliedVoltages) = new Matrix<double>();
			Vector<double>* Line;
			char X[80];
			double X1, X2, X3, X4;

			File >> X >> X;
			while (!File.eof()) {
				RowsNumber ++;
				// Import Angular Position
				File >> X1 >> X >> X2 >> X >> X3 >> X >> X4 >> X >> X >> X;
				Line = new Vector<double>(4);
				Line->Add(X1); Line->Add(X2); Line->Add(X3); Line->Add(X4);
				(*AngularPositions)->AddRowRef(Line);
				// Import Motor Currents
				File >> X1 >> X >> X2 >> X >> X3 >> X >> X4 >> X >> X >> X;
				Line = new Vector<double>(4);
				Line->Add(X1); Line->Add(X2); Line->Add(X3); Line->Add(X4);
				(*MotorCurrents)->AddRowRef(Line);
				// Import Applied Voltages
				File >> X1 >> X >> X2 >> X >> X3 >> X >> X4 >> X >> X >> X;
				Line = new Vector<double>(4);
				Line->Add(X1); Line->Add(X2); Line->Add(X3); Line->Add(X4);
				(*AppliedVoltages)->AddRowRef(Line);
			}
		}
		catch (...) {
			cerr << "Error. The file is probably corrupted." << endl;
		}

		// Close the file
		File.close();

		// Final Message
		time_t EndTime = time(NULL);
		long ImportTime = static_cast<long>(EndTime-StartTime);
		char Line[80];
		sprintf(Line, "\nImported %d samples correctly in %s.\n", RowsNumber, TimeToString(ImportTime));	
		cout << Line << endl;
	}

}
		
#endif
