#ifndef SHOW_CPP
#define SHOW_CPP

#include <iostream>
#include <string>
#include <stdlib.h>
#include "OnlineSVR.h"


namespace onlinesvr
{

	// I/O Operations
	void OnlineSVR::ShowMessage (const char* Message, int VerbosityLevel)
	{
		this->ShowMessage(const_cast<char*>(Message),VerbosityLevel);
	}

	void OnlineSVR::ShowMessage (char* Message, int VerbosityLevel)
	{
		if (this->Verbosity>0 && this->Verbosity>=VerbosityLevel) {
			cout << Message << endl;
		}
	}

	void OnlineSVR::ShowInfo ()
	{
		// Line 1
		cout << endl;
		cout << "------------------------------------" << endl;	
		// Line 2
		cout << "          Online SVR Info" << endl;	
		// Line 3
		cout << "------------------------------------" << endl;
		// Info
		cout << " C:           " << this->C << endl;
		cout << " Epsilon:     " << this->Epsilon << endl;		
		switch (this->KernelType) {
			case OnlineSVR::KERNEL_LINEAR:
				cout << " KernelType:  " << "Linear"  << endl;
				break;
			case OnlineSVR::KERNEL_POLYNOMIAL:
				cout << " KernelType:  " << "Polynomial"  << endl;
				break;
			case OnlineSVR::KERNEL_RBF:
				cout << " KernelType:  " << "Radial Basis Function"  << endl;
				break;
			case OnlineSVR::KERNEL_RBF_GAUSSIAN:
				cout << " KernelType:  " << "Gaussian Radial Basis Function"  << endl;
				break;
			case OnlineSVR::KERNEL_RBF_EXPONENTIAL:
				cout << " KernelType:  " << "Exponential Radial Basis Function"  << endl;
			case OnlineSVR::KERNEL_MLP:
				cout << " KernelType:  " << "Multi Layer Perceptron"  << endl;
				break;
		}
		cout << " KernelParam: " << this->KernelParam  << endl << endl;	

		// Number of samples trained
		cout << " Number of Samples Trained: " << this->GetSamplesTrainedNumber() << endl;
		cout << "  >> Support Samples:   " << this->GetSupportSetElementsNumber() << endl;
		cout << "  >> Error Samples:     " << this->GetErrorSetElementsNumber() << endl;
		cout << "  >> Remaining Samples: " << this->GetRemainingSetElementsNumber() << endl;

		// Last Line
		cout << "------------------------------------" << endl << endl;
	}

	void OnlineSVR::ShowDetails ()
	{
		Vector<double>* H = this->Margin(this->X,this->Y);
		this->ShowDetails(H,-1);
		H->Clear();
	}

	void OnlineSVR::ShowDetails (Vector<double>* H, int SampleIndex)
	{
		// Line 1
		cout << endl << "-------------------------------------------------------------------------------" << endl;
		// Line 2
		cout << "                               " << "ONLINE SVR DETAILS" << "                              " << endl;
		// Line 3
		cout << "-------------------------------------------------------------------------------" << endl;	
		// Line 4
		cout << "  ELEMENT   Weights                 H" << endl;
		// Support Set Elements
		int i;
		for (i=0; i<this->GetSupportSetElementsNumber(); i++) {
			int Index = this->SupportSetIndexes->GetValue(i);
			this->ShowLine(const_cast<char*>("S"), i, Index, this->Weights->GetValue(Index), H->GetValue(Index));
		}
		// Error Set Elements
		for (i=0; i<this->GetErrorSetElementsNumber(); i++) {
			int Index = this->ErrorSetIndexes->GetValue(i);
			this->ShowLine(const_cast<char*>("E"), i, Index, this->Weights->GetValue(Index), H->GetValue(Index));
		}
		// Remaining Set Elements
		for (i=0; i<this->GetRemainingSetElementsNumber(); i++) {
			int Index = this->RemainingSetIndexes->GetValue(i);
			this->ShowLine(const_cast<char*>("R"), i, Index, this->Weights->GetValue(Index), H->GetValue(Index));
		}
		// New Sample Element
		if (SampleIndex >=0) {
			this->ShowLine(const_cast<char*>("C"), -1, SampleIndex, this->Weights->GetValue(SampleIndex), H->GetValue(SampleIndex));
		}
		// TOT	
		char Line[81];
		sprintf(Line, "  TOTAL     %.16f", this->Weights->Sum());
		cout << Line << endl;
		// Last Line	
		cout << "-------------------------------------------------------------------------------" << endl;	
		if (SampleIndex == -1)
			cout << endl;
	}

	void OnlineSVR::ShowVariations (Vector<double>* H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double Lc1, double Lc2, Vector<double>* Ls, Vector<double>* Le, Vector<double>* Lr, int OperationType)
	{
		// Show Details
		this->ShowDetails(H,SampleIndex);
		// Line 1
		cout << "                                   " << "VARIATIONS" << "                                  " << endl;
		// Line 2
		cout << "-------------------------------------------------------------------------------" << endl;
		// Line 3
		cout << "  ELEMENT   Weights/H               BETA/GAMMA            VARIATION              " << endl;
		if (OperationType == OPERATION_LEARNING) {
			// Lc1
			this->ShowLine(const_cast<char*>("LC"), 1, SampleIndex, H->GetValue(SampleIndex), Gamma->GetValue(SampleIndex), Lc1);
			// Lc2
			this->ShowLine(const_cast<char*>("LC"), 2, SampleIndex, this->Weights->GetValue(SampleIndex), 0, Lc2);
		} 
		else {
			// Lc
			this->ShowLine(const_cast<char*>("LC"), -1, SampleIndex, this->Weights->GetValue(SampleIndex), Gamma->GetValue(SampleIndex), Lc1);
		}
		// Ls
		int i;
		for (i=0; i<this->GetSupportSetElementsNumber(); i++) {
			int Index = this->SupportSetIndexes->GetValue(i);
			this->ShowLine(const_cast<char*>("LS"), i, Index, this->Weights->GetValue(Index), Beta->GetValue(i+1), Ls->GetValue(i));
		}
		// Le
		for (i=0; i<this->GetErrorSetElementsNumber(); i++) {
			int Index = this->ErrorSetIndexes->GetValue(i);
			this->ShowLine(const_cast<char*>("LE"), i, Index, H->GetValue(Index), Gamma->GetValue(Index), Le->GetValue(i));
		}
		// Lr
		for (i=0; i<this->GetRemainingSetElementsNumber(); i++) {
			int Index = this->RemainingSetIndexes->GetValue(i);
			this->ShowLine(const_cast<char*>("LR"), i, Index, H->GetValue(Index), Gamma->GetValue(Index), Lr->GetValue(i));
		}
		// Total
		char Line[81];
		double Total = Beta->Sum();
		if (Beta->GetLength()>0) {
			Total -= Beta->GetValue(0);
		}
		sprintf(Line, "  TOTAL                           %.16f", Total);
		cout << Line << endl;	
		// Last Line
		cout << "-------------------------------------------------------------------------------" << endl << endl;
	}

	void OnlineSVR::ShowLine(char* SetName, int SetIndex, int SampleIndex, double Info1, double Info2)
	{
		// Initialization
		string Word = "";
		string Line = "";
		char* S1 = new char[50];
		char* S2 = new char[256];
		char* S3 = new char[256];
		char* S5 = new char[3];

		// Set Informations
		if (SetIndex>=0)
			sprintf(S1, "  %s%d (%d)          ", SetName, SetIndex, SampleIndex);
		else
			sprintf(S1, "  %s  (%d)          ", SetName, SampleIndex);
		Word = S1;
		Line += Word.substr(0,12);

		// Info1
		if (Info1 == -INF)
			sprintf(S2, "-INF                      ");
		else if (Info1 == INF)
			sprintf(S2, "INF                      ");
		else
			sprintf(S2, "%.16f                      ", Info1);
		Word = S2;
		Line += Word.substr(0,22);

		// Info2
		if (Info2 == -INF)
			sprintf(S3, "-INF                      ");
		else if (Info2 == INF)
			sprintf(S3, "INF                      ");
		else
			sprintf(S3, "%.16f                      ", Info2);
		Word = S3;
		Line += Word.substr(0,22);

		// End
		sprintf(S5, "\n");
		Line += S5;
		cout << Line;

		delete S1;
		delete S2;
		delete S3;
		delete S5;
	}

	void OnlineSVR::ShowLine(char* SetName, int SetIndex, int SampleIndex, double Info1, double Info2, double Info3)
	{
		// Initialization
		string Word = "";
		string Line = "";
		char* S1 = new char[50];
		char* S2 = new char[256];
		char* S3 = new char[256];
		char* S4 = new char[256];
		char* S5 = new char[3];

		// Set Informations
		if (SetIndex>=0)
			sprintf(S1, "  %s%d (%d)          ", SetName, SetIndex, SampleIndex);
		else
			sprintf(S1, "  %s  (%d)          ", SetName, SampleIndex);
		Word = S1;
		Line += Word.substr(0,12);

		// Info1
		if (Info1 == -INF)
			sprintf(S2, "-INF                      ");
		else if (Info1 == INF)
			sprintf(S2, "INF                      ");
		else
			sprintf(S2, "%.16f                      ", Info1);
		Word = S2;
		Line += Word.substr(0,22);

		// Info2
		if (Info2 == -INF)
			sprintf(S3, "-INF                      ");
		else if (Info2 == INF)
			sprintf(S3, "INF                      ");
		else
			sprintf(S3, "%.16f                      ", Info2);
		Word = S3;
		Line += Word.substr(0,22);

		// Info3		
		if (Info3 == -INF)
			sprintf(S4, "-INF                      ");
		else if (Info3 == INF)
			sprintf(S4, "INF                      ");
		else
			sprintf(S4, "%.16f                      ", Info3);
		Word = S4;
		Line += Word.substr(0,22);

		// End
		sprintf(S5, "\n");
		Line += S5;
		cout << Line;

		delete S1;
		delete S2;
		delete S3;
		delete S4;
		delete S5;
	}

	char* OnlineSVR::TimeToString (long Time)
	{	
		int MinuteTime = 60;
		int HourTime = MinuteTime*60;
		int DayTime = HourTime*24;

		int Days = static_cast<int>(Time/DayTime);
		int Hours = static_cast<int>((Time-Days*DayTime)/HourTime);
		int Minutes = static_cast<int>((Time-Days*DayTime-Hours*HourTime)/MinuteTime);
		int Seconds = static_cast<int>(Time-Days*DayTime-Hours*HourTime-Minutes*MinuteTime);
		
		char* Line = new char[80];
		if (Days>0) {
			sprintf(Line,"%d days, %d hours, %d minutes and %d seconds", Days, Hours, Minutes, Seconds);
		}
		else if (Hours>0) {
			sprintf(Line,"%d hours, %d minutes and %d seconds", Hours, Minutes, Seconds);
		}
		else if (Minutes>0) {
			sprintf(Line,"%d minutes and %d seconds", Minutes, Seconds);
		}
		else {
			sprintf(Line,"%d seconds", Seconds);
		}		
		return Line;
	}

}
	
#endif
