
#include "OnlineSVR.h"
#include <math.h>

using namespace onlinesvr; 

int main ()  
{ 

	// Make a new OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();

	// Set the parameters
	SVR->SetC(1);
	SVR->SetEpsilon(0.01);
	SVR->SetKernelType(OnlineSVR::KERNEL_RBF);
	SVR->SetKernelParam(30);
	SVR->SetVerbosity(OnlineSVR::VERBOSITY_NORMAL);	

	// Build the training set
	Matrix<double>* TrainingSetX = Matrix<double>::RandMatrix(200,1);
	Vector<double>* TrainingSetY = new Vector<double>();
	for (int i=0; i<TrainingSetX->GetLengthRows(); i++)
		TrainingSetY->Add(sin(TrainingSetX->GetValue(i,0)));
	
	// Train the OnlineSVR
	SVR->Train(TrainingSetX,TrainingSetY);

	// Show the OnlineSVR info
	SVR->ShowInfo();

	// Predict some new values
	Matrix<double>* TestSetX = new Matrix<double>();
	Vector<double>* X1 = new Vector<double>();
	Vector<double>* X2 = new Vector<double>();
	X1->Add(0);
	X2->Add(1);
	TestSetX->AddRowRef(X1);
	TestSetX->AddRowRef(X2);
	Vector<double>* PredictedY = SVR->Predict(TestSetX);
	cout << "f(0) = " << PredictedY->GetValue(0) << endl;
	cout << "f(1) = " << PredictedY->GetValue(1) << endl;	

	// Forget some samples
	Vector<int>* RemainingSamples = SVR->GetRemainingSetIndexes()->Clone();
	//SVR->Forget(RemainingSamples);

	// Save the OnlineSVR
	SVR->SaveOnlineSVR("Sin.svr");
	SVR->LoadOnlineSVR("Sin.svr");

	// Delete	
	delete SVR;
	delete TrainingSetX;
	delete TrainingSetY;	
	delete TestSetX;
	delete PredictedY;
	delete RemainingSamples;	
	
	// Variables
	Matrix<double>* X;
	Vector<double>* Y;
	Vector<double>* EpsilonList = new Vector<double>();
	Vector<double>* CList = new Vector<double>();
	Vector<double>* KernelParamList = new Vector<double>();

	// Params
	EpsilonList->Add(0.0001);
	EpsilonList->Add(0.001);
	EpsilonList->Add(0.01);
	EpsilonList->Add(0.1);
	EpsilonList->Add(1);	
	CList->Add(0.01);
	CList->Add(0.1);
	CList->Add(1);
	CList->Add(10);
	CList->Add(100);
	KernelParamList->Add(0.00001);
	KernelParamList->Add(0.0001);
	KernelParamList->Add(0.001);
	KernelParamList->Add(0.01);
	KernelParamList->Add(0.1);
	KernelParamList->Add(1);
	KernelParamList->Add(10);
	KernelParamList->Add(100);
	KernelParamList->Add(1000);

/*	EpsilonList->Add(0.06);
	EpsilonList->Add(0.07);
	EpsilonList->Add(0.08);
	EpsilonList->Add(0.09);
	EpsilonList->Add(0.10);
	EpsilonList->Add(0.11);
	EpsilonList->Add(0.12);
	EpsilonList->Add(0.13);
	EpsilonList->Add(0.14);
	EpsilonList->Add(0.15);
	CList->Add(100);
	KernelParamList->Add(0.00002);
	KernelParamList->Add(0.00004);
	KernelParamList->Add(0.00006);
	KernelParamList->Add(0.00008);
	KernelParamList->Add(0.00010);
	KernelParamList->Add(0.00012);
	KernelParamList->Add(0.00014);
	KernelParamList->Add(0.00016);
	KernelParamList->Add(0.00018);
	KernelParamList->Add(0.00020);
*/

	// Load	
//	OnlineSVR::Import("C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 04-10\\Old\\NormalizedGravity1.txt", &X, &Y);	
//	OnlineSVR::CrossValidation(X, Y, EpsilonList, CList, KernelParamList, 3, "CrossValidation2.txt");
	//OnlineSVR::LeaveOneOut(X, Y, EpsilonList, CList, KernelParamList, "LeaveOneOut1.txt");

/*
	// V1	
	Vector<double>* Errors = new Vector<double>();
	OnlineSVR* SVR = new OnlineSVR();
	int i;

	SVR->SetEpsilon(0.1);
	SVR->SetC(1);	
	SVR->SetKernelType(OnlineSVR::KERNEL_RBF);
	SVR->SetKernelParam(1);
	SVR->SetVerbosity(OnlineSVR::VERBOSITY_NO_MESSAGES);		
	
	OnlineSVR::Import("C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\NormalizedGravity1.txt", &X, &Y);
	Matrix<double>* X1 = X->ExtractRows(1,900);
	Matrix<double>* X2 = X->ExtractRows(901,1000);
	Vector<double>* Y1 = Y->Extract(1,900);
	Vector<double>* Y2 = Y->Extract(901,1000);

	for (i=0; i<X1->GetLengthRows(); i++) 
	{
		// Error
		cout << "Training " << i+1 << "/" << X1->GetLengthRows() << endl;
		Errors->Add(SVR->Margin(X->GetRowRef(i),Y->GetValue(i)));
		SVR->Train(X1->GetRowRef(i),Y1->GetValue(i));
		if (i%20==0) 
		{
			Vector<double>* TrainingErrors;
			char FileName[80];
			TrainingErrors = SVR->Margin(X2,Y2);
			sprintf(FileName, "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\V1\\%d.txt", i);
			TrainingErrors->Save(FileName);
		}

	}
	Errors->Save("C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\V1\\PredictErrorsV1.txt");
	SVR->ShowInfo();


	// V2	
	Errors->Clear();
	SVR = new OnlineSVR();
	SVR->SetEpsilon(0.1);
	SVR->SetC(1);	
	SVR->SetKernelType(OnlineSVR::KERNEL_RBF);
	SVR->SetKernelParam(1);
	SVR->SetVerbosity(OnlineSVR::VERBOSITY_NO_MESSAGES);			
	OnlineSVR::Import("C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\NormalizedGravity2.txt", &X, &Y);
	X1 = X->ExtractRows(1,900);
	X2 = X->ExtractRows(901,1000);
	Y1 = Y->Extract(1,900);
	Y2 = Y->Extract(901,1000);

	for (i=0; i<X1->GetLengthRows(); i++) 
	{
		// Error
		cout << "Training " << i+1 << "/" << X1->GetLengthRows() << endl;
		Errors->Add(SVR->Margin(X->GetRowRef(i),Y->GetValue(i)));
		SVR->Train(X1->GetRowRef(i),Y1->GetValue(i));
		if (i%20==0) 
		{
			Vector<double>* TrainingErrors;
			char FileName[80];
			TrainingErrors = SVR->Margin(X2,Y2);
			sprintf(FileName, "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\V2\\%d.txt", i);
			TrainingErrors->Save(FileName);
		}

	}
	Errors->Save("C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\V2\\PredictErrorsV2.txt");
	SVR->ShowInfo();	

	// V3
	Errors->Clear();
	SVR = new OnlineSVR();
	SVR->SetEpsilon(0.1);
	SVR->SetC(1);	
	SVR->SetKernelType(OnlineSVR::KERNEL_RBF);
	SVR->SetKernelParam(1);
	SVR->SetVerbosity(OnlineSVR::VERBOSITY_NO_MESSAGES);
	OnlineSVR::Import("C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\NormalizedGravity3.txt", &X, &Y);
	X1 = X->ExtractRows(1,900);
	X2 = X->ExtractRows(901,1000);
	Y1 = Y->Extract(1,900);
	Y2 = Y->Extract(901,1000);

	for (i=0; i<X1->GetLengthRows(); i++) 
	{
		// Error
		cout << "Training " << i+1 << "/" << X1->GetLengthRows() << endl;
		Errors->Add(SVR->Margin(X->GetRowRef(i),Y->GetValue(i)));
		SVR->Train(X1->GetRowRef(i),Y1->GetValue(i));
		if (i%20==0) 
		{
			Vector<double>* TrainingErrors;
			char FileName[80];
			TrainingErrors = SVR->Margin(X2,Y2);
			sprintf(FileName, "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\V3\\%d.txt", i);
			TrainingErrors->Save(FileName);
		}

	}
	Errors->Save("C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\RobotArm\\Dati 12-10\\V3\\PredictErrorsV3.txt");
	SVR->ShowInfo();	
	*/

	system("pause");
    return 0;
}
