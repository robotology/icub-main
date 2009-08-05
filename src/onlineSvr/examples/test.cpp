#include "OnlineSVR.h"
#include <math.h>

using namespace onlinesvr; 

void TestLinear ();
void TestSin ();
void TestRand ();
void TestErrorTollerance ();
void TestErrorTolleranceAdvanced ();
void TestEpsilon ();
void TestC ();


int main ()  
{ 
	TestErrorTolleranceAdvanced();
		
	system("pause");
    return 0;
}



void TestLinear ()
{
	// Make a new OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();

	// Set the parameters
	SVR->SetC(1);
	SVR->SetEpsilon(0.01);
	SVR->SetKernelType(OnlineSVR::KERNEL_LINEAR);
	SVR->SetKernelParam(30);
	SVR->SetVerbosity(1);

	// Build the training set
	Matrix<double> *TrainingSetX, *X;
	Vector<double> *TrainingSetY, *Y;
	char* FileName = "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\BuildDataSet\\Linear.txt";
	OnlineSVR::Import(FileName, &X, &Y);
	TrainingSetX = X->ExtractRows(0,4999);
	TrainingSetY = Y->Extract(0,4999);
	
	// Train the OnlineSVR
	SVR->Train(TrainingSetX,TrainingSetY);

	// Show the OnlineSVR info
	SVR->ShowInfo();

	// Forget samples
	Vector<int>* Indexes = Vector<int>::GetSequence(0,1,SVR->GetSamplesTrainedNumber()-1);
	SVR->Forget(Indexes);

	// Delete	
	delete SVR;
	delete TrainingSetX;
	delete TrainingSetY;
	delete X;
	delete Y;
	delete Indexes;
}


void TestSin ()
{
	// Make a new OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();

	// Set the parameters
	SVR->SetC(1);
	SVR->SetEpsilon(0.01);
	SVR->SetKernelType(OnlineSVR::KERNEL_LINEAR);
	SVR->SetKernelParam(30);
	SVR->SetVerbosity(1);

	// Build the training set
	Matrix<double> *TrainingSetX, *X;
	Vector<double> *TrainingSetY, *Y;
	char* FileName = "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\BuildDataSet\\Sin.txt";
	OnlineSVR::Import(FileName, &X, &Y);
	TrainingSetX = X->ExtractRows(0,4999);
	TrainingSetY = Y->Extract(0,4999);
	
	// Train the OnlineSVR
	SVR->Train(TrainingSetX,TrainingSetY);

	// Show the OnlineSVR info
	SVR->ShowInfo();

	// Forget samples
	Vector<int>* Indexes = Vector<int>::GetSequence(0,1,SVR->GetSamplesTrainedNumber()-1);
	SVR->Forget(Indexes);

	// Delete	
	delete SVR;
	delete TrainingSetX;
	delete TrainingSetY;
	delete X;
	delete Y;
	delete Indexes;
}


void TestRand ()
{
	// Make a new OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();

	// Set the parameters
	SVR->SetC(1);
	SVR->SetEpsilon(0.01);
	SVR->SetKernelType(OnlineSVR::KERNEL_RBF);
	SVR->SetKernelParam(30);
	SVR->SetVerbosity(1);

	// Build the training set
	Matrix<double> *TrainingSetX, *X;
	Vector<double> *TrainingSetY, *Y;
	char* FileName = "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\BuildDataSet\\Rand.txt";
	OnlineSVR::Import(FileName, &X, &Y);
	TrainingSetX = X->ExtractRows(0,4999);
	TrainingSetY = Y->Extract(0,4999);
	
	// Train the OnlineSVR
	SVR->Train(TrainingSetX,TrainingSetY);

	// Show the OnlineSVR info
	SVR->ShowInfo();

	// Forget samples
	Vector<int>* Indexes = Vector<int>::GetSequence(0,1,SVR->GetSamplesTrainedNumber()-1);
	SVR->Forget(Indexes);

	// Delete	
	delete SVR;
	delete TrainingSetX;
	delete TrainingSetY;
	delete X;
	delete Y;
	delete Indexes;
}




void TestErrorTollerance ()
{
	// Make a new OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();

	// Set the parameters
	SVR->SetC(1);
	SVR->SetEpsilon(0.01);
	SVR->SetKernelType(OnlineSVR::KERNEL_LINEAR);
	SVR->SetKernelParam(30);
	SVR->SetVerbosity(1);
	SVR->SetAutoErrorTollerance(false);
	SVR->SetErrorTollerance(1e-10);

	// Build the training set
	Matrix<double> *TrainingSetX, *X;
	Vector<double> *TrainingSetY, *Y;
	char* FileName = "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\BuildDataSet\\Sin.txt";
	OnlineSVR::Import(FileName, &X, &Y);
	TrainingSetX = X->ExtractRows(0,4999);
	TrainingSetY = Y->Extract(0,4999);
	
	// Train the OnlineSVR
	SVR->Train(TrainingSetX,TrainingSetY);

	// Show the OnlineSVR info
	SVR->ShowInfo();

	// Forget samples
	Vector<int>* Indexes = Vector<int>::GetSequence(0,1,SVR->GetSamplesTrainedNumber()-1);
	SVR->Forget(Indexes);

	// Delete	
	delete SVR;
	delete TrainingSetX;
	delete TrainingSetY;
	delete X;
	delete Y;
	delete Indexes;
}

void TestErrorTolleranceAdvanced ()
{
	// Make a new OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();

	// Set the parameters
	SVR->SetC(1e-10);
	SVR->SetEpsilon(1e-15);
	SVR->SetKernelType(OnlineSVR::KERNEL_RBF);
	SVR->SetKernelParam(30);
	SVR->SetVerbosity(1);
	SVR->SetAutoErrorTollerance(false);
	SVR->SetErrorTollerance(1e-30);

	// Build the training set
	Matrix<double> *TrainingSetX, *X;
	Vector<double> *TrainingSetY, *Y;
	char* FileName = "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\BuildDataSet\\Rand.txt";
	OnlineSVR::Import(FileName, &X, &Y);
	TrainingSetX = X->ExtractRows(0,4999);
	TrainingSetY = Y->Extract(0,4999);
	
	// Train the OnlineSVR
	SVR->Train(TrainingSetX,TrainingSetY);

	// Show the OnlineSVR info
	SVR->ShowInfo();
	SVR->SaveOnlineSVR("Rand.svr");

	// Forget samples
	Vector<int>* Indexes = Vector<int>::GetSequence(0,1,SVR->GetSamplesTrainedNumber()-1);
	SVR->Forget(Indexes);

	// Delete	
	delete SVR;
	delete TrainingSetX;
	delete TrainingSetY;
	delete X;
	delete Y;
	delete Indexes;
}

void TestEpsilon ()
{
	// Make a new OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();

	// Set the parameters
	SVR->SetC(1);
	SVR->SetEpsilon(1e-10);
	SVR->SetKernelType(OnlineSVR::KERNEL_LINEAR);
	SVR->SetKernelParam(30);
	SVR->SetVerbosity(1);

	// Build the training set
	Matrix<double> *TrainingSetX, *X;
	Vector<double> *TrainingSetY, *Y;
	char* FileName = "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\BuildDataSet\\Sin.txt";
	OnlineSVR::Import(FileName, &X, &Y);
	TrainingSetX = X->ExtractRows(0,4999);
	TrainingSetY = Y->Extract(0,4999);
	
	// Train the OnlineSVR
	SVR->Train(TrainingSetX,TrainingSetY);

	// Show the OnlineSVR info
	SVR->ShowInfo();

	// Forget samples
	Vector<int>* Indexes = Vector<int>::GetSequence(0,1,SVR->GetSamplesTrainedNumber()-1);
	SVR->Forget(Indexes);

	// Delete	
	delete SVR;
	delete TrainingSetX;
	delete TrainingSetY;
	delete X;
	delete Y;
	delete Indexes;
}

void TestC ()
{
	// Make a new OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();

	// Set the parameters
	SVR->SetC(1e-10);
	SVR->SetEpsilon(0.1);
	SVR->SetKernelType(OnlineSVR::KERNEL_LINEAR);
	SVR->SetKernelParam(30);
	SVR->SetVerbosity(1);

	// Build the training set
	Matrix<double> *TrainingSetX, *X;
	Vector<double> *TrainingSetY, *Y;
	char* FileName = "C:\\Documents and Settings\\Francesco\\Desktop\\OnlineSVR Matlab\\BuildDataSet\\Sin.txt";
	OnlineSVR::Import(FileName, &X, &Y);
	TrainingSetX = X->ExtractRows(0,4999);
	TrainingSetY = Y->Extract(0,4999);
	
	// Train the OnlineSVR
	SVR->Train(TrainingSetX,TrainingSetY);

	// Show the OnlineSVR info
	SVR->ShowInfo();

	// Forget samples
	Vector<int>* Indexes = Vector<int>::GetSequence(0,1,SVR->GetSamplesTrainedNumber()-1);
	SVR->Forget(Indexes);

	// Delete	
	delete SVR;
	delete TrainingSetX;
	delete TrainingSetY;
	delete X;
	delete Y;
	delete Indexes;
}

