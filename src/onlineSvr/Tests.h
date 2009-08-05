#ifndef TESTS_H
#define TESTS_H

#include <iostream>
#include <math.h>
#include "OnlineSVR.h"

namespace onlinesvr
{

	// Test List
	void TestLinear ();
	void TestPerformanceStabilize ();
	void TestRobotArm ();

	// Other Functions
	double f (double x);
	OnlineSVR* BuildOnlineSVR (double C, double Epsilon, int KernelType, double KernelParam, int Verbosity);




	////////////////////////////
	// METHODS IMPLEMENTATION //
	////////////////////////////


	// Test List
	void TestLinear() 
	{
		// Parameters
		int IterationNumber = 10;
		int SamplesNumber = 100;
		int SamplesDimension = 2;

		// Initialization
		OnlineSVR* SVM = new OnlineSVR();
		SVM->SetC(10);
		SVM->SetEpsilon(0.0001);
		SVM->SetKernelType(SVM->KERNEL_LINEAR);
		SVM->SetKernelParam(30);
		SVM->SetVerbosity(0);
		SVM->SetStabilizedLearning(true);

		for (int i=0; i<IterationNumber; i++)
		{
			// Build the Data
			Matrix<double>* X = new Matrix<double>();
			for (int j=0; j<SamplesNumber; j++) {
				Vector<double>* V = new Vector<double>(SamplesDimension);
				for (int k=0; k<SamplesDimension; k++)
					V->Add(rand());
				X->AddRowRef(V);
			}
			Vector<double>* Y = new Vector<double>(SamplesNumber);
			for (int j=0; j<SamplesNumber; j++)
				Y->Add(f(X->Values->Values[j]->Values[0]));
			Vector<int>* Indexes = new Vector<int>(SamplesNumber);
			for (int j=0; j<SamplesNumber; j++)
				Indexes->Add(j);

			// Training
			SVM->Train(X,Y);
			SVM->ShowInfo();
			SVM->Forget(Indexes);
			SVM->ShowInfo();
			cout << "End Test " << i << endl << endl;		

			// Clear
			delete X;
			delete Y;
			delete Indexes;

			//system("pause");
		}

	}

	void TestPerformanceStabilize ()
	{
		// Parameters
		int TestNumber = 10;
		// Starting SVR Parameters
		OnlineSVR * SVR;
		double C = 1;
		double Epsilon = 0.01;
		int KernelType = SVR->KERNEL_RBF;
		double KernelParam = 30;
		int Verbosity = 0;
		// Changed Parameters    
		double IncrementC = 2;
		double DecrementC = 0.5;
		double IncrementEpsilon = 0.1;
		double DecrementEpsilon = 0.001;
		double IncrementKernelParam = 100;
		double DecrementKernelParam = 25;

		// Samples Number to Test
		Vector<int>* SamplesNumberList = new Vector<int>();
		SamplesNumberList->Add(10);
		SamplesNumberList->Add(20);
		SamplesNumberList->Add(50);
		SamplesNumberList->Add(100);
		SamplesNumberList->Add(150);
		SamplesNumberList->Add(200);
		SamplesNumberList->Add(300);
		SamplesNumberList->Add(400);
		SamplesNumberList->Add(500);
		SamplesNumberList->Add(600);
		SamplesNumberList->Add(700);
		SamplesNumberList->Add(800);
		SamplesNumberList->Add(900);
		SamplesNumberList->Add(1000);

		// TITLE
		cout << endl << "PERFORMANCE TEST  -  Stabilizing VS New Learning when a parameter change" << endl << endl;

		// Test 1 - Increasing C
		cout << "TEST1: INCREASING C" << endl;
		Matrix<int>* IncrementCFlops = new Matrix<int>();
		for (int i=0; i<SamplesNumberList->GetLength(); i++) {
			cout << "Training with " << SamplesNumberList->Values[i] <<  " samples" << endl;
			int TotalFlops1 = 0;
			int TotalFlops2 = 0;
			for (int j=0; j<TestNumber; j++) {
				// Build TrainingSet
				Matrix<double>* X = X->RandMatrix(SamplesNumberList->Values[i],1);
				Vector<double>* Y = new Vector<double>(SamplesNumberList->Values[i]);
				for (int k=0; k<SamplesNumberList->Values[i]; k++)
					Y->Add(f(X->Values->Values[k]->Values[0]));
				// Training
				OnlineSVR* SVR = BuildOnlineSVR(C,Epsilon,KernelType,KernelParam,Verbosity);
				SVR->Train(X,Y);
				// Stabilize
				SVR->SetC(IncrementC);
				int Flops1 = SVR->Stabilize();
				TotalFlops1 += Flops1;
				cout << "  >> Stabilize   : " << Flops1 << " flops" << endl;
				delete SVR;
				// New Learning
				SVR = BuildOnlineSVR(IncrementC,Epsilon,KernelType,KernelParam,Verbosity);
				int Flops2 = SVR->Train(X,Y);
				TotalFlops2 += Flops2;
				cout << "  >> New Learning: " << Flops2 << " flops" << endl;
				delete SVR;
				delete X;
				delete Y;
			}
			// Save Results
			Vector<int>* Results = new Vector<int>(3);
			Results->Add(SamplesNumberList->Values[i]);
			Results->Add(TotalFlops1/TestNumber);
			Results->Add(TotalFlops2/TestNumber);
			IncrementCFlops->AddRowRef(Results);
		}
		IncrementCFlops->Save(const_cast<char*>("IncrementC.txt"));

		// Test 2 - Decreasing C
		cout << endl << "TEST2: DECREASING C" << endl;
		Matrix<int>* DecrementCFlops = new Matrix<int>();
		for (int i=0; i<SamplesNumberList->GetLength(); i++) {
			cout << "Training with " << SamplesNumberList->Values[i] <<  " samples" << endl;
			int TotalFlops1 = 0;
			int TotalFlops2 = 0;
			for (int j=0; j<TestNumber; j++) {
				// Build TrainingSet
				Matrix<double>* X = X->RandMatrix(SamplesNumberList->Values[i],1);
				Vector<double>* Y = new Vector<double>(SamplesNumberList->Values[i]);
				for (int k=0; k<SamplesNumberList->Values[i]; k++)
					Y->Add(f(X->Values->Values[k]->Values[0]));
				// Training
				OnlineSVR* SVR = BuildOnlineSVR(C,Epsilon,KernelType,KernelParam,Verbosity);
				SVR->Train(X,Y);
				// Stabilize
				SVR->SetC(IncrementC);
				int Flops1 = SVR->Stabilize();
				TotalFlops1 += Flops1;
				cout << "  >> Stabilize   : " << Flops1 << " flops" << endl;	
				delete SVR;
				// New Learning
				SVR = BuildOnlineSVR(DecrementC,Epsilon,KernelType,KernelParam,Verbosity);
				int Flops2 = SVR->Train(X,Y);
				TotalFlops2 += Flops2;
				cout << "  >> New Learning: " << Flops2 << " flops" << endl;
				delete SVR;
				delete X;
				delete Y;
			}
			// Save Results
			Vector<int>* Results = new Vector<int>(3);
			Results->Add(SamplesNumberList->Values[i]);
			Results->Add(TotalFlops1/TestNumber);
			Results->Add(TotalFlops2/TestNumber);
			DecrementCFlops->AddRowRef(Results);
		}
		DecrementCFlops->Save(const_cast<char*>("DecrementC.txt"));

		// Test 3 - Increasing Epsilon
		cout << endl << "TEST3: INCREASING EPSILON" << endl;
		Matrix<int>* IncrementEpsilonFlops = new Matrix<int>();
		for (int i=0; i<SamplesNumberList->GetLength(); i++) {
			cout << "Training with " << SamplesNumberList->Values[i] <<  " samples" << endl;
			int TotalFlops1 = 0;
			int TotalFlops2 = 0;
			for (int j=0; j<TestNumber; j++) {
				// Build TrainingSet
				Matrix<double>* X = X->RandMatrix(SamplesNumberList->Values[i],1);
				Vector<double>* Y = new Vector<double>(SamplesNumberList->Values[i]);
				for (int k=0; k<SamplesNumberList->Values[i]; k++)
					Y->Add(f(X->Values->Values[k]->Values[0]));
				// Training
				OnlineSVR* SVR = BuildOnlineSVR(C,Epsilon,KernelType,KernelParam,Verbosity);
				SVR->Train(X,Y);
				// Stabilize
				SVR->SetC(IncrementC);
				int Flops1 = SVR->Stabilize();
				TotalFlops1 += Flops1;
				cout << "  >> Stabilize   : " << Flops1 << " flops" << endl;			
				delete SVR;
				// New Learning
				SVR = BuildOnlineSVR(C,IncrementEpsilon,KernelType,KernelParam,Verbosity);
				int Flops2 = SVR->Train(X,Y);
				TotalFlops2 += Flops2;
				cout << "  >> New Learning: " << Flops2 << " flops" << endl;
				delete SVR;
				delete X;
				delete Y;
			}
			// Save Results
			Vector<int>* Results = new Vector<int>(3);
			Results->Add(SamplesNumberList->Values[i]);
			Results->Add(TotalFlops1/TestNumber);
			Results->Add(TotalFlops2/TestNumber);
			IncrementEpsilonFlops->AddRowRef(Results);
		}
		IncrementEpsilonFlops->Save(const_cast<char*>("IncrementEpsilon.txt"));

		// Test 4 - Decreasing C
		cout << endl << "TEST4: DECREASING EPSILON" << endl;
		Matrix<int>* DecrementEpsilonFlops = new Matrix<int>();
		for (int i=0; i<SamplesNumberList->GetLength(); i++) {
			cout << "Training with " << SamplesNumberList->Values[i] <<  " samples" << endl;
			int TotalFlops1 = 0;
			int TotalFlops2 = 0;
			for (int j=0; j<TestNumber; j++) {
				// Build TrainingSet
				Matrix<double>* X = X->RandMatrix(SamplesNumberList->Values[i],1);
				Vector<double>* Y = new Vector<double>(SamplesNumberList->Values[i]);
				for (int k=0; k<SamplesNumberList->Values[i]; k++)
					Y->Add(f(X->Values->Values[k]->Values[0]));
				// Training
				OnlineSVR* SVR = BuildOnlineSVR(C,Epsilon,KernelType,KernelParam,Verbosity);
				SVR->Train(X,Y);
				// Stabilize
				SVR->SetC(IncrementC);
				int Flops1 = SVR->Stabilize();
				TotalFlops1 += Flops1;
				cout << "  >> Stabilize   : " << Flops1 << " flops" << endl;			
				delete SVR;
				// New Learning
				SVR = BuildOnlineSVR(C,DecrementEpsilon,KernelType,KernelParam,Verbosity);
				int Flops2 = SVR->Train(X,Y);
				TotalFlops2 += Flops2;
				cout << "  >> New Learning: " << Flops2 << " flops" << endl;
				delete SVR;
				delete X;
				delete Y;			
			}
			// Save Results
			Vector<int>* Results = new Vector<int>(3);
			Results->Add(SamplesNumberList->Values[i]);
			Results->Add(TotalFlops1/TestNumber);
			Results->Add(TotalFlops2/TestNumber);
			DecrementEpsilonFlops->AddRowRef(Results);
		}
		DecrementEpsilonFlops->Save(const_cast<char*>("DecrementEpsilon.txt"));

		// Test 5 - Increasing KernelParam
		cout << endl << "TEST5: INCREASING KERNEL PARAM" << endl;
		Matrix<int>* IncrementKernelParamFlops = new Matrix<int>();
		for (int i=0; i<SamplesNumberList->GetLength(); i++) {
			cout << "Training with " << SamplesNumberList->Values[i] <<  " samples" << endl;
			int TotalFlops1 = 0;
			int TotalFlops2 = 0;
			for (int j=0; j<TestNumber; j++) {
				// Build TrainingSet
				Matrix<double>* X = X->RandMatrix(SamplesNumberList->Values[i],1);
				Vector<double>* Y = new Vector<double>(SamplesNumberList->Values[i]);
				for (int k=0; k<SamplesNumberList->Values[i]; k++)
					Y->Add(f(X->Values->Values[k]->Values[0]));
				// Training
				OnlineSVR* SVR = BuildOnlineSVR(C,Epsilon,KernelType,KernelParam,Verbosity);
				SVR->Train(X,Y);
				// Stabilize
				SVR->SetC(IncrementC);
				int Flops1 = SVR->Stabilize();
				TotalFlops1 += Flops1;
				cout << "  >> Stabilize   : " << Flops1 << " flops" << endl;			
				delete SVR;
				// New Learning
				SVR = BuildOnlineSVR(C,Epsilon,KernelType,IncrementKernelParam,Verbosity);
				int Flops2 = SVR->Train(X,Y);
				TotalFlops2 += Flops2;
				cout << "  >> New Learning: " << Flops2 << " flops" << endl;
				delete SVR;
				delete X;
				delete Y;
			}
			// Save Results
			Vector<int>* Results = new Vector<int>(3);
			Results->Add(SamplesNumberList->Values[i]);
			Results->Add(TotalFlops1/TestNumber);
			Results->Add(TotalFlops2/TestNumber);
			IncrementKernelParamFlops->AddRowRef(Results);
		}
		IncrementKernelParamFlops->Save(const_cast<char*>("IncrementKernelParam.txt"));

		// Test 6 - Decreasing KernelParam
		cout << endl << "TEST6: DECREASING KERNEL PARAM" << endl;
		Matrix<int>* DecrementKernelParamFlops = new Matrix<int>();
		for (int i=0; i<SamplesNumberList->GetLength(); i++) {
			cout << "Training with " << SamplesNumberList->Values[i] <<  " samples" << endl;
			int TotalFlops1 = 0;
			int TotalFlops2 = 0;
			for (int j=0; j<TestNumber; j++) {
				// Build TrainingSet
				Matrix<double>* X = X->RandMatrix(SamplesNumberList->Values[i],1);
				Vector<double>* Y = new Vector<double>(SamplesNumberList->Values[i]);
				for (int k=0; k<SamplesNumberList->Values[i]; k++)
					Y->Add(f(X->Values->Values[k]->Values[0]));
				// Training
				OnlineSVR* SVR = BuildOnlineSVR(C,Epsilon,KernelType,KernelParam,Verbosity);
				SVR->Train(X,Y);
				// Stabilize
				SVR->SetC(IncrementC);
				int Flops1 = SVR->Stabilize();
				TotalFlops1 += Flops1;
				cout << "  >> Stabilize   : " << Flops1 << " flops" << endl;			
				delete SVR;
				// New Learning
				SVR = BuildOnlineSVR(C,Epsilon,KernelType,DecrementKernelParam,Verbosity);
				int Flops2 = SVR->Train(X,Y);
				TotalFlops2 += Flops2;
				cout << "  >> New Learning: " << Flops2 << " flops" << endl;
				delete SVR;
				delete X;
				delete Y;
			}
			// Save Results
			Vector<int>* Results = new Vector<int>(3);
			Results->Add(SamplesNumberList->Values[i]);
			Results->Add(TotalFlops1/TestNumber);
			Results->Add(TotalFlops2/TestNumber);
			DecrementKernelParamFlops->AddRowRef(Results);
		}
		DecrementKernelParamFlops->Save(const_cast<char*>("DecrementKernelParam.txt"));

		// Free
		delete SamplesNumberList;
	}

	void TestRobotArm()
	{
		// Parameters
		int TrainingSetSize = 562;
		int ValidationSetSize = 562;

		// Load the data
		Matrix<double>* AngularPositions;
		Matrix<double>* MotorCurrents;
		Matrix<double>* AppliedVoltages;
		OnlineSVR::Import(const_cast<char*>("RobotArm1.txt"), &AngularPositions, &MotorCurrents, &AppliedVoltages);

		// Test Angular Positions - AppliedVoltages
		// 1
		Matrix<double>*X = AngularPositions;//->ExtractCols(0,0);
		Vector<double>* Y = AppliedVoltages->GetColCopy(0);	
		// Extract training and validation set
		Matrix<double>* TrainingSetX = X->ExtractRows(0,TrainingSetSize-1);
		Vector<double>* TrainingSetY = Y->Extract(0,TrainingSetSize-1);
		Matrix<double>* ValidationSetX = X->ExtractRows(0,ValidationSetSize-1);
		Vector<double>* ValidationSetY = Y->Extract(0,ValidationSetSize-1);
		OnlineSVR* SVR1 = new OnlineSVR();
		SVR1->SetSaveKernelMatrix(true);
		SVR1->SelfLearning(TrainingSetX, TrainingSetY, ValidationSetX, ValidationSetY, 1);
		SVR1->SaveOnlineSVR(const_cast<char*>("RobotArmAV1.svr"));
		// Clear
		delete X;
		delete Y;
		delete TrainingSetX;
		delete TrainingSetY;
		delete ValidationSetX;
		delete ValidationSetY;
		delete SVR1;

		// Clear
		delete AngularPositions;
		delete MotorCurrents;
		delete AppliedVoltages;
	}


	// Other Functions
	double f (double x)
	{
		return sin(x*10);
	}

	OnlineSVR* BuildOnlineSVR (double C, double Epsilon, int KernelType, double KernelParam, int Verbosity)
	{
		OnlineSVR* SVR = new OnlineSVR();
		SVR->SetC(C);
		SVR->SetEpsilon(Epsilon);
		SVR->SetKernelType(KernelType);
		SVR->SetKernelParam(KernelParam);
		SVR->SetVerbosity(Verbosity);
		return SVR;
	}

}
	
#endif
