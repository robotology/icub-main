#include <iostream>
#include <math.h>
#include <string.h>
#include "OnlineSVR.h"

using namespace std;
using namespace onlinesvr;

// portable implementation of non-standard stricmp
// just captures true-false behavior, not order of comparison
int pstricmp(const char *str1, const char *str2) {
  if (strlen(str1)!=strlen(str2)) {
    return 1;
  }
  for (int i=0; i<strlen(str1); i++) {
    if (str1[i]!=str2[i]) {
      return 1;
    }
  }
  return 0;
}


/*
PARAMETER LIST
OPERATIONS: -Train -Forget -Stabilize -Predict -Margin
PARAMETERS: -C -Epsilon -KernelType -KernelParam -KernelParam2 -Verbosity -ErrorTollerance
OPTIONS   : -SaveKernelMatrix -NotSaveKernelMatrix -StabilizedLearning -NotStabilizedLearning 
I/O       : -Load -Save -Data
*/

void ShowUsage ();

int main (int argc, char* argv[])
{
	// Default Parameters
	bool NewOnlineSVR = true;
	int LearningType = -1;
	char InputFile[256] = "";
	char OutputFile[256] = "";	
	char DataFile[256] = "";
	double C = 30;
	double Epsilon = 0.1;
	int KernelType = OnlineSVR::KERNEL_RBF;
	double KernelParam = 30;
	double KernelParam2 = 0;
	double ErrorTollerance = Epsilon/10;
	int Verbosity = 1;
	bool StabilizedLearning = true;
	bool SaveKernelMatrix = true;	
	bool ChangedC = false;
	bool ChangedEpsilon = false;
	bool ChangedKernelType = false;
	bool ChangedKernelParam = false;
	bool ChangedKernelParam2 = false;
	bool ChangedErrorTollerance = false;
	bool ChangedVerbosity = false;
	bool ChangedStabilizedLearning = false;
	bool ChangedSaveKernelMatrix = false;	

	// Check Wrong Params
	if (argc==1) {
		//cerr << "Parameters not valid." << endl;		
		ShowUsage();
		system("pause");
		exit(0);
	}

	// Load Params
	for (int i=1; i<argc; i++) {
		// Train
		if (pstricmp(argv[i],"-Train")==0) {
			LearningType = OnlineSVR::OPERATION_LEARNING;
			//cout << "Operation = Train" << endl;
		}
		// Forget
		else if (pstricmp(argv[i],"-Forget")==0) {
			LearningType = OnlineSVR::OPERATION_UNLEARNING;
			//cout << "Operation = Forgetting" << endl;
		}
		// Stabilize
		else if (pstricmp(argv[i],"-Stabilize")==0) {
			LearningType = OnlineSVR::OPERATION_STABILIZING;
			//cout << "Operation = Stabilizing" << endl;
		}
		// Predict
		else if (pstricmp(argv[i],"-Predict")==0) {
			LearningType = OnlineSVR::OPERATION_PREDICT;
			//cout << "Operation = Predict" << endl;
		}
		// Margin
		else if (pstricmp(argv[i],"-Margin")==0) {
			LearningType = OnlineSVR::OPERATION_MARGIN;
			//cout << "Operation = Margin" << endl;
		}
		// C
		else if (pstricmp(argv[i],"-C")==0) {
			float FloatC = 0;
			sscanf (argv[++i], "%f", &FloatC);
			C = static_cast<double>(FloatC);
			ChangedC = true;
			//cout << "C = " << C << endl;
		} 
		// Epsilon
		else if (pstricmp(argv[i],"-Epsilon")==0) {
			float FloatEpsilon = 0;
			sscanf (argv[++i], "%f", &FloatEpsilon);
			Epsilon = static_cast<double>(FloatEpsilon);
			ChangedEpsilon = true;
			//cout << "Epsilon = " << Epsilon << endl;
		}
		// KernelType
		else if (pstricmp(argv[i],"-KernelType")==0) {
			i++;
			if (pstricmp(argv[i],"Linear")==0)
				KernelType = OnlineSVR::KERNEL_LINEAR;
			else if (pstricmp(argv[i],"Polynomial")==0)
				KernelType = OnlineSVR::KERNEL_POLYNOMIAL;
			else if (pstricmp(argv[i],"RBF")==0)
				KernelType = OnlineSVR::KERNEL_RBF;
			else if (pstricmp(argv[i],"GaussianRBF")==0)
				KernelType = OnlineSVR::KERNEL_RBF_GAUSSIAN;
			else if (pstricmp(argv[i],"ExponentialRBF")==0)
				KernelType = OnlineSVR::KERNEL_RBF_EXPONENTIAL;
			else if (pstricmp(argv[i],"MLP")==0)
				KernelType = OnlineSVR::KERNEL_MLP;
			else {
				cerr << "Error: Kernel Type not valid!!" << endl;
				system("pause");
				exit (1);
			}
			ChangedKernelType = true;
		}
		// KernelParam
		else if (pstricmp(argv[i],"-KernelParam")==0) {
			float FloatKernelParam = 0;
			sscanf (argv[++i], "%f", &FloatKernelParam);
			KernelParam = static_cast<double>(FloatKernelParam);
			ChangedKernelParam = true;
			//cout << "KernelParam = " << KernelParam << endl;
		}
		// KernelParam 2
		else if (pstricmp(argv[i],"-KernelParam2")==0) {			
			float FloatKernelParam2 = 0;
			sscanf (argv[++i], "%f", &FloatKernelParam2);
			KernelParam2 = static_cast<double>(FloatKernelParam2);
			ChangedKernelParam2 = true;			
			//cout << "KernelParam2 = " << KernelParam2 << endl;
		}
		// Error Tollerance
		else if (pstricmp(argv[i],"-ErrorTollerance")==0) {			
			float FloatErrorTollerance = 0;
			sscanf (argv[++i], "%f", &FloatErrorTollerance);
			ErrorTollerance = static_cast<double>(FloatErrorTollerance);
			ChangedErrorTollerance = true;			
			//cout << "ErrorTollerance = " << ErrorTollerance << endl;
		}		
		// Verbosity
		else if (pstricmp(argv[i],"-Verbosity")==0) {			
			sscanf (argv[++i], "%d", &Verbosity);
			ChangedVerbosity = true;
			//cout << "Verbosity = " << Verbosity << endl;
		}
		// SaveKernelMatrix
		else if (pstricmp(argv[i],"-SaveKernelMatrix")==0) {			
			SaveKernelMatrix = true;
			ChangedSaveKernelMatrix = true;
			//cout << "Save the Kernel Matrix" << endl;
		}
		// NotSaveKernelMatrix
		else if (pstricmp(argv[i],"-NotSaveKernelMatrix")==0) {			
			SaveKernelMatrix = false;
			ChangedSaveKernelMatrix = true;
			//cout << "Don't save the Kernel Matrix" << endl;
		}
		// StabilizedLearning
		else if (pstricmp(argv[i],"-StabilizedLearning")==0) {			
			StabilizedLearning = true;
			ChangedStabilizedLearning = true;
			//cout << "Stabilized Learning" << endl;
		}
		// NotStabilizedLearning
		else if (pstricmp(argv[i],"-NotStabilizedLearning")==0) {			
			StabilizedLearning = false;
			ChangedStabilizedLearning = true;
			//cout << "Not Stabilized Learning" << endl;
		}
		// Load
		else if (pstricmp(argv[i],"-Load")==0) {		
			NewOnlineSVR = false;
			sscanf (argv[++i], "%s", InputFile);
			//cout << "Input = " << InputFile << endl;
		}
		// Save
		else if (pstricmp(argv[i],"-Save")==0) {			
			sscanf (argv[++i], "%s", OutputFile);
			//cout << "Output = " << OutputFile << endl;
		}
		// Data
		else if (pstricmp(argv[i],"-Data")==0) {			
			sscanf (argv[++i], "%s", DataFile);
			//cout << "Data = " << DataFile << endl;
		}
		// Error
		else {
			cerr << "Error! Parameter Not Valid!" << endl;
			// system("pause");
			exit (1);
		}
	}

	// Build OnlineSVR
	OnlineSVR* SVR = new OnlineSVR();
	if (!NewOnlineSVR)
		SVR->LoadOnlineSVR(InputFile);
	if (ChangedC)
		SVR->SetC(C);
	if (ChangedEpsilon)
		SVR->SetEpsilon(Epsilon);
	if (ChangedKernelType)
		SVR->SetKernelType(KernelType);	
	if (ChangedKernelParam)
		SVR->SetKernelParam(KernelParam);
	if (ChangedKernelParam2)
		SVR->SetKernelParam2(KernelParam2);
	if (ChangedErrorTollerance) {
		SVR->SetAutoErrorTollerance(false);
		SVR->SetErrorTollerance(ErrorTollerance);
	}
	if (ChangedVerbosity)
		SVR->SetVerbosity(Verbosity);
	if (ChangedStabilizedLearning)
		SVR->SetStabilizedLearning(StabilizedLearning);	
	if (ChangedSaveKernelMatrix)
		SVR->SetSaveKernelMatrix(SaveKernelMatrix);
	if (LearningType==OnlineSVR::OPERATION_LEARNING) {
		Matrix<double>* X = NULL;
		Vector<double>* Y = NULL;
		SVR->Import(DataFile, &X, &Y);
		SVR->Train(X,Y);
		if (strlen(OutputFile)==0)
			sprintf(OutputFile,"%s","OnlineSVR1.svr");
		SVR->SaveOnlineSVR(OutputFile);


		SVR->ShowDetails();

		delete X;
		delete Y;
		delete SVR;
	}
	else if (LearningType==OnlineSVR::OPERATION_UNLEARNING) {
		Vector<int>* Indexes = Indexes->Load(DataFile);		
		SVR->Forget(Indexes);
		if (strlen(OutputFile)==0)
			sprintf(OutputFile,"%s","OnlineSVR1.svr");
		SVR->SaveOnlineSVR(OutputFile);
		delete Indexes;
		delete SVR;
	} else if (LearningType==OnlineSVR::OPERATION_STABILIZING) {
		int StabilizationNumber = 0;
		while (!SVR->VerifyKKTConditions()) {
			SVR->Stabilize();
			StabilizationNumber ++;
			if (StabilizationNumber>SVR->GetSamplesTrainedNumber()) {
				cerr << "Error: it's impossible to stabilize the OnlineSVR. Please add or remove some samples." << endl;		
				break;
			}
		}
		if (SVR->GetVerbosity()>=3)
			SVR->ShowDetails();
		if (strlen(OutputFile)==0)
			sprintf(OutputFile,"%s","OnlineSVR1.svr");
		SVR->SaveOnlineSVR(OutputFile);
		delete SVR;
	} else if (LearningType==OnlineSVR::OPERATION_PREDICT) {
		Matrix<double>* X = X->Load(DataFile);
		Vector<double>* Y = SVR->Predict(X);
		if (strlen(OutputFile)==0)
			sprintf(OutputFile,"%s","Predict.txt");
		Y->Save(OutputFile);
		delete X;
		delete Y;
		delete SVR;
	} else if (LearningType==OnlineSVR::OPERATION_MARGIN) {
		Matrix<double>* X = NULL;
		Vector<double>* Y = NULL;		
		SVR->Import(DataFile, &X, &Y);
		Vector<double>* Margin = SVR->Margin(X,Y);
		if (strlen(OutputFile)==0)
			sprintf(OutputFile,"%s","Margin.txt");
		Margin->Save(OutputFile);
		delete X;
		delete Y;
		delete Margin;
		delete SVR;
	} else {
		cerr << "Error! Operation not definited!" << endl;
		system("pause");
		exit (1);
	}
		
	//system("pause");
	return 0;
}


void ShowUsage ()
{
	// Usage
	cout << endl << "OnlineSVR Usage:" << endl;
	cout << "OnlineSVR -<LearningMode> -Data <DataFile> -<OtherOptions>" << endl;
	// Learning Process
	cout << endl << "Learning Params:" << endl;
	cout << "-Train\t\t\t Train new samples" << endl;
	cout << "-Forget\t\t\t Forget trained samples" << endl;
	cout << "-Stabilize\t\t Stabilize the OnlineSVR with new params" << endl;
	cout << "-Predict\t\t Predict the output of new samples" << endl;
	cout << "-Margin\t\t\t Find the error made by OnlineSVR on new samples" << endl;
	// OnlineSVR Parameters
	cout << endl << "OnlineSVR Parameters:" << endl;
	cout << "-Epsilon\t\t Epsilon Param (Default:0.1)" << endl;
	cout << "-C\t\t\t C Param (Default:1)" << endl;
	cout << "-KernelType\t\t Type of Kernel (Default:RBF)" << endl;
	cout << "-KernelParam\t\t Kernel Param" << endl;
	cout << "-KernelParam2\t\t Kernel Param 2" << endl;
	cout << "-Verbosity\t\t Level of Verbosity (0 to 3, Default:1)" << endl;
	cout << "-ErrorTollerance\t Set manually the error tollerance (Default:Epsilon/10)" << endl;
	// Other Parameters
	cout << endl << "Other Parameters:" << endl;
	cout << "-SaveKernelMatrix\t Save kernel matrix (More Space, Less Time)(Default)" << endl;
	cout << "-NotSaveKernelMatrix\t Don't save kernel matrix (Less Space, More Time)" << endl;
	cout << "-StabilizedLearining\t Add stability to svr (Less Errors, More Time)(Default)" << endl;
	cout << "-NotStabilizedLearining\t Don't add stability to svr (More Errors, Less Time)" << endl;
	// I/O Parameters
	cout << endl << "Input/Output Parameters:" << endl;
	cout << "-Load\t\t\t Load an OnlineSVR before computations" << endl;
	cout << "-Save\t\t\t Save OnlineSVR after computations" << endl;
	cout << "-Data\t\t\t Data file that contains samples to process" << endl;
	// Kernel Types
	cout << endl << "Kernels Types: (KP = KernelParam)" << endl;
	cout << "Linear\t\t (Linear)\t\t   K=X1*X2'" << endl;
	cout << "Polynomial \t (Polynomial)\t\t   K=(X1*X2'+1)^KP" << endl;
	cout << "RBF\t\t (Radial Basis Function)   K=exp(-KP*sum(dist(X1X2)^2))" << endl;
	cout << "GaussianRBF\t (Gaussian RBF)\t\t   K=exp(-sum(dist(X1,X2)^2/2*(KP^2))" << endl;
	cout << "ExponentialRBF\t (Exponential RBF)\t   K=exp(-sum(dist(X1,X2)/2*(KP^2))" << endl;
	cout << "MLP\t\t (MultiLayer Perceptron)   K=tanh((X1*X2')*KP+KP2)" << endl << endl;	
}
