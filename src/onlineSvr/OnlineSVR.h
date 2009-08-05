#ifndef ONLINE_SVR_H
#define ONLINE_SVR_H

#include "Vector.h"
#include "Matrix.h"

using namespace std;

namespace onlinesvr
{

	class OnlineSVR
	{

	public:

		// Constants
		enum {
    		OPERATION_LEARNING=200,
			OPERATION_UNLEARNING = 201,
			OPERATION_STABILIZING = 202,
			OPERATION_PREDICT = 203,
			OPERATION_MARGIN = 204,

			NO_SET = 10,
			SUPPORT_SET = 11,
			ERROR_SET = 12,
			REMAINING_SET = 13,

			KERNEL_LINEAR = 100,
			KERNEL_POLYNOMIAL = 101,
			KERNEL_RBF = 102,
			KERNEL_RBF_GAUSSIAN = 103,
			KERNEL_RBF_EXPONENTIAL = 104,
			KERNEL_MLP = 105,
		
			VERBOSITY_NO_MESSAGES = 0,
			VERBOSITY_NORMAL = 1,
			VERBOSITY_DETAILS = 2,
			VERBOSITY_DEBUG = 3
	   };

		// Initialization
		OnlineSVR();
		~OnlineSVR();
		void Clear();
		OnlineSVR* Clone();

		// Attributes Operations
		double GetC ();
		void SetC (double C);
		double GetEpsilon ();
		void SetEpsilon (double Epsilon);
		int GetKernelType ();
		void SetKernelType (int);
		double GetKernelParam ();
		void SetKernelParam (double KernelParam);
		double GetKernelParam2 ();
		void SetKernelParam2 (double KernelParam);
		bool GetAutoErrorTollerance ();
		void SetAutoErrorTollerance (bool AutoErrorTollerance);
		double GetErrorTollerance ();
		void SetErrorTollerance (double ErrorTollerance);
		int GetVerbosity ();
		void SetVerbosity (int Verbosity);
		bool GetStabilizedLearning ();
		void SetStabilizedLearning (bool StabilizedLearning);
		bool GetSaveKernelMatrix ();
		void SetSaveKernelMatrix (bool SaveKernelMatrix);
		int GetSamplesTrainedNumber ();
		int GetSupportSetElementsNumber ();
		int GetErrorSetElementsNumber ();
		int GetRemainingSetElementsNumber ();	
		Vector<int>* GetSupportSetIndexes();
		Vector<int>* GetErrorSetIndexes();
		Vector<int>* GetRemainingSetIndexes();
		
		// Learning Operations
		int Train (Matrix<double>* X, Vector<double>* Y);
		int Train (double**X, double *Y, int ElementsNumber, int ElementsSize);	
		int Train (Vector<double>* X, double Y);
		int Forget (Vector<int>* Indexes);
		int Forget (int* Indexes, int N);
		int Forget (int Index);
		int Stabilize ();
		void SelfLearning (Matrix<double>* TrainingSetX, Vector<double>* TrainingSetY, Matrix<double>* ValidationSetX, Vector<double>* ValidationSetY, double ErrorTollerance);	
		static void CrossValidation (Matrix<double>* TrainingSetX, Vector<double>* TrainingSetY, Vector<double>* EpsilonList, Vector<double>* CList, Vector<double>* KernelParamList, int SetNumber, char* ResultsFileName);
		static double CrossValidation (Vector<Matrix<double>*>* SetX, Vector<Vector<double>*>* SetY, double Epsilon, double C, double KernelParam);
		static void LeaveOneOut (Matrix<double>* TrainingSetX, Vector<double>* TrainingSetY, Vector<double>* EpsilonList, Vector<double>* CList, Vector<double>* KernelParamList, char* ResultsFileName);
		static double LeaveOneOut (Matrix<double>* SetX, Vector<double>* SetY, double Epsilon, double C, double KernelParam);

		// Predict/Margin Operations
		double Predict (Vector<double>* X);
		double Predict (double* X, int ElementsSize);
		Vector<double>* Predict (Matrix<double>* X);
		double* Predict (double** X, int ElementsNumber, int ElementsSize);	
		double Margin (Vector<double>* X, double Y);
		double Margin (double* X, double Y, int ElementsSize);
		Vector<double>* Margin (Matrix<double>* X, Vector<double>* Y);
		double* Margin (double** X, double* Y, int ElementsNumber, int ElementsSize);
		
		// Control Operations
		bool VerifyKKTConditions ();
		void FindError(Matrix<double>* ValidationSetX, Vector<double>* ValidationSetY, double* MinError, double* MeanError, double* MaxError);

		// I/O Operations
		void ShowInfo ();
		void ShowDetails ();
		void LoadOnlineSVR(char* Filename);
		void SaveOnlineSVR(char* Filename);
		static void Import(char* Filename, Matrix<double>** X, Vector<double>** Y);
		static void Import(char* Filename, Matrix<double>** AngularPositions, Matrix<double>** MotorCurrents, Matrix<double>** AppliedVoltages);
		
	private:

		// Parameter Attributes
		double C;
		double Epsilon;
		int KernelType;
		double KernelParam;
		double KernelParam2;
		bool AutoErrorTollerance;
		double ErrorTollerance;
		int Verbosity;
		int SamplesTrainedNumber;
		bool StabilizedLearning;
		bool SaveKernelMatrix;

		// Training Set Attributes
		Matrix<double>* X;
		Vector<double>* Y;
		Vector<double>* Weights;
		double Bias;

		// Work Set Attributes
		Vector<int>* SupportSetIndexes;
		Vector<int>* ErrorSetIndexes;
		Vector<int>* RemainingSetIndexes;	
		Matrix<double>* R;
		Matrix<double>* KernelMatrix;

		// Private Learning Operations
		int Learn (Vector<double>* X, double Y);
		int Unlearn (int Index);		
		
		// Kernel Operations
		double Kernel (Vector<double>* V1, Vector<double>* V2);
		Matrix<double>* Q (Vector<int>* V1, Vector<int>* V2);
		Matrix<double>* Q (Vector<int>* V);
		Vector<double>* Q (Vector<int>* V, int Index);
		Vector<double>* Q (int Index);
		double Q (int Index1, int Index2);	
		
		// Matrix R Operations
		void AddSampleToR (int SampleIndex, int SampleOldSet, Vector<double>* Beta, Vector<double>* Gamma);
		void RemoveSampleFromR (int SampleIndex);
		Vector<double>* FindBeta (int SampleIndex);
		Vector<double>* FindGamma (Vector<double>* Beta, int SampleIndex);
		double FindGammaSample (Vector<double>* Beta, int SampleIndex);

		// KernelMatrix Operations
		void AddSampleToKernelMatrix (Vector<double>* X);
		void RemoveSampleFromKernelMatrix (int SampleIndex);
		void BuildKernelMatrix ();
		double Predict (int Index);

		// Variation Operations
		double FindVariationLc1 (Vector<double>* H, Vector<double>* Gamma, int SampleIndex, int Direction);
		double FindVariationLc2 (int SampleIndex, int Direction);
		double FindVariationLc (int SampleIndex);
		Vector<double>* FindVariationLs (Vector<double>* H, Vector<double>* Beta, int Direction);
		Vector<double>* FindVariationLe (Vector<double>* H, Vector<double>* Gamma, int Direction);
		Vector<double>* FindVariationLr (Vector<double>* H, Vector<double>* Gamma, int Direction);
		void FindLearningMinVariation (Vector<double>* H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double* MinVariation, int* MinIndex, int* Flag);
		void FindUnlearningMinVariation (Vector<double>* H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double* MinVariation, int* MinIndex, int* Flag);
		
		// Set Operations
		void UpdateWeightsAndBias (Vector<double>** H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double MinVariation);
		void AddSampleToRemainingSet (int SampleIndex);
		void AddSampleToSupportSet (Vector<double>** H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double MinVariation);
		void AddSampleToErrorSet (int SampleIndex, double MinVariation);
		void MoveSampleFromSupportSetToErrorRemainingSet (int MinIndex, double MinVariation);
		void MoveSampleFromErrorSetToSupportSet (Vector<double>** H, Vector<double>* Beta, Vector<double>* Gamma, int MinIndex, double MinVariation);
		void MoveSampleFromRemainingSetToSupportSet (Vector<double>** H, Vector<double>* Beta, Vector<double>* Gamma, int MinIndex, double MinVariation);
		void RemoveSampleFromSupportSet (int SampleSetIndex);
		void RemoveSampleFromErrorSet (int SampleSetIndex);
		void RemoveSampleFromRemainingSet (int SampleSetIndex);
		void RemoveSample (int SampleIndex);
		
		// Other Control Operations
		bool VerifyKKTConditions (Vector<double>* H);
		bool VerifyKKTConditions (Vector<double>* H, int* SampleIndex, int* SetName, int* SampleSetIndex);
		bool VerifyKKTConditions (int SampleIndex);
		static bool IsEquals (double Value1, double Value2, double Error);
		static bool IsLesser (double Value1, double Value2, double Error);
		static bool IsBigger (double Value1, double Value2, double Error);
		static bool IsContained (double Value, double From, double To, double Error);

		// Other I/O Operations
		void ShowMessage (char* Message, int VerbosityLevel);
		void ShowMessage (const char* Message, int VerbosityLevel);
		void ShowDetails (Vector<double> *H, int SampleIndex);
		void ShowVariations (Vector<double>* H, Vector<double>* Beta, Vector<double>* Gamma, int SampleIndex, double Lc1, double Lc2, Vector<double> *Ls, Vector<double> *Le, Vector<double> *Lr, int OperationType);
		void ShowLine(char *SetName, int SetIndex, int SampleIndex, double Info1, double Info2);
		void ShowLine(char *SetName, int SetIndex, int SampleIndex, double Info1, double Info2, double Info3);
		static char* TimeToString (long Time);

	};

}

#endif
