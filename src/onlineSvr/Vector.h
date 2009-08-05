#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
 
using namespace std;

namespace onlinesvr
{

	#ifndef MATH_UTILS
		#define MATH_UTILS
		#define INF 9.9e99
		
		template <class T>
		T ABS (T X) { if (X>=0) return X; else return -X; }

		template <class T>
		int SIGN (T X) { if (X>=0) return 1; else return -1; }
	#endif


	template<class T>
	class Vector
	{
	public:
		// Attributes
		T* Values;

		// Initialization
		Vector ();	
		Vector (T* X, int N);
		Vector (int Length);
		~Vector ();
		Vector<T>* Clone();
		int GetLength ();
		int GetStepSize ();
		void SetStepSize (int X);
		T GetValue (int Index);
		void SetValue (int Index, T Value);

		// Add/Remove Operations
		void Clear ();
		void Add (T X);
		void AddAt (T X, int Index);
		void RemoveAt (int Index);
		Vector<T>* Extract (int FromIndex, int ToIndex);
		
		// Pre-built Vectors
		static Vector<double>* ZeroVector (int Length);
		static Vector<double>* RandVector (int Length);
		static Vector<T>* GetSequence(T Start, T Step, T End);

		// Mathematical Operations
		void SumScalar (T X);	
		void ProductScalar (T X);
		void DivideScalar (T X);
		void PowScalar (T X);
		void SumVector (Vector<T>* V);	
		static Vector<T>* SumVector (Vector<T>* V1, Vector<T>* V2);	
		void SubtractVector (Vector<T>* V);
		static Vector<T>* SubtractVector (Vector<T>* V1, Vector<T>* V2);	
		void ProductVector (Vector<T>* V);
		static Vector<T>* ProductVector (Vector<T>* V1, Vector<T>* V2);
		T ProductVectorScalar (Vector<T>* V);
		static T ProductVectorScalar (Vector<T>* V1, Vector<T>* V2);
		T Sum();
		T AbsSum();

		// Comparison Operations
		T Min();
		void Min(T* MinValue, int*MinIndex);	
		T MinAbs();
		void MinAbs(T* MinValue, int*MinIndex);
		T Max();
		void Max(T* MaxValue, int*MaxIndex);
		T MaxAbs();
		void MaxAbs(T* MaxValue, int*MaxIndex);
		T Mean();
		T MeanAbs();	

		// Sorting Operations
		void Sort();
		void RemoveDuplicates();
		int Find(T X);

		// I/O Operations
		static Vector<T>* Load(char* Filename);
		void Save (char* Filename);
		void Print ();
		void Print (char* VectorName);

		// Operators Redefinition	
		T operator [] (int Index);

	private:
		// Private Attributes	
		int	Length;
		int	MaxLength;
		int	StepSize;

		// Private Methods
		void Resize ();
		void Resize (int NewSize);
	};




		
	////////////////////////////
	// METHODS IMPLEMENTATION //
	////////////////////////////

		
	// INITIALIZATION
	template<class T>
	Vector<T>::Vector ()
	{
		this->Length = 0;
		this->MaxLength = 0;		
		this->StepSize = 100;
		this->Values = NULL;
	}

	template<class T>
	Vector<T>::Vector (T* X, int N)
	{
		this->MaxLength = 0;
		this->StepSize = 100;
		this->Resize((int(N/this->StepSize)+1)*this->StepSize);
		for (int i=0; i<N; i++) {
			this->Values[i] = X[i];
		}
		this->Length = N;	
	}

	template<class T>
	Vector<T>::Vector (int Length)
	{
		this->Length = 0;
		this->MaxLength = 0;		
		this->StepSize = Length + 10;
		this->Values = NULL;
	}

	template<class T>
	Vector<T>::~Vector()
	{
		this->Clear();
	}

	template<class T>
	Vector<T>* Vector<T>::Clone()
	{
		return new Vector<T>(this->Values,this->Length);
	}

	template<class T>
	int Vector<T>::GetLength ()
	{
		return this->Length;
	}

	template<class T>
	int Vector<T>::GetStepSize ()
	{
		return this->StepSize;
	}

	template<class T>
	void Vector<T>::SetStepSize (int X)
	{
		this->StepSize = X;
	}

	template<class T>
	void Vector<T>::Resize ()
	{
		this->Resize(this->MaxLength+this->StepSize);
	}

	template<class T>
	void Vector<T>::Resize (int NewSize)
	{
		if (this->MaxLength==0) {
			this->Values = new T[NewSize];
		}
		else {
			T* NewValues = new T[NewSize];
			for (int i=0; i<this->GetLength(); i++)
				NewValues[i] = this->Values[i];
			delete[] this->Values;
			this->Values = NewValues;
		}

		this->MaxLength = NewSize;
	}

	template<class T>
	T Vector<T>::GetValue (int Index)
	{
		return this->Values[Index];
	}

	template<class T>
	void Vector<T>::SetValue (int Index, T Value)
	{
		this->Values[Index] = Value;
	}
	
	// Add/Remove Operations
	template<class T>
	void Vector<T>::Clear ()
	{
		this->Length = 0;
		this->MaxLength = 0;
		delete[] this->Values;
		this->Values = NULL;
	}

	template<class T>
	void Vector<T>::Add(T X)
	{	
		if (this->Length==this->MaxLength)
			this->Resize();
		this->Values[this->Length++] = X;
	}

	template<class T>
	void Vector<T>::AddAt (T X, int Index)
	{
		if (Index>=0 && Index<=this->Length) {
			if (this->Length==this->MaxLength) {
				this->Resize();
			}
			for (int i=this->Length-1; i>=Index; i--) {
				this->Values[i+1] = this->Values[i];
			}
			this->Values[Index] = X;
			this->Length ++;
		}
		else {
			cerr << "Error! It's impossible to add an element in an invalid index." << endl;
		}
	}

	template<class T>
	void Vector<T>::RemoveAt (int Index)
	{
		if (Index>=0 && Index<this->Length) {
			for (int i=Index; i<this->Length-1; i++) {
				this->Values[i] = this->Values[i+1];
			}
			this->Length --;
		}
		else {
			cerr << "Error! It's impossible to remove an element from the vector that doesn't exist." << endl;
		}
	}

	template<class T>
	Vector<T>* Vector<T>::Extract (int FromIndex, int ToIndex)
	{
		if (FromIndex>=0 && ToIndex<=this->Length-1 && FromIndex<=ToIndex) {
			Vector<T>* V = new Vector<T>(ToIndex-FromIndex+1);
			for (int i=FromIndex; i<=ToIndex; i++)
				V->Add(this->Values[i]);
			return V;
		}
		else {
			cerr << "Error! It's impossible to extract the vector: invalid indexes" << endl;
			return new Vector<T>();
		}
	}

		
	// Pre-built Vectors
	template<class T>
	Vector<double>* Vector<T>::ZeroVector (int Length)
	{
		Vector<double>* V = new Vector<double>(Length);
		for (int i=0; i<Length; i++)
			V->Add(0);
		return V;
	}

	template<class T>
	Vector<double>* Vector<T>::RandVector (int Length)
	{	
		Vector<double>* V = new Vector<double>(Length);
		for (int i=0; i<Length; i++)
			V->Add(static_cast<double>(rand())/static_cast<double>(RAND_MAX ));
			
		return V;
	}

	template<class T>
	Vector<T>* Vector<T>::GetSequence(T Start, T Step, T End)
	{
		Vector<T>* V = new Vector<T>();
		if (Start<End) {
			for (T i=Start; i<=End; i+=Step) {
				V->Add(i);
			}
		}
		else {
			for (T i=Start; i>=End; i-=Step) {
				V->Add(i);
			}
		}
		return V;
	}


	// Mathematical Operations
	template<class T>
	void Vector<T>::SumScalar (T X)
	{
		for (int i=0; i<this->GetLength(); i++) {
			this->Values[i] += X;
		}
	}

	template<class T>
	void Vector<T>::ProductScalar (T X)
	{
		for (int i=0; i<this->GetLength(); i++) {
			this->Values[i] *= X;
		}
	}

	template<class T>
	void Vector<T>::DivideScalar (T X)
	{
		if (X != 0) {
			for (int i=0; i<this->GetLength(); i++) {
				this->Values[i] /= X;
			}
		} 
		else {
			for (int i=0; i<this->GetLength(); i++) {
				this->Values[i] = SIGN(this->Values[i]) * INF;
			}		
		}
	}

	template<class T>
	void Vector<T>::PowScalar (T X)
	{
		for (int i=0; i<this->GetLength(); i++) {
			this->Values[i] = pow(this->Values[i], X);
		}
	}

	template<class T>
	void Vector<T>::SumVector (Vector<T>* V)
	{
		if (this->GetLength() == V->GetLength()) {
			for (int i=0; i<this->GetLength(); i++) {
				this->Values[i] += V->Values[i];
			}
		}
		else {
			cerr << "Error! It's impossible to sum two vectors with different length." << endl;
		}
	}

	template<class T>
	Vector<T>* Vector<T>::SumVector (Vector<T>* V1, Vector<T>* V2) {
		if (V1->GetLength() == V2->GetLength()) {
			Vector *V3 = new Vector(V1->Values, V1->GetLength());
			for (int i=0; i<V1->GetLength(); i++) {
				V3->Values[i] += V2->Values[i];
			}
			return V3;
		}
		else {
			cerr << "Error! It's impossible to sum two vectors with different length." << endl;
			return new Vector<T>();
		}
	}

	template<class T>
	void Vector<T>::SubtractVector (Vector<T>* V)
	{
		if (this->GetLength() == V->GetLength()) {
			for (int i=0; i<this->GetLength(); i++) {
				this->Values[i] -= V->Values[i];
			}
		}
		else {
			cerr << "Error! It's impossible to sum two vectors with different length." << endl;
		}
	}

	template<class T>
	Vector<T>* Vector<T>::SubtractVector (Vector<T>* V1, Vector<T>* V2) {
		if (V1->GetLength() == V2->GetLength()) {
			Vector *V3 = V1->Clone();
			for (int i=0; i<V1->GetLength(); i++) {
				V3->Values[i] -= V2->Values[i];
			}
			return V3;
		}
		else {
			cerr << "Error! It's impossible to subtract two vectors with different length." << endl;
			return new Vector<T>();
		}
	}

	template<class T>
	void Vector<T>::ProductVector (Vector* V)
	{
		if (this->GetLength() == V->GetLength()) {
			for (int i=0; i<this->GetLength(); i++) {
				this->Values[i] *= V->Values[i];
			}
		}
		else {
			cerr << "Error! It's impossible to sum two vectors with different length." << endl;
		}
	}

	template<class T>
	Vector<T>* Vector<T>::ProductVector (Vector<T>* V1, Vector<T>* V2)
	{	
		if (V1->GetLength() == V2->GetLength()) {
			Vector *V3 = new Vector(V1->Values, V1->GetLength());
			for (int i=0; i<V1->GetLength(); i++) {
				V3->Values[i] *= V2->Values[i];
			}
			return V3;
		}
		else {
			cerr << "Error! It's impossible to sum two vectors with different length." << endl;
			return new Vector<T>();
		}
	}

	template<class T>
	T Vector<T>::ProductVectorScalar (Vector<T>* V)
	{	
		if (this->GetLength() == V->GetLength()) {
			T Product = 0;
			for (int i=0; i<this->GetLength(); i++) {
				Product += this->Values[i] * V->Values[i];			
			}
			return Product;
		}	
		else {
			cerr << "Error! It's impossible to sum two vectors with different length." << endl;		
			T Product;
			return Product;
		}
	}

	template<class T>
	T Vector<T>::ProductVectorScalar (Vector<T>* V1, Vector<T>* V2)
	{	
		if (V1->GetLength() == V2->GetLength()) {
			T Product = 0;
			for (int i=0; i<V1->GetLength(); i++) {
				Product += V1->Values[i] * V2->Values[i];
			}
			return Product;
		}
		else {
			cerr << "Error! It's impossible to multiply two vectors with different length." << endl;
			T Product;
			return Product;
		}
	}

	template<class T>
	T Vector<T>::Sum()
	{
		T X = 0;
		for (int i=0; i<this->Length; i++) {
			X += this->Values[i];
		}
		return X;
	}
	template<class T>

	T Vector<T>::AbsSum()
	{
		T X = 0;
		for (int i=0; i<this->Length; i++) {
			X += ABS(this->Values[i]);
		}
		return X;
	}

		
	// Comparison Operations
	template<class T>
	T Vector<T>::Min()
	{
		if (this->Length>0) {
			T MinValue = this->Values[0];
			for (int i=1; i<this->Length; i++) {
				if (this->Values[i]<MinValue) {
					MinValue = this->Values[i];
				}
			}
			return MinValue;
		}
		else {
			T MinValue;
			return MinValue;
		}
	}

	template<class T>
	void Vector<T>::Min(T* MinValue, int* MinIndex)
	{
		if (this->Length>0) {
			(*MinValue) = this->Values[0];
			(*MinIndex) = 0;
			for (int i=1; i<this->Length; i++) {
				if (this->Values[i]<(*MinValue)) {
					(*MinValue) = this->Values[i];
					(*MinIndex) = i;
				}
			}
		}
		else {
			(*MinValue) = -1;
			(*MinIndex) = -1;
		}
	}

	template<class T>
	T Vector<T>::MinAbs()
	{
		if (this->Length>0) {
			T MinValue = ABS(this->Values[0]);
			for (int i=1; i<this->Length; i++) {
				if (ABS(this->Values[i])<MinValue) {
					MinValue = ABS(this->Values[i]);
				}
			}
			return MinValue;
		}
		else {
			T MinValue;
			return MinValue;
		}
	}

	template<class T>
	void Vector<T>::MinAbs(T* MinValue, int* MinIndex)
	{
		if (this->Length>0) {
			(*MinValue) = ABS(this->Values[0]);
			(*MinIndex) = 0;
			for (int i=1; i<this->Length; i++) {
				if (ABS(this->Values[i])<(*MinValue)) {
					(*MinValue) = ABS(this->Values[i]);
					(*MinIndex) = i;
				}
			}
		}
		else {
			(*MinValue) = -1;
			(*MinIndex) = -1;
		}
	}

	template<class T>
	T Vector<T>::Max()
	{
		if (this->Length>0) {
			T MaxValue = this->Values[0];
			for (int i=1; i<this->Length; i++) {
				if (this->Values[i]>MaxValue) {
					MaxValue = this->Values[i];
				}
			}
			return MaxValue;
		}
		else {
			T MaxValue;
			return MaxValue;
		}
	}

	template<class T>
	void Vector<T>::Max(T* MaxValue, int* MaxIndex)
	{
		if (this->Length>0) {
			(*MaxValue) = this->Values[0];
			(*MaxIndex) = 0;
			for (int i=1; i<this->Length; i++) {
				if (this->Values[i]>(*MaxValue)) {
					(*MaxValue) = this->Values[i];
					(*MaxIndex) = i;
				}
			}
		}
		else {
			(*MaxValue) = -1;
			(*MaxIndex) = -1;
		}
	}

	template<class T>
	T Vector<T>::MaxAbs()
	{
		if (this->Length>0) {
			T MaxValue = ABS(this->Values[0]);
			for (int i=1; i<this->Length; i++) {
				if (ABS(this->Values[i])>MaxValue) {
					MaxValue = ABS(this->Values[i]);
				}
			}
			return MaxValue;
		}
		else {
			T MaxValue;
			return MaxValue;
		}
	}

	template<class T>
	void Vector<T>::MaxAbs(T* MaxValue, int* MaxIndex)
	{
		if (this->Length>0) {
			(*MaxValue) = ABS(this->Values[0]);
			(*MaxIndex) = 0;
			for (int i=1; i<this->Length; i++) {
				if (ABS(this->Values[i])>(*MaxValue)) {
					(*MaxValue) = ABS(this->Values[i]);
					(*MaxIndex) = i;
				}
			}
		}
		else {
			(*MaxValue) = -1;
			(*MaxIndex) = -1;
		}
	}

	template<class T>
	T Vector<T>::Mean()
	{
		if (this->Length>0) {		
			T MeanValue = 0;
			for (int i=1; i<this->Length; i++) {			
				MeanValue += this->Values[i];
			}
			return MeanValue/this->Length;
		}
		else {
			T MeanValue;
			return MeanValue;
		}
	}

	template<class T>
	T Vector<T>::MeanAbs()
	{
		if (this->Length>0) {		
			T MeanValue = 0;
			for (int i=1; i<this->Length; i++) {			
				MeanValue += ABS(this->Values[i]);
			}
			return MeanValue/this->Length;
		}
		else {
			T MeanValue;
			return MeanValue;
		}
	}

		
	// Sorting Operations
	template<class T>
	void Vector<T>::Sort()
	{	
		sort(&this->Values[0], &this->Values[this->Length-1]);
	}

	template<class T>
	void Vector<T>::RemoveDuplicates()
	{
		for (int i=this->Length-1; i>0; i--) {
			for (int j=0; j<i; j++) {
				if (this->Values[i]==this->Values[j]) {
					this->RemoveAt(i);				
					break;
				}
			}
		}
	}

	template<class T>
	int Vector<T>::Find(T X)
	{
		for (int i=0; i<this->Length; i++) {
			if (this->Values[i] == X) {
				return i;
			}
		}
		return -1;
	}

		
	// I/O Operations
	template<class T>
	Vector<T>* Vector<T>::Load(char* Filename)
	{
		// Open the file
		ifstream File (Filename, ios::in);
		if (!File) {
			cerr << "Error. It's impossible to open the file." << endl;
			return new Vector<T>();
		}
		Vector<T>* V;
		// Load the vector
		try {			
			T Value;
			V = new Vector<T>();
			while (!File.eof()) {
				File >> Value;
				V->Add(Value);
			}
		}
		catch (...) {
			//cerr << "Error. It's impossible to complete the save." << endl;
		}
		// Close the file
		File.close();

		return V;
	}

	template<class T>
	void Vector<T>::Save (char* Filename)
	{
		// Open the file
		ofstream File (Filename, ios::out);
		if (!File) {
			cerr << "Error. It's impossible to create the file." << endl;
			return;
		}
		File.precision(30);
		// Save the vector
		try {				
			for (int i=0; i<this->Length; i++)
				File << this->Values[i] << " ";
			File << endl;
		}
		catch (...) {
			cerr << "Error. It's impossible to complete the save." << endl;
		}
		// Close the file
		File.close();
	}

	template<class T>
	void Vector<T>::Print ()
	{
		for (int i=0; i<this->Length; i++) {
			cout << this->Values[i] << "\t";
		}
		cout << endl;
	}

	template<class T>
	void Vector<T>::Print(char *VectorName)
	{
		cout << VectorName << " \t";
		Print();
	}

		
	// Operators Redefinition
	template<class T>
	T Vector<T>::operator [] (int Index)
	{
		return this->Values[Index];
	}

}

#endif
