#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include "Vector.h"

using namespace std;

namespace onlinesvr
{

	template<class T>
	class Matrix
	{
	public:
		// Attributes
		Vector<Vector<T>*>* Values;

		// Initialization
		Matrix ();
		Matrix (double** X, int Rows, int Cols);
		~Matrix ();
		Matrix<T>* Clone ();
		int GetLengthRows ();
		int GetLengthCols ();

		// Selection Operations
		Vector<T>* GetRowRef (int Index);
		Vector<T>* GetRowCopy (int Index);
		Vector<T>* GetColCopy (int Index);
		T GetValue (int RowIndex, int ColIndex);
		void SetValue (int RowIndex, int ColIndex, T Value);

		// Add/Remove Operations
		void Clear ();
		void AddRowRef (Vector<T>* V);
		void AddRowCopy (Vector<T>* V);
		void AddRowCopy (T* V, int N);
		void AddRowRefAt (Vector<T>* V, int Index);
		void AddRowCopyAt (Vector<T>* V, int Index);
		void AddRowCopyAt (T* V, int N, int Index);
		void AddColCopy (Vector<T>* V);
		void AddColCopy (T* V, int N);
		void AddColCopyAt (Vector<T>* V, int Index);
		void AddColCopyAt (T* V, int N, int Index);
		void RemoveRow (int Index);
		void RemoveCol (int Index);
		Matrix<T>* ExtractRows (int FromRowIndex, int ToRowIndex);
		Matrix<T>* ExtractCols (int FromColIndex, int ToColIndex);

		// Pre-built Matrix
		static Matrix<double>* ZeroMatrix (int RowsNumber, int ColsNumber);
		static Matrix<double>* RandMatrix (int RowsNumber, int ColsNumber);

		// Mathematical Operations
		void SumScalar (T X);
		void ProductScalar (T X);
		void DivideScalar (T X);
		void PowScalar (T X);
		void SumMatrix (Matrix<T>* M);
		void SubtractMatrix (Matrix<T>* M);
		Vector<T>* ProductVector (Vector<T>* V);
		static Vector<T>* ProductVector (Matrix* M, Vector<T>* V);
		static Matrix<T>* ProductVectorVector (Vector<T>* V1, Vector<T>* V2);
		static Matrix<T>* ProductMatrixMatrix (Matrix<T>* M1, Matrix<T>* M2);

		// I/O Operations
		static Matrix<double>* Load(char* Filename);
		void Save (char* Filename);	
		void Print ();
		void Print (char* MatrixName);
		
		// Operators Redefinition	
		Vector<T> operator [] (int Index);

	private:
		// Private Attributes
		int	StepSize;

		// Private Methods
		void Resize ();
		void Resize (int NewRowNumber, int NewColNumber);
	};




		
	////////////////////////////
	// METHODS IMPLEMENTATION //
	////////////////////////////

		
	// INITIALIZATION
	template<class T>
	Matrix<T>::Matrix ()
	{
		this->StepSize = 100;
		this->Values = new Vector<Vector<T>*>();
		this->Values->SetStepSize(this->StepSize);
	}

	template<class T>
	Matrix<T>::Matrix (double** X, int Rows, int Cols)
	{
		this->StepSize = 100;
		this->Values = new Vector<Vector<T>*>(Rows);
		this->Values->SetStepSize(this->StepSize);
		for (int i=0; i<Rows; i++) {
			this->Values->Add(new Vector<T>(X[i],Cols));
		}
	}

	template<class T>
	Matrix<T>::~Matrix ()
	{
		this->Clear();
		delete this->Values;
	}

	template<class T>
	Matrix<T>* Matrix<T>::Clone ()
	{
		int i;
		Matrix<T>* M = new Matrix<T>();
		for (i=0; i<this->GetLengthRows(); i++)
			M->AddRowCopy(this->GetRowRef(i));		
		return M;
	}

	template<class T>
	int Matrix<T>::GetLengthRows ()
	{
		return this->Values->GetLength();
	}

	template<class T>
	int Matrix<T>::GetLengthCols ()
	{
		if (this->Values->GetLength()==0) {
			return 0;
		}
		else {
			return this->Values->Values[0]->GetLength();
		}
	}

		
	// Selection Operations
	template<class T>
	Vector<T>* Matrix<T>::GetRowRef (int Index)
	{
		if (Index>=0 && Index<this->GetLengthRows()) {
			return this->Values->Values[Index];
		}
		else {
			cerr << "Error! It's impossible to get an row from the matrix that doesn't exist." << endl;
			return new Vector<T>();
		}
	}

	template<class T>
	Vector<T>* Matrix<T>::GetRowCopy (int Index)
	{
		if (Index>=0 && Index<this->GetLengthRows()) {
			return this->Values->Values[Index]->Clone();
		}
		else {
			cerr << "Error! It's impossible to get an row from the matrix that doesn't exist." << endl;
			return new Vector<T>();
		}
	}

	template<class T>
	Vector<T>* Matrix<T>::GetColCopy (int Index)
	{
		if (Index>=0 && Index<this->GetLengthCols()) {
			Vector<T>* V = new Vector<T>();
			for (int i=0; i<this->GetLengthRows(); i++) {
				V->Add(this->Values->Values[i]->Values[Index]);
			}
			return V;
		}
		else {
			cerr << "Error! It's impossible to get an row from the matrix that doesn't exist." << endl;
			return new Vector<T>();
		}
	}

	template<class T>
	T Matrix<T>::GetValue (int RowIndex, int ColIndex)
	{
		return this->Values->Values[RowIndex]->Values[ColIndex];
	}

	template<class T>
	void Matrix<T>::SetValue (int RowIndex, int ColIndex, T Value)
	{
		this->Values->Values[RowIndex]->Values[ColIndex] = Value;
	}

	// Add/Remove Operations
	template<class T>
	void Matrix<T>::Clear ()
	{
		for (int i=0; i<this->GetLengthRows(); i++) {
			delete this->Values->Values[i];		
		}	
		this->Values->Clear();
	}

	template<class T>
	void Matrix<T>::AddRowRef (Vector<T>* V)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0) {
			this->Values->Add(V);
			this->Values->Values[this->Values->GetLength()-1]->SetStepSize(this->StepSize);
		}
		else if (this->GetLengthCols() == V->GetLength()) {
			this->Values->Add(V);
			this->Values->Values[this->Values->GetLength()-1]->SetStepSize(this->StepSize);
		}
		else {
			cerr << "Error! It's impossible to add a row of different length." << endl;		
		}
	}

	template<class T>
	void Matrix<T>::AddRowCopy (Vector<T>* V)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0) {
			this->Values->Add(V->Clone());
			this->Values->Values[this->Values->GetLength()-1]->SetStepSize(this->StepSize);
		}
		else if (this->GetLengthCols() == V->GetLength()) {
			this->Values->Add(V->Clone());
			this->Values->Values[this->Values->GetLength()-1]->SetStepSize(this->StepSize);
		}
		else {
			cerr << "Error! It's impossible to add a row of different length." << endl;		
		}
	}
	template<class T>
	void Matrix<T>::AddRowCopy (T* V, int N)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0) {
			Vector<T>* NewV = new Vector<T>(V,N);
			this->Values->Add(NewV);
			this->Values->Values[this->Values->GetLength()-1]->SetStepSize(this->StepSize);
		}
		else if (this->GetLengthCols() == V->Length) {
			Vector<T>* NewV = new Vector<T>(V,N);
			this->Values->Add(NewV);	
			this->Values->Values[this->Values->GetLength()-1]->SetStepSize(this->StepSize);
		}
		else {
			cerr << "Error! It's impossible to add a row of different length." << endl;
		}
	}

	template<class T>
	void Matrix<T>::AddRowRefAt (Vector<T>* V, int Index)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0 && Index==0) {
			this->Values->AddAt(V,Index);
			this->Values->Values[Index]->SetStepSize(this->StepSize);
		}
		else if (this->GetLengthCols()==V->GetLength() && Index>=0 && Index<=this->GetLengthRows()) {
			this->Values->AddAt(V,Index);
			this->Values->Values[Index]->SetStepSize(this->StepSize);
		}
		else {
			cerr << "Error! It's impossible to add a row of different length or in a bad index." << endl;
		}
	}

	template<class T>
	void Matrix<T>::AddRowCopyAt (Vector<T>* V, int Index)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0 && Index==0) {
			this->Values->AddAt(V->Clone(),Index);
			this->Values->Values[Index]->SetStepSize(this->StepSize);
		}
		else if (this->GetLengthCols()==V->GetLength() && Index>=0 && Index<=this->GetLengthRows()) {
			this->Values->AddAt(V->Clone(),Index);
			this->Values->Values[Index]->SetStepSize(this->StepSize);
		}
		else {
			cerr << "Error! It's impossible to add a row of different length or in a bad index." << endl;
		}
	}

	template<class T>
	void Matrix<T>::AddRowCopyAt (T* V, int N, int Index)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0 && Index==0) {
			this->Values->AddAt(new Vector<T>(V,N),Index);
			this->Values->Values[Index]->SetStepSize(this->StepSize);
		}
		else if (this->GetLengthCols()==V->Length && Index>=0 && Index<=this->GetLengthRows()) {
			this->Values->AddAt(new Vector<T>(V,N),Index);
			this->Values->Values[Index]->SetStepSize(this->StepSize);
		}
		else {
			cerr << "Error! It's impossible to add a row of different length or in a bad index." << endl;
		}
	}

	template<class T>
	void Matrix<T>::AddColCopy (Vector<T>* V)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0) {
			for (int i=0; i<V->GetLength(); i++) {
				Vector<T>* V3 = new Vector<T>();
				V3->Add(V->Values[i]);
				V3->SetStepSize(this->StepSize);
				this->Values->Add(V3);
			}
		}
		else if (this->GetLengthRows() == V->GetLength()) {
			for (int i=0; i<V->GetLength(); i++) {
				this->Values->Values[i]->Add(V->Values[i]);
			}
		}
		else {
			cerr << "Error! It's impossible to add a column of different length." << endl;		
		}
	}

	template<class T>
	void Matrix<T>::AddColCopy (T* V, int N)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0) {
			for (int i=0; i<N; i++) {
				Vector<T>* V3 = new Vector<T>();
				V3->Add(V[i]);
				V3->SetStepSize(this->StepSize);
				this->Values->Add(V3);
			}
		}
		else if (this->GetLengthRows() == N) {
			for (int i=0; i<N; i++) {
				this->Values->Values[i]->Add(V[i]);
			}
		}
		else {
			cerr << "Error! It's impossible to add a column of different length." << endl;		
		}
	}

	template<class T>
	void Matrix<T>::AddColCopyAt (Vector<T>* V, int Index)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0 && Index==0) {
			this->AddColCopy(V);
		}
		else if (this->GetLengthRows()==V->GetLength() && Index>=0 && Index<=this->GetLengthRows()) {
			for (int i=0; i<V->GetLength(); i++) {
				this->Values->Values[i]->AddAt(V->Values[i],Index);
			}				
		}
		else {
			cerr << "Error! It's impossible to add a row of different length or in a bad index." << endl;
		}
	}

	template<class T>
	void Matrix<T>::AddColCopyAt (T* V, int N, int Index)
	{
		if (this->GetLengthRows()==0 && this->GetLengthCols()==0 && Index==0) {
			this->AddCol(V,N);
		}
		else if (this->GetLengthRows()==N && Index>=0 && Index<=this->GetLengthRows()) {
			for (int i=0; i<N; i++) {
				this->Values->Values[i]->AddAt(V[i],Index);
			}			
		}
		else {
			cerr << "Error! It's impossible to add a row of different length or in a bad index." << endl;
		}
	}

	template<class T>
	void Matrix<T>::RemoveRow (int Index)
	{
		if (Index>=0 && Index<this->GetLengthRows()) {
			Vector<T>* V = this->Values->Values[Index];
			this->Values->RemoveAt(Index);
			delete V;
		}
		else {
			cerr << "Error! It's impossible to remove an element from the matrix that doesn't exist." << endl;
		}
	}

	template<class T>
	void Matrix<T>::RemoveCol (int Index)
	{
		if (Index>=0 && Index<this->GetLengthCols()) {
			for (int i=0; i<this->GetLengthRows(); i++) {
				this->Values->Values[i]->RemoveAt(Index);
			}
		}
		else {
			cerr << "Error! It's impossible to remove an element from the matrix that doesn't exist." << endl;
		}
	}

	template<class T>
	Matrix<T>* Matrix<T>::ExtractRows (int FromRowIndex, int ToRowIndex)
	{
		if (FromRowIndex>=0 && ToRowIndex<=this->GetLengthRows()-1 && FromRowIndex<=ToRowIndex) {
			Matrix<T>* M = new Matrix<T>();
			for (int i=FromRowIndex; i<=ToRowIndex; i++)
				M->AddRowRef(this->GetRowCopy(i));
			return M;
		}
		else {
			cerr << "Error! It's impossible to extract the rows: invalid indexes" << endl;
			return new Matrix<T>();
		}
	}

	template<class T>
	Matrix<T>* Matrix<T>::ExtractCols (int FromColIndex, int ToColIndex)
	{
		if (FromColIndex>=0 && ToColIndex<=this->GetLengthCols()-1 && FromColIndex<=ToColIndex) {
			Matrix<T>* M = new Matrix<T>();
			for (int i=0; i<this->GetLengthRows(); i++)
				M->AddRowRef(this->GetRowRef(i)->Extract(FromColIndex,ToColIndex));
			return M;
		}
		else {
			cerr << "Error! It's impossible to extract the columns: invalid indexes" << endl;
			return new Matrix<T>();
		}
	}

			
	// Pre-built Matrix
	template<class T>
	Matrix<double>* Matrix<T>::ZeroMatrix (int RowsNumber, int ColsNumber)
	{
		Matrix<double>* M = new Matrix<double>();
		for (int i=0; i<RowsNumber; i++) {
			Vector<double>* V = V->ZeroVector(ColsNumber);
			M->AddRowRef(V);
		}
		return M;
	}

	template<class T>
	Matrix<double>* Matrix<T>::RandMatrix (int RowsNumber, int ColsNumber)
	{
		Matrix<double>* M = new Matrix<double>();
		for (int i=0; i<RowsNumber; i++) {
			Vector<double>* V = V->RandVector(ColsNumber);
			M->AddRowRef(V);
		}
		return M;
	}

		
	// Mathematical Operations
	template<class T>
	void Matrix<T>::SumScalar (T X)
	{
		for (int i=0; i<this->GetLengthRows(); i++) {
			this->Values->Values[i]->SumScalar(X);
		}
	}

	template<class T>
	void Matrix<T>::ProductScalar (T X) 
	{
		for (int i=0; i<this->GetLengthRows(); i++) {
			this->Values->Values[i]->ProductScalar(X);
		}
	}

	template<class T>
	void Matrix<T>::DivideScalar (T X) 
	{
		for (int i=0; i<this->GetLengthRows(); i++) {
			this->Values->Values[i]->DivideScalar(X);
		}
	}

	template<class T>
	void Matrix<T>::PowScalar (T X) 
	{
		for (int i=0; i<this->GetLengthRows(); i++) {
			this->Values->Values[i]->PowScalar(X);
		}
	}

	template<class T>
	void Matrix<T>::SumMatrix (Matrix<T>* M)
	{
		for (int i=0; i<this->GetLengthRows(); i++) {
			this->Values->Values[i]->SumVector(M->Values->Values[i]);
		}
	}

	template<class T>
	void Matrix<T>::SubtractMatrix (Matrix<T>* M)
	{
		for (int i=0; i<this->GetLengthRows(); i++) {
			this->Values->Values[i]->SubtractVector(M->Values->Values[i]);
		}
	}

	template<class T>
	Vector<T>* Matrix<T>::ProductVector (Vector<T>* V)
	{
		if (this->GetLengthCols()==0 || this->GetLengthCols()==V->GetLength()) {
			Vector<T>* V2 = new Vector<T>(this->GetLengthRows());
			for (int i=0; i<this->GetLengthRows(); i++) {
				V2->Add(V->ProductVectorScalar(this->Values->Values[i],V));
			}
			return V2;
		}
		else {
			cerr << "Error! It's impossible to multiply a matrix and a vector with different length." << endl;
			return new Vector<T>();
		}
	}

	template<class T>
	Vector<T>* Matrix<T>::ProductVector (Matrix* M, Vector<T>* V)
	{
		if (M->GetLengthCols()==0 || M->GetLengthCols()==V->GetLength()) {
			Vector<T>* V2 = new Vector<T>(M->GetLengthRows());
			for (int i=0; i<M->GetLengthRows(); i++) {
				V2->Add(V->ProductVectorScalar(M->Values->Values[i],V));
			}
			return V2;
		}
		else {
			cerr << "Error! It's impossible to multiply a matrix and a vector with different length." << endl;
			return new Vector<T>();
		}
	}

	template<class T>
	Matrix<T>* Matrix<T>::ProductVectorVector (Vector<T>* V1, Vector<T>* V2)
	{
		if (V1->GetLength() == V2->GetLength()) {
			Matrix<T>* M = new Matrix();
			for (int i=0; i<V1->GetLength(); i++)
			{
				Vector<T>* V4 = V2->Clone();
				V4->ProductScalar(V1->Values[i]);
				M->AddRowRef(V4);
			}
			return M;
		}
		else {		
			cerr << "Error! It's impossible to multiply two vectors with different length." << endl;
			return new Matrix<T>();
		}
	}

	template<class T>
	Matrix<T>* Matrix<T>::ProductMatrixMatrix (Matrix<T>* M1, Matrix<T>* M2)
	{
		if (M1->GetLengthCols()==M2->GetLengthCols()) {
			Matrix<T>* M3 = new Matrix<T>();
			for (int i=0; i<M1->GetLengthRows(); i++) {
				Vector<T>* V = new Vector<T>(M2->GetLengthRows());
				for (int j=0; j<M2->GetLengthRows(); j++) {
					V->Add(V->ProductVectorScalar(M1->Values->Values[i],M2->Values->Values[i]));
				}
				M3->AddRowRef(V);
			}
			return M3;
		}
		else {
			cerr << "Error! It's impossible to multiply two matrix with not compatiple size." << endl;
			return new Matrix<T>();
		}

	}

		
	// I/O Operations
	template<class T>
	Matrix<double>* Matrix<T>::Load(char* Filename)
	{
		// Open the file
		ifstream File (Filename, ios::in);
		if (!File) {
			cerr << "Error. It's impossible to open the file." << endl;
			return new Matrix<double>();
		}
		Matrix<double>* M = new Matrix<double>();	
		// Save the vector
		try {
			int RowsNumber, ColsNumber;
			double Value;
			File >> RowsNumber >> ColsNumber;
			for (int i=0; i<RowsNumber; i++) {
				Vector<double>* V = new Vector<double>(ColsNumber);
				for (int j=0; j<ColsNumber; j++) {
					File >> Value;
					V->Add(Value);
				}
				M->AddRowRef(V);
			}
		}
		catch (...) {
			cerr << "Error. It's impossible to complete the save." << endl;
		}
		// Close the file
		File.close();

		return M;
	}

	template<class T>
	void Matrix<T>::Save (char* Filename)
	{
		// Open the file
		ofstream File (Filename, ios::out);
		if (!File) {
			cerr << "Error. It's impossible to create the file." << endl;
			return;
		}
		File.precision(30);
		// Save the matrix
		try {				
			File << this->GetLengthRows() << " " << this->GetLengthCols() << endl;
			for (int i=0; i<this->GetLengthRows(); i++) {
				for (int j=0; j<this->GetLengthCols(); j++) 
					File << this->Values->Values[i]->Values[j] << " ";
				File << endl;
			}
		}
		catch (...) {
			cerr << "Error. It's impossible to complete the save." << endl;
		}
		// Close the file
		File.close();
	}

	template<class T>
	void Matrix<T>::Print ()
	{
		for (int i=0; i<this->GetLengthRows(); i++) {
			this->Values->Values[i]->Print();
		}
	}

	template<class T>
	void Matrix<T>::Print (char* MatrixName)
	{
		cout << MatrixName << endl;
		this->Print();
	}


	// Operators Redefinition
	template<class T>
	Vector<T> Matrix<T>::operator [] (int Index)
	{
		return (*this->Values->Values[Index]);
	}
	
}

#endif
