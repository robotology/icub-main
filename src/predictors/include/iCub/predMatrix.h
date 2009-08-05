/*****************************************************************************************
*		
* 
*	Generic Matrix Manipulation. When have time, maybe use YARPMath
*
*
*	Paulo Carreiras, vislab, isr/ist, June 2008
******************************************************************************************/

#ifndef PREDMATRIX_H
#define PREDMATRIX_H

#include <iostream>
#include <fstream>

//#include <math.h>
//#include <malloc.h>
//#include <string.h>

#include <stdlib.h>
#include <cmath>

#define DEFLINES 5
#define DEFCOLUMNS 5
#define DEBUG_

using namespace std;

template<class gType>
class predMatrix{
	
public:
	int nLines;			// numero de linhas
	int nColumns;		// numero de colunas
	gType *pData;		// dados
	void minv(double *mat, int n, double *d, double *invmat);

public:
	//construtores
	predMatrix(void);											//construtor default
	predMatrix(int _nLines, int _nColumns);					//construtor parametrizado
	predMatrix(int _nLines, int _nColumns, gType *_data);		//construtor parametrizado com valores
	predMatrix(int _nLines, int _nColumns, gType value);		//construtor parametrizado com valor da diagonal
	predMatrix(const predMatrix &orig);							//construtor cópia

	//destructor
	~predMatrix(void);										//destructor

	//overload de operadores
	predMatrix operator +(const predMatrix &orig);				//overload do operador "+"
	predMatrix operator -(const predMatrix &orig);				//overload do operador "-"
	predMatrix& operator =(const predMatrix &orig);				//overload do operador "="
	predMatrix operator *(const predMatrix &orig);				//overload do operador "*" (matriz por matriz)
	predMatrix operator *(const gType value);					//overload do operador "*" (matriz por escalar)
	predMatrix operator /(const predMatrix &orig);				//overload do operador "/" (matriz por matriz)
	predMatrix operator /(const gType value);					//overload do operador "/" (matriz por escalar)
	predMatrix operator ^(const int value);					//overload do operador "^" (potencia inteira)
	template <class U>
	friend ostream& operator <<(ostream &out, predMatrix &orig); //overload do operador "<<"
	template <class U>
	friend predMatrix operator *(const double value, predMatrix &orig); //overload do operador "*" (escalar por matriz)
	//overload do operador ">>"
	istream& read(istream& is=cin);
	template <class U>
	friend istream& operator>>(istream& is, predMatrix<U> *M1){return M1->read(is);};
	template <class U>
	friend istream& operator>>(istream& is, predMatrix<U> &M1) {return M1.read(is);};

	//Funcoes utilitarias
	predMatrix Transposta(void);								//transpoe a matriz
	gType Determinante(void);       //calcula o determinante de uma matriz
	void IntroduzValor(int nLine, int _nColumn, gType value);//Introduz um valor numa posicao da matriz
	void Inicializa(void);									//inicializa a matriz a zeros
	void Visualiza(void);									//Visualisa a matrix
	template <class U>
	friend void loadMatrices(char *fname, predMatrix * MM, int n);
	template <class U>
	friend void saveMatrices(char *fname, predMatrix * MM, int n);
	int existeNaMatrix(const gType value);		//verifica se existe o valor na matrix
	gType getValue(const int position); //retorna um valor da matrix
	void setValue(const int position,  const gType value); //insere um valor na matrix
	void getMatrixPart(predMatrix &newM, const int partNumber, const int pixelsBorder); //retorna uma submatriz de uma matriz
	void putMatrixPart(const predMatrix &orig, const int partNumber, const int pixelsBorder); //insere uma submatriz numa matriz
	gType media();
	int nLins(){return nLines;};
	int nCols(){return nColumns;};

};

/*****************************************************************************
*****************************************************************************/
//construtor default.
//Cria objecto predMatrix com as dimensões predefinidas e inicializado a zero
template <class gType>
predMatrix<gType>::predMatrix(void){

#ifdef DEBUG
	cout << "---->Chamada do construtor default" << endl;
#endif
	
	nLines = DEFLINES;
	nColumns = DEFCOLUMNS;
	pData = new gType [nLines*nColumns];
	(*this).Inicializa();
}

/*****************************************************************************
*****************************************************************************/
//construtor parametrizado.
//Cria objecto predMatrix com as dimensões especificadas e inicializado a zero
template <class gType>
predMatrix<gType>::predMatrix(int _nLines, int _nColumns){

#ifdef DEBUG
	cout << "---->Chamada do construtor parametrizado" << endl;
#endif
	
	nLines = _nLines;
	nColumns = _nColumns;
	pData = new gType [nLines*nColumns];
	(*this).Inicializa();
}

/*****************************************************************************
*****************************************************************************/
//construtor parametrizado.
//Cria objecto predMatrix com as dimensões especificadas e inicializado com os
//valores contidos no vector dado
template <class gType>
predMatrix<gType>::predMatrix(int _nLines, int _nColumns, gType *_data){ 

#ifdef DEBUG
	cout << "---->Chamada do construtor parametrizado com valores especificados" << endl;
#endif
	
		nLines = _nLines;
		nColumns = _nColumns;
		pData = new gType [nLines*nColumns];
		for(int i = 0; i<nLines*nColumns; i++){
			pData[i] = _data[i];
		}
}

/*****************************************************************************
*****************************************************************************/
//construtor parametrizado.
//Cria objecto predMatrix com as dimensões especificadas e inicializado na 
//diagonal com o valor dado
template <class gType>
predMatrix<gType>::predMatrix(int _nLines, int _nColumns, gType _value){ 

#ifdef DEBUG
	cout << "---->Chamada do construtor parametrizado com valor para a diagonal" << endl;
#endif
	
	nLines = _nLines;
	nColumns = _nColumns;

	if(nLines != nColumns){
		cout<<"AVISO: A matrix nao e diagonal"<<endl;
	}

	pData = new gType [nLines*nColumns];
	(*this).Inicializa();
	for(int i = 0; i<nLines; i++){
		pData[i*(nColumns+1)] = _value;
	}
}

/*****************************************************************************
*****************************************************************************/
//construtor cópia.
template <class gType>
predMatrix<gType>::predMatrix(const predMatrix<gType> &orig){ 

#ifdef DEBUG
	cout << "---->Chamada do construtor copia" << endl;
#endif
	
	nLines = orig.nLines;
	nColumns = orig.nColumns;
	pData = new gType [nLines*nColumns];
	for(int i = 0; i<nLines*nColumns; i++){
		pData[i] = orig.pData[i];
	}
}

/*****************************************************************************
*****************************************************************************/
//Destructor
template <class gType>
predMatrix<gType>::~predMatrix(void){

#ifdef DEBUG
	cout << "---->Chamada do destructor" << endl;
#endif
	
	delete []pData;
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador ">>"
template <class gType>
istream& predMatrix<gType>::read(istream& is)
{
	int i,j,k;
	for (k=0, i=0; i<nLines; i++)
		for (j=0; j<nColumns; j++)
			is >> pData[k++];
	return is;
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "+"
template <class gType>
predMatrix<gType> predMatrix<gType>::operator +(const predMatrix<gType> &orig){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"+\"" << endl;
#endif
	
	predMatrix<gType> temp(*this);
	
	if((nLines==orig.nLines)&&(nColumns==orig.nColumns)){
		temp.nLines=nLines;
		temp.nColumns=nColumns;
		for(int i = 0; i<nLines*nColumns; i++){
			temp.pData[i] = pData[i]+orig.pData[i];
		}
	}else{
		cout<<"ERRO: Operator \"+\": as dimensoes nao coincidem"<<endl;
	}
	return(temp);
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "-"
template <class gType>
predMatrix<gType> predMatrix<gType>::operator -(const predMatrix<gType> &orig){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"-\"" << endl;
#endif
	
	predMatrix<gType> temp(*this);
	
	if((nLines==orig.nLines)&&(nColumns==orig.nColumns)){
		temp.nLines=nLines;
		temp.nColumns=nColumns;
		for(int i = 0; i<nLines*nColumns; i++){
			temp.pData[i] = pData[i]-orig.pData[i];
		}
	}else{
		cout<<"ERRO: Operator \"-\": as dimensoes nao coincidem"<<endl;
	}
	return(temp);
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "="
template <class gType>
predMatrix<gType>& predMatrix<gType>::operator =(const predMatrix<gType> &orig){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"=\"" << endl;
#endif
	
	if((nLines!=orig.nLines)||(nColumns!=orig.nColumns)){
		//cout << "AVISO: As dimensoes nao coincidem. A matriz sera realocada"<<endl;
		delete[] pData;
		nLines=orig.nLines;
		nColumns=orig.nColumns;
		pData = new gType [nLines*nColumns];
	}
	for(int i = 0; i<nLines*nColumns; i++){
		pData[i] = orig.pData[i];
	}
	
	return(*this);
}


/*****************************************************************************
*****************************************************************************/
//Overload do operador "*" (matriz por matriz)
template <class gType>
predMatrix<gType> predMatrix<gType>::operator *(const predMatrix<gType> &orig){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"*\" (matriz por matriz)" << endl;
#endif
	
	predMatrix<gType> temp(nLines, orig.nColumns);
	
	if(nColumns==orig.nLines){
		for(int i = 0; i<nLines; i++){
			for(int j = 0; j<orig.nColumns; j++){
				temp.pData[i*orig.nColumns+j] = 0;
				for( int k = 0; k<nColumns; k++){
					temp.pData[i*orig.nColumns+j] = temp.pData[i*orig.nColumns+j]+
						pData[i*orig.nLines+k]*orig.pData[k*orig.nColumns+j];
				}
			}
		}
	}else{
		cout<<"ERRO: Operator \"*\" (M * M): as dimensoes nao coincidem"<<endl;
	}
	return(temp);
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "*" (matriz por escalar)
template <class gType>
predMatrix<gType> predMatrix<gType>::operator *(const gType value){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"*\" (matriz por escalar)" << endl;
#endif
	
	predMatrix<gType> temp(nLines, nColumns);
	
	for(int i = 0; i<nLines*nColumns; i++){
		temp.pData[i] = pData[i]*value;	
	}

	return(temp);
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "/" (matriz por matriz)
template <class gType>
predMatrix<gType> predMatrix<gType>::operator /(const predMatrix<gType> &orig){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"/\" (matriz por matriz)" << endl;
#endif

	double determinante;
	predMatrix<gType> temp(*this);
	predMatrix<gType> invTemp(orig);

	if((orig.nLines==orig.nColumns)&&(orig.nLines==nColumns)){
		minv(orig.pData, orig.nLines, &determinante,invTemp.pData);
		if(determinante>1e-4||determinante<-1e-4){
			temp =(*this)*invTemp;
			return (temp);
		}else{
			cout<<"ERRO: a matriz do denominador nao e invertivel (determinante = 0)"<<endl;
			return(*this);
		}
	}else{
		cout<<"ERRO: Operator \"/\": as dimensoes nao coincidem"<<endl;
	}
	return(*this);
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "/" (matriz por escalar)
template <class gType>
predMatrix<gType> predMatrix<gType>::operator /(const gType value){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"/\" (matriz por escalar)" << endl;
#endif
	
	predMatrix<gType> temp(nLines, nColumns);
	
	for(int i = 0; i<nLines*nColumns; i++){
		temp.pData[i] = pData[i]/value;	
	}

	return(temp);
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "^" (potencia inteira)
template <class gType>
predMatrix<gType> predMatrix<gType>::operator ^(const int value){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"^\" (potencia inteira)" << endl;
#endif
	
	if(value>1){
		predMatrix<gType> temp =(*this)^(value/2);
		return (value%2) ? temp*temp*(*this) : temp*temp;
	}else{
		return(*this);
	}
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "<<"
template <class gType>
ostream & operator <<(ostream &out, predMatrix<gType> &orig){

#ifdef DEBUG
	out << "---->Chamada do overload do operador \"<<\"" << endl;
#endif
	
	out<<"---------------------"<<endl;
	for(int i = 0; i<orig.nLines; i++){
		out<<"[\t";
		for(int j = 0; j<orig.nColumns; j++){
			out<<orig.pData[orig.nColumns*i+j]<<"\t";
		}
		out<<"]"<<endl;
	}
	out<<"---------------------"<<endl;
	return out;
}

/*****************************************************************************
*****************************************************************************/
//Overload do operador "*" (escalar por matriz)
template <class gType>
predMatrix<gType> operator *(const double value, predMatrix<gType> &orig){

#ifdef DEBUG
	cout << "---->Chamada do overload do operador \"*\" (escalar por matriz)" << endl;
#endif
	
	predMatrix<gType> temp(orig.nLines, orig.nColumns);
	
	for(int i = 0; i<orig.nLines*orig.nColumns; i++){
		temp.pData[i] = value*orig.pData[i];	
	}

	return(temp);
}

/*****************************************************************************
*****************************************************************************/
//Calcula a média de uma matriz
template <class gType>
gType predMatrix<gType>::media(){
	gType res=0;
	for(int i=0;i<nLines*nColumns; i++)
		res+=pData[i];
	return res/(nLines*nColumns);
}


/*****************************************************************************
*****************************************************************************/
//Calcula a transposta da matriz
template <class gType>
predMatrix<gType> predMatrix<gType>::Transposta(void){
	
#ifdef DEBUG
	cout << "---->Chamada da funcao Transposta" << endl;
#endif

	predMatrix<gType> temp(nColumns, nLines);

	for(int i = 0; i<nLines; i++){
		for(int j = 0; j<nColumns; j++){ 
			temp.pData[i+j*nLines] = pData[i*nColumns+j];
		}
	}

	return(temp);
}

/*****************************************************************************
*****************************************************************************/
//Calcula o determinante de uma matriz
template <class gType>
gType predMatrix<gType>::Determinante(void){

#ifdef DEBUG
	cout << "---->Chamada da função determinante)" << endl;
#endif

	double determinante;
	predMatrix<gType> temp(*this);

	minv(pData,nLines,&determinante,temp.pData);

	return (determinante);
}


/*****************************************************************************
*****************************************************************************/
//insere uma submatriz numa matriz
template <class gType>
void predMatrix<gType>::putMatrixPart(const predMatrix<gType> &orig, const int partNumber, const int pixelsBorder){

#ifdef DEBUG
	cout << "---->Chamada da funcao putMatrixPart" << endl;
#endif

	switch(partNumber){
		case 1:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[i*nColumns+k] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 2:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[i*nColumns+(k+orig.nColumns-2*pixelsBorder)] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 3:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[i*nColumns+(k+2*(orig.nColumns-2*pixelsBorder))] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 4:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[i*nColumns+(k+3*(orig.nColumns-2*pixelsBorder))] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 5:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+orig.nLines-2*pixelsBorder)*nColumns+k] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 6:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+orig.nLines-2*pixelsBorder)*nColumns+(k+orig.nColumns-2*pixelsBorder)] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 7:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+orig.nLines-2*pixelsBorder)*nColumns+(k+2*(orig.nColumns-2*pixelsBorder))] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 8:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+orig.nLines-2*pixelsBorder)*nColumns+(k+3*(orig.nColumns-2*pixelsBorder))] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 9:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+2*(orig.nLines-2*pixelsBorder))*nColumns+k] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 10:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+2*(orig.nLines-2*pixelsBorder))*nColumns+(k+orig.nColumns-2*pixelsBorder)] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 11:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+2*(orig.nLines-2*pixelsBorder))*nColumns+(k+2*(orig.nColumns-2*pixelsBorder))] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 12:
			for(int i=0; i<(orig.nLines-2*pixelsBorder); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+2*(orig.nLines-2*pixelsBorder))*nColumns+(k+3*(orig.nColumns-2*pixelsBorder))] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 13:
			for(int i=0; i<(orig.nLines-2*pixelsBorder+2); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+3*(orig.nLines-2*pixelsBorder))*nColumns+k] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 14:
			for(int i=0; i<(orig.nLines-2*pixelsBorder+2); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+3*(orig.nLines-2*pixelsBorder))*nColumns+(k+orig.nColumns-2*pixelsBorder)] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 15:
			for(int i=0; i<(orig.nLines-2*pixelsBorder+2); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+3*(orig.nLines-2*pixelsBorder))*nColumns+(k+2*(orig.nColumns-2*pixelsBorder))] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		case 16:
			for(int i=0; i<(orig.nLines-2*pixelsBorder+2); i++){
				for(int k=0; k<(orig.nColumns-2*pixelsBorder); k++){
					pData[(i+3*(orig.nLines-2*pixelsBorder))*nColumns+(k+3*(orig.nColumns-2*pixelsBorder))] = orig.pData[(i+pixelsBorder)*orig.nColumns+(k+pixelsBorder)];
				}
			}
			break;
		default:
			
			break;
	}
	
	
}



/*****************************************************************************
*****************************************************************************/
//retorna uma submatriz de uma matriz
template <class gType>
void predMatrix<gType>::getMatrixPart(predMatrix<gType> &newM, const int partNumber, const int pixelsBorder){

#ifdef DEBUG
	cout << "---->Chamada da funcao getMatrixPart" << endl;
#endif

	for(int i=0; i<newM.nLines*newM.nColumns; i++){
		newM.pData[i] = 1;
	}

	switch(partNumber){
		case 1:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(i<pixelsBorder){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						if(k<pixelsBorder){
							newM.pData[i*newM.nColumns+k] = 1;
						}else{
							newM.pData[i*newM.nColumns+k] = pData[(i-pixelsBorder)*nColumns+k-pixelsBorder];
						}
					}
				}
			}
			break;
		case 2:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(i<pixelsBorder){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						newM.pData[i*newM.nColumns+k] = pData[(i-pixelsBorder)*nColumns+(k+newM.nColumns-3*pixelsBorder)];
					}
				}
			}
			break;
		case 3:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(i<pixelsBorder){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						newM.pData[i*newM.nColumns+k] = pData[(i-pixelsBorder)*nColumns+(k+2*(newM.nColumns-2*pixelsBorder))-pixelsBorder];
					}
				}
			}
			break;
		case 4:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(i<pixelsBorder){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						if(k>(newM.nColumns-1-pixelsBorder)){
							newM.pData[i*newM.nColumns+k] = 1;
						}else{
							newM.pData[i*newM.nColumns+k] = pData[(i-pixelsBorder)*nColumns+(k+3*(newM.nColumns-2*pixelsBorder))-pixelsBorder];
						}
					}
				}
			}
			break;
		case 5:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(k<pixelsBorder){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						newM.pData[i*newM.nColumns+k] = pData[(i+newM.nLines-3*pixelsBorder)*nColumns+k-pixelsBorder];
					}
				}
			}
			break;
		case 6:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					newM.pData[i*newM.nColumns+k] = pData[(i+newM.nLines-3*pixelsBorder)*nColumns+(k+newM.nColumns-3*pixelsBorder)];
				}
			}
			break;
		case 7:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					newM.pData[i*newM.nColumns+k] = pData[(i+newM.nLines-3*pixelsBorder)*nColumns+(k+2*(newM.nColumns-2*pixelsBorder))-pixelsBorder];
				}
			}
			break;
		case 8:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(k>(newM.nColumns-1-pixelsBorder)){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						newM.pData[i*newM.nColumns+k] = pData[(i+newM.nLines-3*pixelsBorder)*nColumns+(k+3*(newM.nColumns-2*pixelsBorder))-pixelsBorder];
					}
				}
			}
			break;
		case 9:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(k<pixelsBorder){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						newM.pData[i*newM.nColumns+k] = pData[(i+2*(newM.nLines-2*pixelsBorder)-pixelsBorder)*nColumns+k-pixelsBorder];
					}
				}
			}
			break;
		case 10:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					newM.pData[i*newM.nColumns+k] = pData[(i+2*(newM.nLines-2*pixelsBorder)-pixelsBorder)*nColumns+(k+newM.nColumns-3*pixelsBorder)];
				}
			}
			break;
		case 11:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					newM.pData[i*newM.nColumns+k] = pData[(i+2*(newM.nLines-2*pixelsBorder)-pixelsBorder)*nColumns+(k+2*(newM.nColumns-2*pixelsBorder))-pixelsBorder];
				}
			}
			break;
		case 12:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(k>(newM.nColumns-1-pixelsBorder)){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						newM.pData[i*newM.nColumns+k] = pData[(i+2*(newM.nLines-2*pixelsBorder)-pixelsBorder)*nColumns+(k+3*(newM.nColumns-2*pixelsBorder))-pixelsBorder];
					}
				}
			}
			break;
		case 13:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(i>(newM.nLines-1-pixelsBorder+2)){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						if(k<pixelsBorder){
							newM.pData[i*newM.nColumns+k] = 1;
						}else{
							newM.pData[i*newM.nColumns+k] = pData[(i+3*(newM.nLines-2*pixelsBorder)-pixelsBorder)*nColumns+k-pixelsBorder];
						}
					}
				}
			}
			break;
		case 14:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(i>(newM.nLines-1-pixelsBorder+2)){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						newM.pData[i*newM.nColumns+k] = pData[(i+3*(newM.nLines-2*pixelsBorder)-pixelsBorder)*nColumns+(k+newM.nColumns-3*pixelsBorder)];
					}
				}
			}
			break;
		case 15:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(i>(newM.nLines-1-pixelsBorder+2)){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						newM.pData[i*newM.nColumns+k] = pData[(i+3*(newM.nLines-2*pixelsBorder)-pixelsBorder)*nColumns+(k+2*(newM.nColumns-2*pixelsBorder))-pixelsBorder];
					}
				}
			}
			break;
		case 16:
			for(int i=0; i<newM.nLines; i++){
				for(int k=0; k<newM.nColumns; k++){
					if(i>(newM.nLines-1-pixelsBorder+2)){
						newM.pData[i*newM.nColumns+k] = 1;
					}else{
						if(k>(newM.nColumns-1-pixelsBorder)){
							newM.pData[i*newM.nColumns+k] = 1;
						}else{
							newM.pData[i*newM.nColumns+k] = pData[(i+3*(newM.nLines-2*pixelsBorder)-pixelsBorder)*nColumns+(k+3*(newM.nColumns-2*pixelsBorder))-pixelsBorder];
						}
					}
				}
			}
			break;
		default:
			
			break;
	}
}


/*****************************************************************************
*****************************************************************************/
//Introduz um valor numa posicao especifica da matriz
template <class gType>
void predMatrix<gType>::IntroduzValor(int nLine, int nColumn, gType value){
	
#ifdef DEBUG
	cout << "---->Chamada da funcao IntroduzValor" << endl;
#endif
	
	if((nLine<0)||(nColumn<0)||(nLine>=nLines)||(nColumn>=nColumns)){
		cout << "ERRO: Posicao nao valida" << endl;
	}else{
		pData[nLine*nColumns+nColumn] = value;
	}
}

/*****************************************************************************
*****************************************************************************/
//inicializa uma matriz a zeros
template <class gType>
void predMatrix<gType>::Inicializa(void){

#ifdef DEBUG
	cout << "---->Chamada da funcao Inicializa" << endl;
#endif

	for(int i = 0; i < nLines*nColumns; i++)
		pData[i] = 0;
}

/*****************************************************************************
*****************************************************************************/
//Visualiza a matriz
template <class gType>
void predMatrix<gType>::Visualiza(void){

#ifdef DEBUG
	cout << "---->Chamada da funcao Visualiza" << endl;
#endif

	cout<<"---------------------"<<endl;
	for(int i = 0; i<nLines; i++){
		cout<<"[\t";
		for(int j = 0; j<nColumns; j++){
			cout<<pData[nColumns*i+j]<<"\t";
		}
		cout<<"]"<<endl;
	}
	cout<<"---------------------"<<endl;
}

/*****************************************************************************
*****************************************************************************/
//verifica se existe o valor na matrix
template <class gType>
int predMatrix<gType>::existeNaMatrix(const gType value){

#ifdef DEBUG
	cout << "---->Chamada da funcao existeNaMatrix" << endl;
#endif

	for(int i = 0; i<nLines*nColumns; i++){
		if(pData[i] == value)
			return 1;
		
	}
	return 0;

}

/*****************************************************************************
*****************************************************************************/
//retorna um valor da matrix
template <class gType>
gType predMatrix<gType>::getValue(const int position){

#ifdef DEBUG
	cout << "---->Chamada da funcao getValue" << endl;
#endif

	return pData[position];

}


/*****************************************************************************
*****************************************************************************/
//insere um valor na matrix
template <class gType>
void predMatrix<gType>::setValue(const int position, const gType value){

#ifdef DEBUG
	cout << "---->Chamada da funcao setValue" << endl;
#endif

	pData[position] = value;

}


// Calcula a inversa de uma matriz
/*----------------------------------------------------------------------------+
| Module: MatInv.c                                                           |
+-----------------------------------------------------------------------------+
| Description:  Implements functions to manipulate matrices                   |
|								              |
| Contact: Carlos Silvestre					              |
|	       							              |
| Notes:							              |
|	       							              |
| History:							              |
|	       							              |
|   Autor:	Date:		Comments:				      |
|   CJS		October de 1995   Version 1.0				      |
|									      |
| Copyright (c) 1995, ISR.						      |
+----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------+
| Function: minv					                      |
|									      |
| Args: mat, n, d, invmat						      |
|	mat    - input matrix.                                                |
|	n      - order of matrix mat.				              |
|	d      - resultant determinant.					      |
|	invmat - resultant inverse of mat.				      |
|									      |
|									      |
|									      |
| Action:                                                                     |
|   The standard gauss-jordan method is used. the determinant		      |
|   is also computed. A zero determinant indicates that 		      |
|   the matrix is singular.						      |
|									      |
|									      |
| Efects:                                                                     |
|									      |
+----------------------------------------------------------------------------*/
template <class gType>
void predMatrix<gType>::minv(double *mat,int n,double *d,double *invmat)
{
  int   k, kk, i, j, ij, ik, ki, kj, jk, ji, nk, iz, jp, jq, jr;
  double  hold, biga, *l, *m;

  /* copy from the original mat to invmat */
  for(i=0;i<=n*n-1;i++)
    {
      invmat[i]=mat[i];
    }

  /* space allocation for the auxiliar vectors */
  l=(double*)malloc((unsigned int)(n*sizeof(double)));
  m=(double*)malloc((unsigned int)(n*sizeof(double)));

  *d=1.0;
  nk= -n; 
  for(k=1;k<=n;k++)
    {
      nk=nk+n;
      l[-1+k]=k;
      m[-1+k]=k;
      kk=nk+k;
      biga=invmat[-1+kk];
      for(j=k;j<=n;j++)
	{ 
	  iz=n*(j-1);
	  for(i=k;i<=n;i++)
	    {
	      ij=iz+i;
	      if( ((double)fabs(biga))<((double)fabs(invmat[-1+ij])))
		{
		  biga=invmat[-1+ij];
		  l[-1+k]=i;
		  m[-1+k]=j;
		}
	    }
	}
      
      /* interchange rows*/
      j=(int)l[-1+k];
      if(j>k)
	{
	  ki=k-n;
	  for(i=1;i<=n;i++)
	    {
	      ki=ki+n;
	      hold= -invmat[-1+ki];
	      ji=ki-k+j;
	      invmat[-1+ki]=invmat[-1+ji];
	      invmat[-1+ji]=hold;
	    }
	}
      /* interchange columns */
      i=(int)m[-1+k];
      if(i>k)
	{
	  jp=n*(i-1);
	  for(j=1;j<=n;j++)
	    {
	      jk=nk+j;
	      ji=jp+j;
	      hold= -invmat[-1+jk];
	      invmat[-1+jk]=invmat[-1+ji];
	      invmat[-1+ji]=hold;
	    }
	}
      /*divide column by minus pivot (value of pivot element is*/
      /*contained in biga) */
      if(biga==0)
	{
	  *d=0.0;
	  free((char*)l);
	  free((char*)m);
	  return;
	} 
      for(i=1;i<=n;i++)
	{
	  if(i!=k)
	    {
	      ik=nk+i;
	      invmat[-1+ik]=invmat[-1+ik]/(-biga);
	    }
	}
      /*reduce invmatrix */
      for(i=1;i<=n;i++)
	{ 
	  ik=nk+i;
	  hold=invmat[-1+ik];
	  ij=i-n; 
	  for(j=1;j<=n;j++)
	    {
	      ij=ij+n;
	      if((i!=k)&&(j!=k))
		{
		  kj=ij-i+k;
		  invmat[-1+ij]=hold*invmat[-1+kj]+invmat[-1+ij];
		}
	    }
	}
      /*divide row by pivot*/
      kj=k-n;
      for(j=1;j<=n;j++)
	{
	  kj=kj+n;
	  if(j!=k)
	    {
	      invmat[-1+kj]=invmat[-1+kj]/biga;
	    }
	}
      /*product of pivots */
      *d=(*d)*biga;
      /*replace pivot by reciprocal */
      invmat[-1+kk]=1.0/biga;
    }
  /*final row and column interchange */
  
  for(k=n-1;k>0;k--) 
    {
      i=(int)l[-1+k];
      if(i>=k)
	{
	  jq=n*(k-1);
	  jr=n*(i-1);
	  for(j=1;j<=n;j++)
	    {
	      jk=jq+j;
	      hold=invmat[-1+jk];
	      ji=jr+j;
	      invmat[-1+jk]= -invmat[-1+ji];
	      invmat[-1+ji] =hold;
	    } 
	  j=(int)m[-1+k];
	}
      
      if(j>k)
	{ 
	  ki=k-n;
	  for(i=1;i<=n;i++)
	    {
	      ki=ki+n; 
	      hold=invmat[-1+ki];
	      ji=ki-k+j;
	      invmat[-1+ki]= -invmat[-1+ji];
	      invmat[-1+ji]=hold; 
	    }
	}
    }
  free((char*)l);
  free((char*)m);
  return;
}

/*****************************************************************************
*****************************************************************************/
//Funcao que le matrizes de um ficheiro
template <class gType>
void loadMatrices(char *fname, predMatrix <gType> * MM, int n)
{
	if (fname==0) {cout << "load matrices from cin not implemented\n"; return; }
	fstream fs(fname, ios::in);
	
	if (!fs.bad())
	{
		for (int i=0; i<n; i++)
		{
			int ncols, nlins;
			fs >> ncols >> nlins;
			predMatrix <double> A(ncols,nlins);
			fs >> A;
			MM[i]= A;
			cout << MM[i];
		}
		if (fname!=0)
			fs.close();
	}
}

/*****************************************************************************
*****************************************************************************/
//Funcao que escreve matrizes num ficheiro
template <class gType>
void saveMatrices(char *fname, predMatrix <gType> * MM, int n)
{
	if (fname==0) {cout << "save matrices to cout not implemented\n"; return; }
	fstream fs(fname, ios::out | ios::trunc);
	//ostream& fs= cout; if (fname!=0) {ofstream fs0(fname); fs=&fs0;}
	
	if (!fs.bad())
	{
		for (int i=0; i<n; i++)
		{
			fs<<MM[i].nLines<<" "<<MM[i].nColumns<<endl;
			fs << MM[i];
		}
		if (fname!=0)
			fs.close();
	}
}

#endif
