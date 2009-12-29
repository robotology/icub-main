#include "Matrix.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;

int   Matrix::bInverseOk = TRUE;

string Matrix::RemoveSpaces(string s){
  basic_string <char>::size_type pos;
  if(s.size()==0)
    return s;

  string ms = s;
  pos = ms.find_first_of(" \t\n");
  while(pos==0){
    ms = ms.substr(1);
    pos = ms.find_first_of(" \t\n");
  }
  pos = ms.find_last_of(" \t\n");
  while(pos==ms.size()-1){
    ms = ms.substr(0,ms.size()-1);
    pos = ms.find_last_of(" \t\n");
  }
  return ms;
}


bool Matrix::Load(const char* filename){
    vector<string> data;
    ifstream file;
    file.open(filename);
    char buf[4096];
    if(file.is_open()){
        file.getline(buf,4096);
        int row = 0;    
        while(!file.eof()){
            data.push_back(RemoveSpaces(string(buf)));
            if(data[row].length()>0){
                row++;
            }else{
                break;    
            }
            file.getline(buf,4096);    
        }
        file.close();
        if(row <= 0){
            Resize(0,0,false);
            return false;
        }
        double val;       
        stringstream ss(data[0]);
        int col = 0;
        while(!ss.eof()){
            ss >> val; col++;  
        }
        if(col <= 0){
            Resize(0,0,false);
            return false;            
        }
        Resize(row,col,false); Zero();
        cout << "File: "<<filename<<" - Matrix size : ("<<row<<","<<col<<")"<<endl;
        
        bool bRet = true;
        for(int i=0;i<row;i++){
            stringstream ss(data[i]);
            double val;
            int cnt = 0;
            while((!ss.eof())&&(cnt<col)){
                ss >> _[i*column+cnt]; cnt++;
            }    
            if(cnt!=col) bRet = false;           
        }
        return bRet;
    }
    return false;       
}

bool Matrix::Save(const char* filename){
    ofstream file;
    file.open(filename);
    if(file.is_open()){
        int row = RowSize();
        int col = ColumnSize();
        for(int i=0;i<row;i++){
            for(int j=0;j<col;j++){
                file << _[i*column+j]<< " ";
            }            
            file << endl;
        }
        file.close();
        return true;
    }else{
        return false;    
    }
}
