#include "XmlTree.h"
#include "Various.h"

#include <stdlib.h>
#include <string.h>

#ifdef WIN32
#pragma warning( disable : 4996)
#endif

#define XTREE_START_TAG   0
#define XTREE_DATA        1
#define XTREE_STOP_TAG    2
#define XTREE_EOF         3

#define XTREE_DEPLOY      2
#define XTREE_OK          1
#define XTREE_ERROR       0


XmlTree::XmlTree()
{
  m_ArrayPtr = NULL;
	Clear();
}

XmlTree::XmlTree(string newName)
{
  m_ArrayPtr = NULL;
  Clear();
  SetName(newName);
}
XmlTree::XmlTree(string newName, string newData)
{
  m_ArrayPtr = NULL;
  Clear();
  SetName(newName);
  SetData(newData);
}

XmlTree::XmlTree(string newName, string newData, string params)
{
  m_ArrayPtr = NULL;
	Clear();
  SetName(newName);
  SetData(newData);
  SetParamsString(params);
}

XmlTree::XmlTree(string newName, string newData, int nbSubTree, ... )
{
  m_ArrayPtr = NULL;
	Clear();
  SetName(newName);
  SetData(newData);
 
  va_list subTrees;
  va_start(subTrees,nbSubTree);
  pXmlTree tTree = NULL;

  for(int i=0;i<nbSubTree;i++)
   {
     tTree = va_arg( subTrees, pXmlTree);
     AddSubTree(tTree);
   }
   va_end( subTrees );   
}

XmlTree::XmlTree(string newName, string newData, string params, int nbSubTree, ... )
{
  m_ArrayPtr = NULL;
	Clear();
  SetName(newName);
  SetData(newData);
  SetParamsString(params);

  va_list subTrees;
  va_start(subTrees,nbSubTree);
  pXmlTree tTree = NULL;

  for(int i=0;i<nbSubTree;i++)
   {
     tTree = va_arg( subTrees, pXmlTree);
     AddSubTree(tTree);
   }
   va_end( subTrees );   
}


XmlTree::~XmlTree()
{
	Clear();
}

pXmlTree XmlTree::Clone(){
  pXmlTree clone = new XmlTree(m_Name,m_Data,GetParamsString());
  if(m_ArrayPtr!=NULL){
    clone->m_ArrayPtr       = malloc(m_ArraySize*m_ArrayTypeSize);
    clone->m_ArraySize      = m_ArraySize;
    clone->m_ArrayTypeSize  = m_ArrayTypeSize;
    memcpy(clone->m_ArrayPtr,m_ArrayPtr,m_ArraySize*m_ArrayTypeSize);
  }
  for(unsigned int i=0;i<m_SubTrees.size();i++)
    clone->AddSubTree(m_SubTrees[i]->Clone());
  return clone;
}

void XmlTree::Clone(pXmlTree src){
  if(src==NULL)
    return;
  Clear();
  SetName(src->m_Name);
  SetData(src->m_Data);
  SetParamsString(src->GetParamsString());
  if(src->m_ArrayPtr!=NULL){
    m_ArraySize      = src->m_ArraySize;
    m_ArrayTypeSize  = src->m_ArrayTypeSize;
    m_ArrayPtr       = malloc(m_ArraySize*m_ArrayTypeSize);
    memcpy(m_ArrayPtr,src->m_ArrayPtr,m_ArraySize*m_ArrayTypeSize);
  }
  for(unsigned int i=0;i<src->m_SubTrees.size();i++)
    AddSubTree(src->m_SubTrees[i]->Clone());  
}


string XmlTree::GetName(){
  return m_Name;
}
string XmlTree::GetData(){
  return m_Data;
}
pXmlTreeList XmlTree::GetParams(){
  return &m_Params;
}
string XmlTree::GetParamsString(){
  string s="";
  for(unsigned int i=0;i<m_Params.size();i++){
    if(i>0)
      s.append(" ");
    s.append(m_Params[i]->GetName());
    s.append("=\"");
    s.append(m_Params[i]->GetData());
    s.append("\"");
  }
  return s;
}
void XmlTree::SetName(string newName){
  m_Name = newName;
}
void XmlTree::SetData(string newData){
  m_Data = newData;
}
void  XmlTree::AppendToData(string newData){
  m_Data.append(newData);
}

void  XmlTree::SetParams(pXmlTreeList params){
  for(unsigned int i=0;i<m_Params.size();i++)
    delete m_Params[i];
  m_Params.clear();

  if(params==NULL)
    return;

  for(unsigned int i=0;i<params->size();i++)
    m_Params.push_back(params->at(i)->Clone());
}

void  XmlTree::SetParamsString(string params){
  //if(params.length()>0)
  //  cout << params<<endl;
  for(unsigned int i=0;i<m_Params.size();i++)
    delete m_Params[i];
  m_Params.clear();

  vector<string> tokens = Tokenize(params," \t\"="," \t");
/*  if(params.length()>0){
    for(unsigned int i=0;i<tokens.size();i++){
      cout << "<**"<<tokens[i] <<"**> ";
    }
    cout <<endl;
  }*/
  string pname,pdata;
  bool  bDataStart=false;
  for(unsigned int i=0;i<tokens.size();i++){
    if(tokens[i]=="="){
      if((i>0) && (tokens[i-1].length()>0)) pname = tokens[i-1];  
    }else if(tokens[i]=="\""){
      if(bDataStart){
        pXmlTree tmpTree = new XmlTree(pname,pdata);
        //tmpTree->Print();
        m_Params.push_back(tmpTree);           
      }else{
        pdata = "";    
      }                
      bDataStart =!bDataStart;
    }else{      
      if(bDataStart){
        if(pdata.length()>0) pdata.append(" ");
        pdata.append(tokens[i]);
      }
    }
  }
    
  /*          
  params.append(" ");
  istringstream ss(params);
  string s;
  while(!ss.eof()){
    ss >> s;
    if(ss.eof())
      return;
    unsigned int eqIndex = s.find("=");
    if(eqIndex==string::npos)
      continue;
    pXmlTree tmpTree = new XmlTree(s.substr(0,eqIndex),s.substr(eqIndex+2,s.size()-eqIndex-3));
    tmpTree->Print();
    m_Params.push_back(tmpTree);
  }*/
}

pXmlTreeList XmlTree::GetSubTrees(){
  return &m_SubTrees;
}

pXmlTree XmlTree::Find(string name, int no){
  return Find(name,"",no);
}

pXmlTree XmlTree::Find(string name, string data, int no){
  unsigned int pointPos = name.find(".");
  string cTag;
  string eTag;

  if(pointPos != string::npos){
    cTag  = name.substr(0,pointPos);
    eTag  = name.substr(pointPos+1,name.size()-pointPos-1);
  }else{
    cTag  = name;
  }
  int cnt = 0;
  for(unsigned int i=0;i<m_SubTrees.size();i++){
    if(cTag.compare(m_SubTrees[i]->m_Name)==0){
      if(eTag.size()==0){
        if(data.size()==0){
          if(cnt==no)
            return m_SubTrees[i];
          cnt++;
        }else{
          if(data.compare(m_SubTrees[i]->m_Data)==0){
            if(cnt==no)
              return m_SubTrees[i];
            cnt++;
          }
        }
      }else{
        return m_SubTrees[i]->Find(eTag,data,no);
      }
    }
  }
  return NULL;
}


string XmlTree::FindData(string name){
  pXmlTree tree = Find(name);
  if(tree!=NULL)
    return tree->m_Data;
  return "";
}

void XmlTree::Clear()
{
	SetName("");
	SetData("");
  SetParamsString("");
  if(m_ArrayPtr!=NULL)
    free(m_ArrayPtr);
  m_ArrayPtr      = NULL;
  m_ArraySize     = 0;
  m_ArrayTypeSize = 0;

  m_BasePath = "";
  for(unsigned int i=0;i<m_SubTrees.size();i++)
		delete m_SubTrees[i];
	m_SubTrees.clear();
}

int XmlTree::AddSubTree(pXmlTree newSubTree)
{
	if(newSubTree!=NULL)
		m_SubTrees.push_back(newSubTree);

	return m_SubTrees.size();
}

int XmlTree::DelSubTree(pXmlTree oldSubTree)
{
  if(oldSubTree!=NULL){
    for(unsigned int i=0;i<m_SubTrees.size();i++)
      if(m_SubTrees[i] == oldSubTree){
        m_SubTrees.erase(m_SubTrees.begin()+i);
        delete oldSubTree;        
        break;
      }else{
        m_SubTrees[i]->DelSubTree(oldSubTree);
      }
  }

	return m_SubTrees.size();
}

void XmlTree::FindAndReplace(string data2find, string data2replace){
  for(unsigned int i=0;i<m_SubTrees.size();i++)
    m_SubTrees[i]->FindAndReplace(data2find,data2replace);
    
  if(m_Data.find(data2find,0)!=string::npos){     
    vector<string> tokens = Tokenize(m_Data);
    for(unsigned int i=0;i<tokens.size();i++){
      if(tokens[i]==data2find){
        tokens[i] = data2replace;  
      }
    } 
    m_Data = Serialize(tokens);  
  }
}


void XmlTree::Print(int indent)
{
  unsigned int i;
  char *c = new char[indent+1];
	
  for(i=0;i<(unsigned int)indent;i++)	c[i]=' ';
	c[i]=0;

  cout << c << "<"<<m_Name;
  if(m_Params.size()>0)
    cout <<" "<<GetParamsString();   
  cout << "> "<< m_Data;
  if(m_SubTrees.size()>0){
    cout <<endl;
	  for(i=0;i<m_SubTrees.size();i++)
  		m_SubTrees[i]->Print(indent+2);
    cout << c;
  }else{
    cout << " ";
  }
  cout <<"</"<<m_Name<<">"<<endl;
  //cout <<endl;
	
	delete c;
}
int XmlTree::GetNextString  (ifstream & file, string & str, string &currStr){
	string s;

  if(currStr.length()<=0){
	  if (file.eof()) return XTREE_EOF;
	  file >> s;
  }else{
    s = currStr;
  }
  if(s[0]=='<'){
    unsigned int pos = s.find_first_of(">");
    if(pos!=string::npos){
      currStr = s.substr(pos+1);
      s = s.substr(0,pos+1);
    }else{
      currStr="";
    }
  }else{
    unsigned int pos = s.find_first_of("<");
    if(pos!=string::npos){
      currStr = s.substr(pos);
      s = s.substr(0,pos);
    }else{
      currStr="";
    }
  }

  str = s;
  return XTREE_DATA;
}
int XmlTree::GetNextTag(ifstream & file, string & str, string & params, string &currStr)
{
	string s;

  if(GetNextString(file,s,currStr)==XTREE_EOF)
    return XTREE_EOF;

	int length = s.length();

  if (s[0]=='<'){ 
    if(s[length-1]=='>'){
		  if(s[1]!='/'){
			  str.assign(s,1,length-2);
			  return XTREE_START_TAG;
		  }else{
			  str.assign(s,2,length-3);
			  return XTREE_STOP_TAG;
		  }
    }else{
      str.assign(s,1,length-1);
      params.assign("");
      while((s[s.length()-1]!='>')){
        if(GetNextString(file,s,currStr)==XTREE_EOF)
          break;
        if(s[s.length()-1]!='>')
          params.append(s);
        else
          params.append(s.substr(0,s.length()-1));
        params.append(" ");
      }
      return XTREE_START_TAG;
    }
  }else{
		str = s;
		return XTREE_DATA;
	}
}



int XmlTree::LoadFromFile(string filename)
{
	ifstream  file;
	string    s,p;
  int       result = XTREE_ERROR;

	file.open(filename.c_str());
  if(file.is_open()){
    m_BasePath = GetPathFromFilename(filename);

    string currS = "";
    int res = GetNextTag(file,s,p,currS);

	  if((res==XTREE_START_TAG)){
  		SetName(s);
      SetParamsString(p);      
		  result = LoadFromStream(file,currS,m_BasePath);
	  }
  
		file.close();
  }else{
    cout << "File: "<<filename<<" not found..."<<endl;
  }
  
  return result;
}


int	XmlTree::LoadFromStream(ifstream & file, string & currStr, string currPath)
{
	string s,p;
	m_Data ="";

	while(!file.eof()){
    s="";
    p="";
		int res = GetNextTag(file,s,p,currStr);
		switch(res){

		case XTREE_DATA:
			m_Data.append(s);
			m_Data.append(" ");
			break;

		case XTREE_START_TAG:
			{
				pXmlTree newField = new XmlTree(s);
        newField->SetParamsString(p);
        int flag = newField->LoadFromStream(file,currStr,currPath);
        if(flag == XTREE_OK){
          
          if(newField->GetName()=="Instance"){
            pXmlTree templates = Find("Templates");
            if(templates!=NULL){
              pXmlTree myTempl = templates->Find(newField->GetParamValue("type"),newField->GetParamValue("name"));
              if(myTempl!=NULL){
                pXmlTree myCopy = myTempl->Clone();
                myCopy->SetData(newField->GetData());
                for(unsigned int i=0;i<newField->m_SubTrees.size();i++){
                  myCopy->Fill(newField->m_SubTrees[i]);
                }
                delete newField;
                newField = myCopy;
              }
            }
          }
					
          AddSubTree(newField);
          
        }else if(flag == XTREE_DEPLOY){
          for(unsigned int i=0;i<newField->m_SubTrees.size();i++){
            AddSubTree(newField->m_SubTrees[i]);
          }
          newField->m_SubTrees.clear();
        }else{
          printf("XTree Error: Tag <%s>: %s\n",m_Name.c_str(),m_Data.c_str()); 
          delete newField;
					return XTREE_ERROR;
        }
			}
			break;

		case XTREE_STOP_TAG:
			if(m_Name.compare(s)==0){
				if(m_Data.size()>0){
					m_Data.assign(m_Data,0,m_Data.size()-1);
        }                    
        if(m_Name.compare("Include")==0){
					string newFile;
					newFile.assign(currPath);
          if(GetParamValue("file").length()>0){
    				newFile.append(GetParamValue("file"));
            pXmlTree newField = new XmlTree();
    				newField->LoadFromFile(newFile);
            
            
            if(GetParamValue("preReplace").length()>0){
              vector<string> replacePairs = Tokenize(GetParamValue("preReplace"));
              for(unsigned int k=0;k<replacePairs.size()/2;k++){
                newField->FindAndReplace(replacePairs[2*k],replacePairs[2*k+1]);                
              }     
            }
            
            pXmlTree subField;
            string oldData = m_Data;
            /*if(m_Data.length()==0)
              subField = newField;
            else
              subField = newField->Find(m_Data);
            */
            subField = newField;
            if(subField!=NULL){
              for(unsigned int i=0;i<m_SubTrees.size();i++){                
                subField->Fill(m_SubTrees[i]);
              }
            }
            if(GetParamValue("postReplace").length()>0){                            
              vector<string> replacePairs = Tokenize(GetParamValue("postReplace"));
              for(unsigned int k=0;k<replacePairs.size()/2;k++){
                newField->FindAndReplace(replacePairs[2*k],replacePairs[2*k+1]);                
              }     
            }

            string newName = GetParamValue("name");
            string newData = GetParamValue("data");
            string subOnly = GetParamValue("SubtreesOnly");

            Clone(subField);
            delete newField;
            if(newName.length()>0)  SetName(newName);           
            if(newData.length()>0)  SetData(newData);
            if(oldData.length()>0)  SetData(oldData);
            

            if(subOnly.length()>0){
              return XTREE_DEPLOY;
            }
          }
        }
				return XTREE_OK;
			}
			else
				return XTREE_ERROR;

		case XTREE_EOF:
			return XTREE_ERROR;

		}
	}	
	return XTREE_ERROR;
}



int XmlTree::SaveToFile(string filename)
{
	ofstream file;
	string s;
  int result = XTREE_ERROR;

	file.open(filename.c_str());
  if(file.is_open()){
	  result = SaveToStream(file);
	  file.close();
  }
	return result;
}

int XmlTree::SaveToStream(ofstream & file, int indent)
{
	int i;
	char *ind = new char[indent+1];
	for(i=0;i<indent;i++) ind[i]=' ';
	ind[i]=0;

	file << ind<<"<" << m_Name;
  if(m_Params.size()>0)
    file <<" "<<GetParamsString();  
  file << "> " << m_Data;
  if(m_SubTrees.size()==0){
    file << " ";
  }else{
    file << endl;
	  for(i=0;i<(int)m_SubTrees.size();i++)
		  m_SubTrees[i]->SaveToStream(file,indent+2);
    file << ind;
  }
	file << "</" << m_Name << ">"<<endl;
	delete ind;
	return XTREE_OK;
}

void XmlTree::Fill(pXmlTree newTree){
  if(newTree==NULL)
    return;
  pXmlTree subTree = Find(newTree->GetName(),newTree->GetData());
  if(subTree == NULL) subTree = Find(newTree->GetName());
  if(subTree!=NULL){
    subTree->SetData(newTree->GetData());
    subTree->SetParamsString(newTree->GetParamsString());
    for(unsigned int i=0;i<newTree->m_SubTrees.size();i++){
      subTree->Fill(newTree->m_SubTrees[i]);
    }
  }else{
    AddSubTree(newTree->Clone());
  }
}

string	XmlTree::ToString(){
  return m_Data;
}

string XmlTree::Get(string name, string def){
  pXmlTree tmp = Find(name);
  return (tmp==NULL?def:tmp->ToString());
}

string XmlTree::CGet(string name, string def){
  pXmlTree tmp = Find(name);
  if(tmp==NULL) AddSubTree(new XmlTree(name,def));
  return (tmp==NULL?def:tmp->ToString());
}

void XmlTree::Set(string name, string def){
  pXmlTree tmp = Find(name);
  if(tmp!=NULL) tmp->SetData(def);
  else AddSubTree(new XmlTree(name,def));
}

double	XmlTree::ToDouble(){
	istringstream ss(m_Data);
	double tmp = 0.0;
	ss >> tmp;
	return tmp;
}
double XmlTree::Get(string name, double def){
  pXmlTree tmp = Find(name);
  return (tmp==NULL?def:tmp->ToDouble());
}
double XmlTree::CGet(string name, double def){
  pXmlTree tmp = Find(name);
  if(tmp==NULL) AddSubTree(new XmlTree(name,DoubleToString(def)));
  return (tmp==NULL?def:tmp->ToDouble());
}
void XmlTree::Set(string name, double def){
  pXmlTree tmp = Find(name);
  if(tmp!=NULL) tmp->SetData(DoubleToString(def));
  else AddSubTree(new XmlTree(name,DoubleToString(def)));
}

float	XmlTree::ToFloat(){
	istringstream ss(m_Data);
	float tmp = 0.0f;
	ss >> tmp;
	return tmp;
}
float XmlTree::Get(string name, float def){
  pXmlTree tmp = Find(name);
  return (tmp==NULL?def:tmp->ToFloat());
}
float XmlTree::CGet(string name, float def){
  pXmlTree tmp = Find(name);
  if(tmp==NULL) AddSubTree(new XmlTree(name,FloatToString(def)));
  return (tmp==NULL?def:tmp->ToFloat());
}
void XmlTree::Set(string name, float def){
  pXmlTree tmp = Find(name);
  if(tmp!=NULL) tmp->SetData(FloatToString(def));
  else AddSubTree(new XmlTree(name,FloatToString(def)));
}

int		XmlTree::ToInt(){
	istringstream ss(m_Data);
	int tmp = 0;
	ss >> tmp;
	return tmp;
}
int XmlTree::Get(string name, int def){
  pXmlTree tmp = Find(name);
  return (tmp==NULL?def:tmp->ToInt());
}
int XmlTree::CGet(string name, int def){
  pXmlTree tmp = Find(name);
  if(tmp==NULL) AddSubTree(new XmlTree(name,IntToString(def)));
  return (tmp==NULL?def:tmp->ToInt());
}
void XmlTree::Set(string name, int def){
  pXmlTree tmp = Find(name);
  if(tmp!=NULL) tmp->SetData(IntToString(def));
  else AddSubTree(new XmlTree(name,IntToString(def)));
}

bool	XmlTree::ToBool(){
	bool tmp = false;
	tmp = ((m_Data.compare("true")==0||m_Data.compare("1")==0)?true:false);
	return tmp;
}
bool XmlTree::Get(string name, bool def){
  pXmlTree tmp = Find(name);
  return (tmp==NULL?def:tmp->ToBool());
}
bool XmlTree::CGet(string name, bool def){
  pXmlTree tmp = Find(name);
  if(tmp==NULL) AddSubTree(new XmlTree(name,BoolToString(def)));
  return (tmp==NULL?def:tmp->ToBool());
}
void XmlTree::Set(string name, bool def){
  pXmlTree tmp = Find(name);
  if(tmp!=NULL) tmp->SetData(BoolToString(def));
  else AddSubTree(new XmlTree(name,BoolToString(def)));
}

int XmlTree::GetArray(string name, float** array){
  pXmlTree tmp = Find(name);
  if(tmp!=NULL){
    int size=0;
    *array = tmp->ToArrayFloat(&size);
    return size;
  }
  return 0;  
}
int XmlTree::GetArray(string name, double** array){
  pXmlTree tmp = Find(name);
  if(tmp!=NULL){
    int size=0;
    *array = tmp->ToArrayDouble(&size);
    return size;
  }
  return 0;  
}


double *XmlTree::ToArrayDouble(int *size){
  if(m_ArrayPtr!=NULL) free(m_ArrayPtr);
  string cData = RemoveSpaces(m_Data);
  istringstream ss(cData);
  vector<double> list;
  double d    = 0.0;
  while(!ss.eof()){
    ss >> d;
    list.push_back(d);
  }
  m_ArraySize = list.size();
  m_ArrayTypeSize = sizeof(double);
  if(m_ArraySize>0){
    m_ArrayPtr = malloc(m_ArraySize*sizeof(double));
    for(int i=0;i<m_ArraySize;i++)
      ((double*)m_ArrayPtr)[i] = list[i];    
  }
  if(size!=NULL) *size = m_ArraySize;
  return ((double*)m_ArrayPtr);
}

float *XmlTree::ToArrayFloat(int *size){
  if(m_ArrayPtr!=NULL) free(m_ArrayPtr);
  string cData = RemoveSpaces(m_Data);
  istringstream ss(cData);
  vector<float> list;
  float d    = 0.0f;
  while(!ss.eof()){
    ss >> d;
    list.push_back(d);
  }
  m_ArraySize = list.size();
  m_ArrayTypeSize = sizeof(float);
  if(m_ArraySize>0){
    m_ArrayPtr = malloc(m_ArraySize*sizeof(float));
    for(int i=0;i<m_ArraySize;i++)
      ((float*)m_ArrayPtr)[i] = list[i];    
  }
  if(size!=NULL) *size = m_ArraySize;
  return ((float*)m_ArrayPtr);
}

int *XmlTree::ToArrayInt(int *size){
  if(m_ArrayPtr!=NULL) free(m_ArrayPtr);
  string cData = RemoveSpaces(m_Data);
  istringstream ss(cData);
  vector<int> list;
  int d    = 0;
  while(!ss.eof()){
    ss >> d;
    list.push_back(d);
  }
  m_ArraySize = list.size();
  m_ArrayTypeSize = sizeof(int);
  if(m_ArraySize>0){
    m_ArrayPtr = malloc(m_ArraySize*sizeof(int));
    for(int i=0;i<m_ArraySize;i++){
      ((int*)m_ArrayPtr)[i] = list[i];
    }
  }
  if(size!=NULL) *size = m_ArraySize;
  return ((int*)m_ArrayPtr);
}

bool *XmlTree::ToArrayBool(int *size){
  if(m_ArrayPtr!=NULL) free(m_ArrayPtr);
  string cData = RemoveSpaces(m_Data);
  istringstream ss(cData);
  vector<bool> list;
  bool d    = false;
  while(!ss.eof()){
    ss >> d;
    list.push_back(d);
  }
  m_ArraySize = list.size();
  m_ArrayTypeSize = sizeof(bool);
  if(m_ArraySize>0){
    m_ArrayPtr = malloc(m_ArraySize*sizeof(bool));
    for(int i=0;i<m_ArraySize;i++)
      ((bool*)m_ArrayPtr)[i] = list[i];    
  }
  if(size!=NULL) *size = m_ArraySize;
  return ((bool*)m_ArrayPtr);
}

int       XmlTree::GetArraySize(){
  return (m_ArrayPtr!=NULL?m_ArraySize:0);
}

void     *XmlTree::GetArray(){
  return m_ArrayPtr;
}


void     *XmlTree::TransposeArray(int rowSize){
  int fSize = GetArraySize();

  if(rowSize<=0)        return m_ArrayPtr;
  if(fSize<=0)          return m_ArrayPtr;
  if(m_ArrayPtr==NULL)  return m_ArrayPtr;
  int step = fSize/rowSize;
  if(step<=0)           return m_ArrayPtr;


  void *tmpArray = malloc(m_ArraySize*m_ArrayTypeSize);

  for(int i=0;i<step;i++){
    for(int j=0;j<rowSize;j++){
      for(int k=0;k<m_ArrayTypeSize;k++){
        ((char*)tmpArray)[(i*rowSize+j)*m_ArrayTypeSize+k] =
          ((char*)m_ArrayPtr)[(j*step+i)*m_ArrayTypeSize+k];
      }
    }
  }

  memcpy(m_ArrayPtr,tmpArray,m_ArraySize*m_ArrayTypeSize);
  free(tmpArray);

  return m_ArrayPtr;
}

string    XmlTree::GetParamValue(string param){
  for(unsigned int i=0;i<m_Params.size();i++){
    if(param == m_Params[i]->GetName())
      return m_Params[i]->GetData();
  }
  return "";
}

void XmlTree::SetParamValue(string param, string value){
  bool bFound = false;
  for(unsigned int i=0;i<m_Params.size();i++){
    if(param == m_Params[i]->GetName()){
      m_Params[i]->SetData(value);
      bFound = true;
    }
  }  
  if(!bFound){
    m_Params.push_back(new XmlTree(param,value));
  }
}

void XmlTree::SetArraySize(int size, int itemSize){
  if(m_ArrayPtr!=NULL) free(m_ArrayPtr);
  m_ArrayPtr      = NULL;
  m_ArraySize     = size;
  m_ArrayTypeSize = itemSize;
  if(m_ArraySize>0){
    m_ArrayPtr = malloc(m_ArraySize*m_ArrayTypeSize);
    memset(m_ArrayPtr,0,m_ArraySize*m_ArrayTypeSize);
  }
}

string XmlTree::GetBasePath(){
  return m_BasePath;  
} 
