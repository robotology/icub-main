// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Eric Sauser, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
// TreeParser.cpp: implementation of the sXMLParser class.
//
//////////////////////////////////////////////////////////////////////

#include "TreeParser.h"

//string	Tree::CurrentPath;

Tree::Tree()
{
	Clear();
}

Tree::Tree(string newName)
{
	Clear();
	m_Name.assign(newName);
}
Tree::Tree(string newName, string newData)
{
	Clear();
	m_Name.assign(newName);
	m_Data.assign(newData);
}
/*
Tree::Tree(string newName, string newData, Tree_List & newSubTree)
{
	Clear();
	m_Name.assign(newName);
	m_Data.assign(newData);
	for(int i=0;i<newSubTree.size();i++)
		m_SubTree.push_back(newSubTree[i]);
}
*/
Tree::~Tree()
{
	Clear();
}

pTree Tree::Clone(){
  pTree clone = new Tree(m_Name);
  clone->m_Data = m_Data;
  for(unsigned int i=0;i<m_SubTree.size();i++)
		clone->m_SubTree.push_back(m_SubTree[i]->Clone());
  return clone;
}

string Tree::GetName(){
  return m_Name;
}
string Tree::GetData(){
  return m_Data;
}
void Tree::SetName(string newName){
  m_Name = newName;
}
void Tree::SetData(string newData){
  m_Data = newData;
}

pTree Tree::Find(string name){
  return Find(name,"");
}

pTree Tree::Find(string name, string data){
  unsigned int pointPos = name.find(".");
  string cTag;
  string eTag;

  if(pointPos != string::npos){
    cTag  = name.substr(0,pointPos);
    eTag  = name.substr(pointPos+1,name.size()-pointPos-1);
  }else{
    cTag  = name;
  }

  //  cout <<name<<" = "<< cTag<<"."<<eTag<<endl;

  for(unsigned int i=0;i<m_SubTree.size();i++){
    if(cTag.compare(m_SubTree[i]->m_Name)==0){
      if(eTag.size()==0){
        if(data.size()==0){
          return m_SubTree[i];
        }else{
          if(data.compare(m_SubTree[i]->m_Data)==0)
            return m_SubTree[i];
        }
      }else{
        return m_SubTree[i]->Find(eTag,data);
      }
    }
  }
  return NULL;
}


string Tree::FindData(string name){
  pTree tree = Find(name);
  if(tree!=NULL)
    return tree->m_Data;
  return "";
}

void Tree::Clear()
{
  for(unsigned int i=0;i<m_SubTree.size();i++)
		delete m_SubTree[i];
	m_SubTree.clear();
	m_Data.assign("");
	m_Name.assign("");
}

int Tree::AddSubTree(pTree newSubTree)
{
	if(newSubTree!=NULL)
		m_SubTree.push_back(newSubTree);

	return m_SubTree.size();
}

int Tree::DelSubTree(pTree oldSubTree)
{
  if(oldSubTree!=NULL){
    for(unsigned int i=0;i<m_SubTree.size();i++)
      if(m_SubTree[i] == oldSubTree){
        m_SubTree.erase(m_SubTree.begin()+i);
        delete oldSubTree;        
        break;
      }else{
        m_SubTree[i]->DelSubTree(oldSubTree);
      }
  }

	return m_SubTree.size();
}

void Tree::Print(unsigned int indent)
{
  unsigned int i;
  char *c = new char[indent+1];
  
  for(i=0;i<indent;i++)	c[i]=' ';
  c[i]=0;
  
  cout << c << m_Name << "\t\t" << m_Data << endl;
  
  for(i=0;i<m_SubTree.size();i++)
    m_SubTree[i]->Print(indent+2);

  delete c;
}

int Tree::GetNextTag(ifstream & file, string & str)
{
	string s;

	if (file.eof()) return TREE_EOF;

	file >> s;
	int length = s.length();
	if ((s[0]=='<') && (s[length-1]=='>')){
		if(s[1]!='/'){
			str.assign(s,1,length-2);
			return TREE_START_TAG;
		}else{
			str.assign(s,2,length-3);
			return TREE_STOP_TAG;
		}
	}
	else{
		str = s;
		return TREE_DATA;
	}
}



int Tree::LoadFromFile(string filename)
{
	ifstream  file;
	string    s;
  int       result = TREE_ERROR;

  /*
	string   ContextCurrentPath = CurrentPath;

	for(int i=filename.size()-1;i>=0;i--)
		if((filename[i]=='/') || (filename[i]=='\\'))
			break;
	if(i>=0) CurrentPath = filename.substr(0,i+1);
  */

	file.open(filename.c_str());
  if(file.is_open()){
    int res = GetNextTag(file,s);

	  if((res==TREE_START_TAG)){
  		m_Name = s;
		  result = LoadFromStream(file);
	  }
  
		file.close();
  }else{
    printf("Error: File not found\n");
  }
  
  return result;
  /*
  else{
		CurrentPath = ContextCurrentPath;
		return false;
	}
	CurrentPath = ContextCurrentPath;
  */
	//return true;
}


int	Tree::LoadFromStream(ifstream & file)
{
  //	char c=' ';
	string s;
	m_Data ="";

	while(!file.eof()){

		int res = GetNextTag(file,s);
		switch(res){

		case TREE_DATA:
			m_Data.append(s);
			m_Data.append(" ");
			break;

		case TREE_START_TAG:
			{
			  //			  printf("Start Tag found: %s\n",s.c_str());
				pTree newField = new Tree(s);
        int flag = newField->LoadFromStream(file);
        if(flag == TREE_OK){
					AddSubTree(newField);
        }else{
          delete newField;
					return TREE_ERROR;
        }
			}
			break;

		case TREE_STOP_TAG:
			if(m_Name.compare(s)==0){
				if(m_Data.size()>0){
					m_Data.assign(m_Data,0,m_Data.size()-1);
					//					printf("Block found: Start Tag: %s, Data %s\n",m_Name.c_str(),m_Data.c_str());

				
          /*
          if(name.compare("Include")==0){
						string newFile;
						newFile.assign(CurrentPath);
						newFile.append(data);
						this->LoadFromXMLFile(newFile);
					}
          */
				}
				return TREE_OK;
			}
			else{
			  printf("Stop Tag incorrect: %s instead of %s having data %s.\n",s.c_str(),m_Name.c_str(),m_Data.c_str());
				return TREE_ERROR;
			}

		case TREE_EOF:
		  {
		    printf("End of File reached: Missing Stop Tag: %s.\n",m_Name.c_str());
			return TREE_ERROR;
		  }

		}
	}	
	return TREE_ERROR;
}



int Tree::SaveToFile(string filename)
{
	ofstream file;
	string s;
  int result = TREE_ERROR;

	file.open(filename.c_str());
  if(file.is_open()){
	  result = SaveToStream(file);
	  file.close();
  }
	return result;
}

int Tree::SaveToStream(ofstream & file, unsigned int indent)
{
	unsigned int i;
	char *ind = new char[indent+1];
	for(i=0;i<indent;i++) ind[i]=' ';
	ind[i]=0;

	file << ind<<"<" << m_Name << "> " << m_Data << endl;
	for(i=0;i<m_SubTree.size();i++)
		m_SubTree[i]->SaveToStream(file,indent+2);

	file << ind<<"</" << m_Name << ">"<<endl;
	delete ind;
	return TREE_OK;
}


double	Tree::ToDouble(){
	istringstream ss(m_Data);
	double tmp = 0.0;
	ss >> tmp;
	return tmp;
}
float	Tree::ToFloat(){
	istringstream ss(m_Data);
	float tmp = 0.0f;
	ss >> tmp;
	return tmp;
}
int		Tree::ToInt(){
	istringstream ss(m_Data);
	int tmp = 0;
	ss >> tmp;
	return tmp;
}
bool	Tree::ToBool(){
	bool tmp = false;
	tmp = (m_Data.compare("true")==0?true:false);
	return tmp;
}

Tree_List *Tree::GetSubTrees(){
  return &m_SubTree;
}

/*

vector<TreeVisitorRegistryItem*>	TreeVisitorRegistry::Items;


TreeVisitorRegistryItem* TreeVisitorRegistry::GetItem(string name){
	for(unsigned int i=0;i<Items.size();i++)
		if(Items[i]->Name.compare(name) == 0)
			return Items[i];
	return NULL;
}

int TreeVisitorRegistry::Register(TreeVisitor*(*NewFunc)(), string name){
	if(GetItem(name) == NULL){
		TreeVisitorRegistryItem *tmp = new TreeVisitorRegistryItem();
		tmp->Name    = name;
		tmp->NewFunc = NewFunc; 
		Items.push_back(tmp);
		return true;
	}
	return false;
}

void TreeVisitorRegistry::Clear()
{
	for(unsigned int i=0;i,Items.size();i++)
		delete Items[i];
	Items.clear();
}

TreeVisitor * TreeVisitorRegistry::GetVisitor(string name){
	TreeVisitorRegistryItem *tmp;
	if((tmp=GetItem(name)) != NULL)
		return tmp->NewFunc();
	return NULL;
}

TreeVisitor * TreeVisitorRegistry::ApplyTree(Tree *tree)
{	
	TreeVisitorRegistryItem *tmp;
	if((tmp=GetItem(tree->name)) != NULL)
		return (tmp->NewFunc())->Apply(tree);
	
	return false;		
}

TreeVisitor * TreeVisitorRegistry::ApplyFile(string filename)
{
	Tree			*myTree		= new Tree();
	TreeVisitor 	*myObject	= NULL;

	myTree->LoadFromXMLFile(filename);
	myObject = TreeVisitorRegistry::ApplyTree(myTree);
	
	delete myTree;
	return myObject;
}


ObjectContainer::ObjectContainer()
:ObjectRegistry(){
}

ObjectContainer::~ObjectContainer(){}


TreeVisitor *ObjectContainer::Apply(Tree * tree){
	int i;
	for(i=0;i<tree->subTree.size();i++)
		Register(TreeVisitorRegistry::ApplyTree(tree->subTree[i]),tree->subTree[i]->data);
	return this;
}
*/

/*
void* Tree::NewSubTree(string val)
{
	return new Tree(val);
}
*/


/*
int Tree::ResetLookUp()
{
	lookupPos=0;
	for(int i=0;i<subTree.size();i++)
		subTree[i]->ResetLookUp();
	return true;
}

pTree	Tree::LookUp(string val,int depth)
{
	pTree	tmp = NULL;
	if(val.compare(name)==0){
		tmp=this;
	}
	else if(depth!=0){
		for(int i=lookupPos;i<subTree.size();i++)
			if((tmp=subTree[i]->LookUp(val,depth-1))!=NULL){
				lookupPos = i+1;
				return tmp;
			}
		for(i=0;i<lookupPos;i++)
			if((tmp=subTree[i]->LookUp(val,depth-1))!=NULL){
				lookupPos = i+1;
				return tmp;
			}
		if(lookupPos>=subTree.size()) 
			lookupPos = 0;
	}
	return tmp;			
}

pTree	Tree::LookUpData(string val,int depth)
{
	pTree	tmp = NULL;

	if(val.compare(data)==0)
		tmp=this;
	else if(depth!=0){
		for(int i=lookupPos;i<subTree.size();i++)
			if((tmp=subTree[i]->LookUpData(val,depth-1))!=NULL){
				lookupPos = i+1;
				return tmp;		
			}
		for(i=0;i<lookupPos;i++)
			if((tmp=subTree[i]->LookUpData(val,depth-1))!=NULL){
				lookupPos = i+1;
				return tmp;		
			}
		if(lookupPos>=subTree.size()) 
			lookupPos = 0;
	}
	return tmp;			
}

pTree	Tree::LookUpData(string val,string type,int depth)
{
	pTree	tmp = NULL;

	if((val.compare(data)==0)&&(type.compare(name)==0))
		tmp=this;
	else if(depth!=0){
		for(int i=lookupPos;i<subTree.size();i++)
			if((tmp=subTree[i]->LookUpData(val,type,depth-1))!=NULL){
				lookupPos = i+1;
				return tmp;		
			}
		for(i=0;i<lookupPos;i++)
			if((tmp=subTree[i]->LookUpData(val,type,depth-1))!=NULL){
				lookupPos = i+1;
				return tmp;		
			}
		if(lookupPos>=subTree.size()) 
			lookupPos = 0;
	}
	return tmp;			
}
*/

