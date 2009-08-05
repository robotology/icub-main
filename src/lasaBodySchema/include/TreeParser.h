// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
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
// sXMLParser.h: interface for the sXMLParser class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __TREE_PARSER_H__
#define __TREE_PARSER_H__

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;


#define TREE_START_TAG   0
#define TREE_DATA        1
#define TREE_STOP_TAG    2
#define TREE_EOF         3

#define TREE_OK          1
#define TREE_ERROR       0


class   Tree;
typedef Tree *pTree;
typedef vector<pTree> Tree_List;


class Tree
{
public:
	//static string	CurrentPath;

protected:
	string        m_Name;
	string        m_Data;
	Tree_List		  m_SubTree;

//private:
	//int				lookupPos;

public:
	Tree();
	Tree(string newName);
	Tree(string newName, string newData);
  virtual ~Tree();

  void      Clear();
  pTree     Clone();

  int       AddSubTree(pTree newSubTree);
  int       DelSubTree(pTree oldSubTree);

  string    GetName();
  void      SetName(string newName);
  string    GetData();
  void      SetData(string newData);


  pTree     Find(string name);
  pTree     Find(string name, string data);
  string    FindData(string name);


  double   ToDouble();
  float	   ToFloat();
  int	   ToInt();
  bool			ToBool();
  string    ToString();
  

  
  int				LoadFromFile(string filename);
  int				SaveToFile(string filename);
  void			Print(unsigned int indent = 0);
  
  Tree_List *GetSubTrees();

private:
  int				LoadFromStream (ifstream & file);
  int				SaveToStream   (ofstream & file, unsigned int indent = 0);
  int				GetNextTag     (ifstream & file, string & str);
};



#endif
