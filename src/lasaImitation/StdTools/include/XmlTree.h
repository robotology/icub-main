#ifndef __XMLTREE_H__
#define __XMLTREE_H__

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <stdarg.h>

using namespace std;


class   XmlTree;
typedef XmlTree *pXmlTree;
typedef vector<pXmlTree> XmlTreeList;
typedef XmlTreeList *pXmlTreeList;



/**
 * \class XmlTree
 * 
 * 
 * \brief A pseudo XML Parser within tree-based storage structure
 * 
 * This class can be used to read and parse pseudo xml files. As a pseudo XML format,
 * the following is accepted: <br> <br> 
 * <Tree_Name0> Data0<br>
 *   <Tree_Name1_1> Data1_1a Data1_1b ... <br>
 *   </Tree_Name1_1> <br>
 *   <Tree_Name1_2> Data1_2 <br>
 *     <Tree_Name1_2_1 Param0="Value0"> Data1_2_1 <br>
 *     </Tree_Name1_2_1> <br>
 *   </Tree_Name1_2> <br>
 * </Tree_Name0> <br>
 * <br>
 * As con be seen, a tree is basically defined by a name (without spaces),. Its data consists of a 
 * series of strings and the tree has eventually sub subtrees and some parameters.
 *  
 */
class XmlTree
{

protected:
	string        m_Name;
	string        m_Data;
  void         *m_ArrayPtr;
  int           m_ArraySize;
  int           m_ArrayTypeSize;

	XmlTreeList		m_SubTrees;
  XmlTreeList   m_Params;
  
  string        m_BasePath;

public:
  /// Empty constructor
	XmlTree();
  /// Create an empty tree with a given name 
	XmlTree(string newName);
  /// Create an empty tree with a given name and data
	XmlTree(string newName, string newData);
  /// Create an empty tree with a given name, data and parameters
  XmlTree(string newName, string newData, string params);
  /// Create an empty tree with a given name, data and a predefined number of subtrees.
  XmlTree(string newName, string newData, int nbSubTree, ... );
  /// Create an empty tree with a given name, data, parametrers, and a predefined number of subtrees.
  XmlTree(string newName, string newData, string params, int nbSubTree, ... );

  /// Destructor
  virtual ~XmlTree();

  /// Clear the tree
  void      Clear();
  
  /// Return a copy of the tree (with subtrees)
  pXmlTree  Clone();
  /// Make a copy of a given tree into self
  void      Clone(pXmlTree src);

  /// Set the given tree as a subtree (no copy)
	int				AddSubTree(pXmlTree newSubTree);
  /// Remove and delete the given tree of the list of subtrees (if it exists)
  int       DelSubTree(pXmlTree oldSubTree);

  /// Return the name of the tree
  string    GetName();
  /// Set the name of the tree
  void      SetName(string newName);

  /// Return the data of the tree
  string    GetData();
  /// Set the data of the tree  
  void      SetData(string newData);
  /// Append the given string at the end of current data  
  void      AppendToData(string newData);

  /// Return the parameters of the tree as a tree  
  pXmlTreeList GetParams();
  /// Return the parameters of the tree as a string
  string    GetParamsString();
  /// Set the parameters of the tree using a tree  
  void      SetParams(pXmlTreeList params);
  /// Set the parameters of the tree using a string
  void      SetParamsString(string params);
  
  /// Return the value of a given parameter name
  string    GetParamValue(string param);
  /// Set the value of a given parameter name
  void      SetParamValue(string param, string value);

  /// Return the list of the subtrees 
  pXmlTreeList GetSubTrees();

  /**
   * \brief Return the subtree having a given name. Return NULL on failure. 
   * If several subtrees have the same name, the second parameter tells which 
   * of them to return. Nested subtrees can also be found using as a first parameter:
   * <br> <br> Name1.Name1_1.Name_1_1_1
   */
  pXmlTree  Find(string name, int no = 0);

  /**
   * \brief Return the subtree having a given name and data. 
   * If several subtrees have the same name, the second parameter tells which 
   * of them to return.
   */
  pXmlTree  Find(string name, string data, int no = 0);
  
  /// Return the data corresponding to the subtree with a given name
  string    FindData(string name);

  /// Replace a given data string in the whole tree
  void      FindAndReplace(string data2find, string data2replace);

  /// Replace and fill the given tree in the tree (according to the name string)
  void      Fill(pXmlTree newTree);

  /// Convert the data to a double
	double		ToDouble();
  /// Convert the data to a float
	float			ToFloat();
  /// Convert the data to an integer
	int				ToInt();
  /// Convert the data to a boolean (0="FALSE"="false"=false, 1="TRUE"="true"=true)
	bool			ToBool();
  /// Return the data
  string    ToString();

  /// Find the subtree with the given name and return its data as a double. If not found, return the default value.
  double    Get(string name, double def);
  /// Find the subtree with the given name and return its data as a float. If not found, return the default value.
  float     Get(string name, float  def);
  /// Find the subtree with the given name and return its data as an int. If not found, return the default value.
  int       Get(string name, int    def);
  /// Find the subtree with the given name and return its data as a bool. If not found, return the default value.
  bool      Get(string name, bool   def);
  /// Find the subtree with the given name and return its data as a string. If not found, return the default value.
  string    Get(string name, string def);

   

  /// Find the subtree with the given name and return its data as a double. If not found, create the tree with the default value.
  double    CGet(string name, double def);
  /// Find the subtree with the given name and return its data as a float. If not found, create the tree with the default value.
  float     CGet(string name, float  def);
  /// Find the subtree with the given name and return its data as an int. If not found, create the tree with the default value.
  int       CGet(string name, int    def);
  /// Find the subtree with the given name and return its data as a bool. If not found, create the tree with the default value.
  bool      CGet(string name, bool   def);
  /// Find the subtree with the given name and return its data as a string. If not found, create the tree with the default value.
  string    CGet(string name, string def);

  /// Set the subtree with the given name with the given value
  void      Set(string name, double def);
  /// Set the subtree with the given name with the given value
  void      Set(string name, float  def);
  /// Set the subtree with the given name with the given value
  void      Set(string name, int    def);
  /// Set the subtree with the given name with the given value
  void      Set(string name, bool   def);
  /// Set the subtree with the given name with the given value
  void      Set(string name, string def);

  /// Return a pointer to an array of float (its size must be known in advance)
  int       GetArray(string name, float** array);
  /// Return a pointer to an array of float (its size must be known in advance)
  int       GetArray(string name, double** array);

  /// Return a pointer to an array of double. The size of it can be found using the second parameter.
  double   *ToArrayDouble(int *size = NULL);
  /// Return a pointer to an array of float. The size of it can be found using the second parameter.
  float    *ToArrayFloat(int *size = NULL);
  /// Return a pointer to an array of int. The size of it can be found using the second parameter.
  int      *ToArrayInt(int *size = NULL);
  /// Return a pointer to an array of bool. The size of it can be found using the second parameter.
  bool     *ToArrayBool(int *size = NULL);
  
  /// Return a pointer to any previousely set array. (Use at your own risk)
  void     *GetArray();
  /// Return the size of any previousely set array. (Use at your own risk)
  int       GetArraySize();
  /// Transpose the data of the array 
  void     *TransposeArray(int rowSize);
  /// Don't touch this...
  void      SetArraySize(int size, int itemSize);

  /// Load the tree from a file (1 on success)
  int				LoadFromFile(string filename);
  /// Load the tree to a file
	int				SaveToFile(string filename);

  /// Get the base path to source file (if any)
  string    GetBasePath(); 

  /// Print the tree to stdout
	void			Print(int indent = 0);


private:
	int				LoadFromStream (ifstream & file, string & currStr, string currPath);
	int				SaveToStream   (ofstream & file, int indent = 0);
	int				GetNextTag     (ifstream & file, string & str, string & params, string &currStr);
  int       GetNextString  (ifstream & file, string & str, string &currStr);

};



#endif 

