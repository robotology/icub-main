#ifndef __TTREE_H__
#define __TTREE_H__

#include <vector>
using namespace std;


template<class T> class TTree
{
private:
  static TTree    UNDEF;
  
protected:
  vector<TTree*>  mSubTrees;
  TTree*          mParent;
  T              *mData;
  
public:
          TTree(TTree* mParent = NULL);
          TTree(const TTree &tree, TTree* mParent = NULL);
  virtual ~TTree();
    

  virtual void    Clone(const TTree &tree);

          bool    operator == (const TTree &tree) const;
          bool    operator != (const TTree &tree) const;

          void    AddSubTree(TTree *subTree);
          void    AddSubTree(const TTree &subTree);          
          void    DelSubTree(TTree *subTree); 
  
          TTree&  GetSubTree(unsigned int no);
          TTree*  GetSubTreePtr(unsigned int no);
          unsigned int     GetSize();
          TTree&  Find(T& dataToFind);
          TTree*  FindPtr(T* dataToFind);
          TTree*  GetParent();
          
          T&      GetData();
          T*      GetDataPtr();
          
          unsigned int     GetTreeSize();
          
          int     CreateIndex(int *parentList, T** dataList, int maxSize, int parentIndex=-1, int currentOffset=0);

          void    Clear();
protected:          
  virtual void     Free();
};

template<class T> TTree<T> TTree<T>::UNDEF;

template<class T> TTree<T>::TTree(TTree* mParent){
  mData = NULL;
  this->mParent = mParent;
  Free();
  mData = new T();
}

template<class T> TTree<T>::TTree(const TTree<T> &tree, TTree* mParent){
  mData = NULL;
  Free();
  Clone(tree);  
  this->mParent=mParent;
}

template<class T> TTree<T>::~TTree(){
  Free();
}

template<class T> void TTree<T>::Free(){
  for(unsigned int i=0;i<mSubTrees.size();i++)
    if(mSubTrees[i]!=NULL) delete mSubTrees[i];
  mSubTrees.clear();
  if(mData!=NULL) delete mData;
  mData = NULL;  
  mParent = NULL;
}

template<class T> void TTree<T>::Clear(){
  for(unsigned int i=0;i<mSubTrees.size();i++)
    if(mSubTrees[i]!=NULL) delete mSubTrees[i];
  mSubTrees.clear();
}

template<class T> void TTree<T>::Clone(const TTree<T> &tree){
  Free();
  mData = new T(*tree.mData);
  mParent = tree.mParent;  
  for(unsigned int i=0;i<tree.mSubTrees.size();i++)
    if(tree.mSubTrees[i]!=NULL)
       mSubTrees.push_back(new TTree<T>(*tree.mSubTrees[i]));  
}

template<class T> bool TTree<T>::operator == (const TTree &tree) const{
  return (this==&tree);
}

template<class T> bool TTree<T>::operator != (const TTree &tree) const{
  return (this!=&tree);
}

template<class T> void TTree<T>::AddSubTree(TTree<T> *subTree){  
  if(subTree!=NULL){
    subTree->mParent = this;
    mSubTrees.push_back(subTree);
  }
}

template<class T> void TTree<T>::AddSubTree(const TTree<T> &subTree){
  mSubTrees.push_back(new TTree<T>(subTree,this));
}


template<class T> void TTree<T>::DelSubTree(TTree<T> *subTree){
  for(unsigned int i=0;i<mSubTrees.size();i++){
    if(mSubTrees[i]==subTree){
      mSubTrees.erase(mSubTrees.begin()+i);
      delete subTree;
      break;
    }
  }
} 
  
template<class T> TTree<T>& TTree<T>::GetSubTree(unsigned int no){
  if(no<mSubTrees.size())
    return *mSubTrees[no];
  return UNDEF; 
}

template<class T> TTree<T>*  TTree<T>::GetSubTreePtr(unsigned int no){
  if(no<mSubTrees.size())
    return mSubTrees[no];
  return NULL;   
}

template<class T> TTree<T>& TTree<T>::Find(T& dataToFind){
  if(*mData==dataToFind)
      return *this;
  for(unsigned int i=0;i<mSubTrees.size();i++){
    TTree& result=UNDEF;
    if((result=mSubTrees[i]->Find(dataToFind))!=UNDEF)
      return result;
  }
  return UNDEF;    
}

template<class T> TTree<T>* TTree<T>::FindPtr(T* dataToFind){
  if(*mData==*dataToFind)
      return    this;
  for(unsigned int i=0;i<mSubTrees.size();i++){
    TTree<T>* result=NULL;
    if((result=mSubTrees[i]->FindPtr(dataToFind))!=NULL)
      return result;
  }
  return NULL;    
}

template<class T> TTree<T>* TTree<T>::GetParent(){
  return mParent;  
}


template<class T> T& TTree<T>::GetData(){  
  return *mData;  
}

template<class T> T*  TTree<T>::GetDataPtr(){  
  return mData;  
}

template<class T> unsigned int TTree<T>::GetSize(){
  return mSubTrees.size(); 
}

template<class T> unsigned int TTree<T>::GetTreeSize(){
  unsigned int size = 1;
  for(unsigned int i=0;i<mSubTrees.size();i++){
    size += mSubTrees[i]->GetTreeSize();
  }
  return size; 
}

template<class T> int TTree<T>::CreateIndex(int *parentList, T** dataList, int maxSize, int parentIndex, int currentOffset){
  if((maxSize<=0)||(parentList==NULL)||(dataList==NULL))
    return 0;
  if(parentIndex<-1) parentIndex=-1;
  
  
  *parentList = parentIndex;
  *dataList   = mData;
  int selfId  = parentIndex+currentOffset+1;
  unsigned int indexCounter = 1;
  for(unsigned int i=0;i<mSubTrees.size();i++){    
    indexCounter += mSubTrees[i]->CreateIndex(parentList+indexCounter,
                                             dataList+indexCounter,
                                             maxSize-indexCounter,
                                             selfId,
                                             indexCounter-1);
  }
  return indexCounter;
}



#endif 
