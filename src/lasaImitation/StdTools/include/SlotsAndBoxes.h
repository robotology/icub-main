#ifndef SLOTSANDBOXES_H_
#define SLOTSANDBOXES_H_

#include <string>
#include <iostream>
using namespace std;

#include "BaseObject.h"
#include "XmlTree.h"


BASEOBJECT_TOPCLASS(IOData)
public:
  enum ConnectionError{
    IODATA_ERR_NONE=0,
    IODATA_ERR_BADTYPES,
    IODATA_ERR_NULLDATA,
    IODATA_ERR_NULLINPUTOBJPTR,
    IODATA_ERR_NULLOUTPUTOBJ        
  };
  enum Mode{
    IODATA_NONE = 0,
    IODATA_INPUT,
    IODATA_OUTPUT  
  };
protected:
  void*   mData;
  void**  mDataPtr;
public:
          IOData();
  virtual ~IOData();
  
  virtual bool  Serialize(void* dataPtr, int maxLength);
  virtual bool  UnSerialize(void* dataPtr, int length);
  virtual int   GetSerializeSize();
  virtual void  CreateInstance(IOData::Mode mode);
  virtual string ToString();

  static ConnectionError Connect(IOData *output, IOData* input);
  static ConnectionError Disconnect(IOData* input);

  void*   GetData();
  void**  GetDataPtr();

};



template<typename T> BASEOBJECT_CLASS(IOTemplate, IOData)
protected:
  bool    bInstanceObj;
  bool    bInstanceObjPtr;
public:
          IOTemplate(IOData::Mode mode = IODATA_NONE);
          IOTemplate(T *object);
          IOTemplate(T **object);
  virtual ~IOTemplate();    

  virtual bool    Serialize(void* dataPtr, int maxLength);
  virtual bool    UnSerialize(void* dataPtr, int length);
  virtual int     GetSerializeSize();
  virtual void    CreateInstance(IOData::Mode mode);
  virtual string  ToString();
        
          bool    BasicSerialize(void* dataPtr, int maxLength);
          bool    BasicUnSerialize(void* dataPtr, int length);
          int     GetBasicSerializeSize();
          string  BasicToString();
  
          T*    Get();
          void  Set(T *object);
          void  SetPtr(T **object);
          
};



BASEOBJECT_TEMPLATEDEF(template<typename T>, IOTemplate<T>);
template<typename T> IOTemplate<T>::IOTemplate(IOData::Mode mode):IOData(){
  bInstanceObj    = false;
  bInstanceObjPtr = false;  
  CreateInstance(mode);  
}
template<typename T> IOTemplate<T>::IOTemplate(T *object):IOData() {mData    = (void*)object;  bInstanceObj = false; bInstanceObjPtr = false;}
template<typename T> IOTemplate<T>::IOTemplate(T **object):IOData(){mDataPtr = (void**)object; bInstanceObj = false; bInstanceObjPtr = false;}
template<typename T> IOTemplate<T>::~IOTemplate(){
  if(bInstanceObj)    delete ((T*)mData);
  if(bInstanceObjPtr) delete ((T**)mDataPtr);
}
template<typename T> void  IOTemplate<T>::CreateInstance(IOData::Mode mode){
  if(bInstanceObj){    delete ((T*)mData);     mData    = NULL;}
  if(bInstanceObjPtr){ delete ((T**)mDataPtr); mDataPtr = NULL;}

  switch(mode){
  case IOData::IODATA_NONE:
    bInstanceObj    = false;
    bInstanceObjPtr = false;  
  break;
  case IOData::IODATA_INPUT:  
    bInstanceObj    = false;
    bInstanceObjPtr = true;
    mDataPtr        = (void**) new (T*)();
    break;
  case IOData::IODATA_OUTPUT:
    bInstanceObj    = true;
    bInstanceObjPtr = false;
    mData           = (void*) new T();
    break;
  }    
}
template<typename T> T*   IOTemplate<T>::Get(){return (T*)mData;}
template<typename T> void IOTemplate<T>::Set(T *object){mData = (void*)object;}
template<typename T> void IOTemplate<T>::SetPtr(T **object){mDataPtr = (void**)object;}
template<typename T> bool  IOTemplate<T>::Serialize(void* dataPtr, int maxLength){return false;}
template<typename T> bool  IOTemplate<T>::UnSerialize(void* dataPtr, int length){return false;}
template<typename T> int   IOTemplate<T>::GetSerializeSize(){return 0;}
template<typename T> string IOTemplate<T>::ToString(){ return "";}
template<typename T> bool  IOTemplate<T>::BasicSerialize(void* dataPtr, int maxLength){
  if(dataPtr==NULL) return false;
  T *obj; if((obj=(T*)GetData())==NULL) return false;      
  if(maxLength < int(sizeof(T))) return false;
  *((T*)dataPtr) = *obj;
  return true;    
}
template<typename T> bool  IOTemplate<T>::BasicUnSerialize(void* dataPtr, int length){
  if(dataPtr==NULL) return false;    
  T *obj; if((obj=(T*)GetData())==NULL) return false;
  if(length<int(sizeof(T))) return false;
  *obj = *((T*)dataPtr);
  return true;
}
template<typename T> int   IOTemplate<T>::GetBasicSerializeSize(){
  return int(sizeof(T));
}
template<typename T> string IOTemplate<T>::BasicToString(){
  T *obj; if((obj=(T*)GetData())==NULL) return "";      
  ostringstream ss;
  ss << *obj;
  return ss.str();;    
}




BASEOBJECT_TOPCLASS(IOSlot)
protected:
  string          mName;
  IOData         *mData;
public:
          IOSlot();
  virtual ~IOSlot();
  
  string  GetName();
  void    SetName(string name);
  IOData* GetData();
  void    SetData(IOData *data);
};


class   InputSlot;
typedef InputSlot *pInputSlot;
typedef vector<pInputSlot> InputSlotsList;

class   OutputSlot;
typedef OutputSlot *pOutputSlot;
typedef vector<pOutputSlot> OutputSlotsList;


// **************************************************************************
// **************************************************************************
// **************************************************************************

BASEOBJECT_CLASS(OutputSlot, IOSlot)
  
  friend class InputSlot;

protected:
  InputSlotsList    mConnectedTo;
  pOutputSlot       mMappedFrom;
  OutputSlotsList   mMappedTo;  

public:
          OutputSlot();
  virtual ~OutputSlot();

          pOutputSlot               GetMappedFrom();

          IOData::ConnectionError   Reconnect();   

          IOData::ConnectionError   ConnectTo       (InputSlot *slot);
          IOData::ConnectionError   DisconnectTo    (InputSlot *slot);

          IOData::ConnectionError   MapTo           (OutputSlot *slot);
          IOData::ConnectionError   UnmapTo         (OutputSlot *slot);

          IOData::ConnectionError   ClearAll        ();
  
protected:
          IOData::ConnectionError   MapFrom         (OutputSlot *slot);
          IOData::ConnectionError   UnmapFrom       (OutputSlot *slot);
          
          pOutputSlot               GetMapOrigin    ();
          IOData::ConnectionError   Spread          (OutputSlot *slot);
          
public:
          void    Print();

};

// **************************************************************************
// **************************************************************************
// **************************************************************************

BASEOBJECT_CLASS(InputSlot, IOSlot)
  friend class OutputSlot;

protected:
  pOutputSlot      mConnectedFrom;  
  pInputSlot       mMappedFrom;
  InputSlotsList   mMappedTo;
  
  pOutputSlot      mHiddenConnectedFrom;
  
public:
          InputSlot();
  virtual ~InputSlot();

          pOutputSlot               GetConnectedFrom();
          pInputSlot                GetMappedFrom();
  
          IOData::ConnectionError   MapTo           (InputSlot *slot);
          IOData::ConnectionError   UnmapTo         (InputSlot *slot);
          IOData::ConnectionError   ClearAll        ();
protected:
          IOData::ConnectionError   ConnectFrom     (OutputSlot *slot);
          IOData::ConnectionError   DisconnectFrom  (OutputSlot *slot);

          IOData::ConnectionError   UnmapFrom       (InputSlot *slot);
          IOData::ConnectionError   MapFrom         (InputSlot *slot);
          
          pInputSlot                GetMapOrigin    ();
          IOData::ConnectionError   Spread          (OutputSlot *slot);
public:
          void    Print();
};




// **************************************************************************
// **************************************************************************
// **************************************************************************



class SlotsBox;
typedef SlotsBox *pSlotsBox;
typedef vector<pSlotsBox> SlotsBoxsList;


BASEOBJECT_TOPCLASS(SlotsBox)

public:
  enum BoxType{CIOTYPE_NONE=0,
               CIOTYPE_INPUT,
               CIOTYPE_OUTPUT,
               CIOTYPE_DELAYED};

protected:
  string                  mBoxName;
  BoxType                 mBoxType;
  
  InputSlotsList          mInputSlots;
  OutputSlotsList         mOutputSlots;
  SlotsBoxsList           mBoxes;
  
  int                     mBoxPriority;
  
  bool                    bBoxIsEnabled;
  bool                    bValidUpdateSequence;
  
  vector<int>             mDepsMatrix;
  vector<int>             mOutputBoxes;
  vector<int>             mInputBoxes;
  vector<int>             mDelayedBoxes;
  vector<int>             mUpdateSequence;
  
public:
          SlotsBox();
  virtual ~SlotsBox();

          string                  GetName();
          void                    SetName(string name);

          void                    SetBoxType(string name, BoxType type);

          void                    EnableBox(bool enable=true);
          void                    EnableBox(string name, bool enable=true);

          pInputSlot              FindInputSlot (string name, int *index = NULL);
          pOutputSlot             FindOutputSlot(string name, int *index = NULL);
          pSlotsBox               FindBox       (string name, int *index = NULL);

          int                     FindInputSlot (pInputSlot  slot);
          int                     FindOutputSlot(pOutputSlot slot);
          int                     FindBox       (pSlotsBox   box);
          
          bool                    IsConnectedTo (pSlotsBox box);
          bool                    IsMappingTo   (pSlotsBox box);
          bool                    IsMappedFrom  (pSlotsBox box);
          
          void                    AddInputSlot    (IOData *data,  string name);
          void                    AddOutputSlot   (IOData *data,  string name);
          void                    AddBox          (pSlotsBox box, string name);

          void                    RemoveBox       (string name);
          void                    RemoveInputSlot (string name);
          void                    RemoveOutputSlot(string name);
      
          IOData::ConnectionError Connect        (string outputBoxName,
                                                  string outputBoxSlotName,
                                                  string inputBoxName,                                           
                                                  string inputBoxSlotName);

          IOData::ConnectionError Disconnect     (string outputBoxName,
                                                  string outputBoxSlotName,
                                                  string inputBoxName,                                           
                                                  string inputBoxSlotName);
                                          
          IOData::ConnectionError MapInputSlot   (string inputSlotName, 
                                                  string inputBoxName,
                                                  string inputBoxSlotName);

          IOData::ConnectionError UnmapInputSlot (string inputSlotName, 
                                                  string inputBoxName,
                                                  string inputBoxSlotName);

          IOData::ConnectionError MapOutputSlot  (string outputBoxName,
                                                  string outputBoxSlotName,
                                                  string outputSlotName);

          IOData::ConnectionError UnmapOutputSlot(string outputBoxName,
                                                  string outputBoxSlotName,
                                                  string outputSlotName);
       
       
          void    Print();
          int     BuildUpdateSequence();

          void    UpdateBox(int level=0);
          void    SetPriority(int priority, bool recursive=false);
          
           
  virtual void    PreUpdate(int level=0);
  virtual void    Update(int level=0);

  virtual void    Init(pXmlTree config = NULL);
  virtual void    SetData(string dataKey, void* data);
  virtual void*   GetData(string dataKey);
  virtual void    Free();
           
protected:
          void  UpdateDependencies();
          int   ResolveDependencies(vector<int> &deps, vector<int> &visited, vector<int> &updated,vector<int> &sequence);    
};




/* In a slot box:
 * Vector *vIn;
 * Vector  vOut;
 * AddInputSlot(new IOVector(vIn),"name"));
 * AddOutputSlot(new IOVector(&vOut),"name"));
 * AddInputSlot((new IOVector())->SetInput(vIn),"name"));
 * AddOutputSlot((new IOVector())->SetOutput(&vOut),"name"));
 */


















#endif /*SLOTSANDBOXES_H_*/
