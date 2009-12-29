#include "SlotsAndBoxes.h"

//#define MDEBUG(x) {x}
#define MDEBUG(x)
#define MWARN(x) {x}

///////////////////////////////////////////////////////

BASEOBJECT_TOPDEF(IOData);
IOData::IOData(){
  mData           = NULL;
  mDataPtr        = NULL;
}
IOData::~IOData(){
}

bool  IOData::Serialize(void* dataPtr, int maxLength){
  return false;
}
bool  IOData::UnSerialize(void* dataPtr, int length){
  return false;  
}
int   IOData::GetSerializeSize(){
  return 0;
}
void  IOData::CreateInstance(IOData::Mode mode){}
string IOData::ToString(){return "";}

void* IOData::GetData(){
  if(mDataPtr!=NULL){
    return *mDataPtr;
  }else if(mData!=NULL){
    return mData;
  }else{
    return NULL;  
  }
}
void**  IOData::GetDataPtr(){
  return mDataPtr;
}


IOData::ConnectionError IOData::Connect(IOData *output, IOData* input){
  if((input==NULL)||(output==NULL))
    return IODATA_ERR_NULLDATA;
    
  if(input->GetObjectType()!=output->GetObjectType()){
    //cout << input->GetClassName()<<"("<<input->GetObjectType()<<") -> "<<output->GetClassName()<<"("<<output->GetObjectType()<<")"<< endl;
    //cout << input->ToString()<< " != "<<output->ToString()<<endl;
    //cout << typeid(input).name()<< " --- " << typeid(output).name()<<endl;
    return IODATA_ERR_BADTYPES;
  }
  
  if(input->mDataPtr==NULL)
    return IODATA_ERR_NULLINPUTOBJPTR;

  if(output->mData==NULL)
    return IODATA_ERR_NULLOUTPUTOBJ;
  
  *(input->mDataPtr) = output->mData;
  return IODATA_ERR_NONE;
}

IOData::ConnectionError IOData::Disconnect(IOData* input){
  if(input==NULL)
    return IODATA_ERR_NULLDATA;  
  
  *(input->mDataPtr) = NULL;  
  return IODATA_ERR_NONE;
}
///////////////////////////////////////////////////////

BASEOBJECT_TOPDEF(IOSlot);
IOSlot::IOSlot(){
  mName         = "";
  mData         = NULL;
}
IOSlot::~IOSlot(){
  if(mData!=NULL) delete mData;
  mData       = NULL;
}
string IOSlot::GetName(){
  return mName;  
}
void  IOSlot::SetName(string name){
  mName = name;  
}
IOData *IOSlot::GetData(){
  return mData;  
}
void  IOSlot::SetData(IOData *data){
  if(mData!=NULL) delete mData;
  mData         = data;
}


// **************************************************************************
// **************************************************************************
// **************************************************************************

BASEOBJECT_DEF(OutputSlot);
OutputSlot::OutputSlot():IOSlot(){
  mConnectedTo.clear();
  mMappedFrom = NULL;
  mMappedTo.clear();  
  
}
OutputSlot::~OutputSlot(){
  ClearAll();
  mConnectedTo.clear();
  mMappedFrom = NULL;
  mMappedTo.clear();  
}

BASEOBJECT_DEF(InputSlot);
InputSlot::InputSlot():IOSlot(){
  mConnectedFrom = NULL;
  mMappedFrom    = NULL;
  mMappedTo.clear();
  mHiddenConnectedFrom = NULL;
}
InputSlot::~InputSlot(){
  ClearAll();
  mConnectedFrom = NULL;
  mMappedFrom    = NULL;
  mMappedTo.clear();
  mHiddenConnectedFrom = NULL;
}

// **************************************************************************


IOData::ConnectionError   OutputSlot::ConnectTo(InputSlot *slot){
  if(slot!=NULL){
    MDEBUG(cout <<"DEBUG: "<<GetName()<<" connects to "<<slot->GetName()<<endl;);  
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE; 

    // Already connected flag (uselful for reconnect)
    bool bConnected=false;        
    // Check if already connected
    for(unsigned int i=0;i<mConnectedTo.size();i++){
      if(mConnectedTo[i] == slot) {bConnected = true; break;}//return IOData::IODATA_ERR_NONE; 
    }
    
    // Connect from the input side
    error = slot->ConnectFrom(this);

    // On success update input list
    if(error==IOData::IODATA_ERR_NONE){
      if(!bConnected) // Only if not already connected
        mConnectedTo.push_back(slot);
      
      // Spread the connetivity
      slot->Spread(GetMapOrigin());            
    }

    return error;
  }
  return IOData::IODATA_ERR_NULLDATA;
}    

IOData::ConnectionError InputSlot::ConnectFrom(OutputSlot *slot){
  if(slot!=NULL){    
    MDEBUG(cout <<"DEBUG: "<<GetName()<<" connects from "<<slot->GetName()<<endl;);
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE; 
    
    // Already connected
    if(mConnectedFrom!=NULL){
      // Same as requested?      
      if(mConnectedFrom==slot){
        //return IOData::IODATA_ERR_NONE;
      }else{ 
        // Otherwise disconnect
        mConnectedFrom->DisconnectTo(this);
      }
    } 

    // Unmap input map if one
    if(mMappedFrom!=NULL){
      mMappedFrom->UnmapTo(this);
    }

    // Try to connect
    error = IOData::Connect(slot->GetData(),GetData());
    // On success
    if(error==IOData::IODATA_ERR_NONE){
      // Update the source
      mConnectedFrom = slot;
    }
    return error;
  }
  return IOData::IODATA_ERR_NULLDATA;  
}
// **************************************************************************
IOData::ConnectionError   OutputSlot::Reconnect(){
  for(unsigned int i=0;i<mConnectedTo.size();i++){
     ConnectTo(mConnectedTo[i]);
  }  
  for(unsigned int i=0;i<mMappedTo.size();i++){
     MapTo(mMappedTo[i]);
  }    
  return IOData::IODATA_ERR_NONE;
}   

// **************************************************************************

IOData::ConnectionError OutputSlot::DisconnectTo(InputSlot *slot){
  if(slot!=NULL){
    MDEBUG(cout <<"DEBUG: "<<GetName()<<" disconnects to "<<slot->GetName()<<endl;);
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE; 

    // Check if already connected
    bool bFound = false;
    for(unsigned int i=0;i<mConnectedTo.size();i++){
      if(mConnectedTo[i] == slot){
        // Remove it from the list
        for(unsigned int j=i;j<mConnectedTo.size()-1;j++)
          mConnectedTo[j] = mConnectedTo[j+1];
        mConnectedTo.pop_back();
        bFound = true;
        break;  
      } 
    }
    if(bFound){
      // if it was actually connected, disconnect
      error = slot->DisconnectFrom(this);
      
      // Spread the null
      slot->Spread(NULL);
    }else{
      error = IOData::IODATA_ERR_NONE;
    }      
    return error;    
  }
  return IOData::IODATA_ERR_NULLDATA;  
}

IOData::ConnectionError InputSlot::DisconnectFrom(OutputSlot *slot){
  if(slot!=NULL){
    
    MDEBUG(cout <<"DEBUG: "<< GetName()<<" disconnects from "<<slot->GetName()<<endl;);
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE; 

    // Disconnect the input data
    error = IOData::Disconnect(GetData());
    // Clear the reference
    mConnectedFrom = NULL;
    
    return error; 
  }  
  return IOData::IODATA_ERR_NULLDATA;  
}

// **************************************************************************

IOData::ConnectionError InputSlot::MapTo(InputSlot *slot){
  if(slot!=NULL){    
    MDEBUG(cout <<"DEBUG: "<<GetName()<<" maps to "<<slot->GetName()<<endl;);
    
    // Check if already mapped
    for(unsigned int i=0;i<mMappedTo.size();i++){
      if(mMappedTo[i] == slot) return IOData::IODATA_ERR_NONE; 
    }
    
    // Map from the input side
    IOData::ConnectionError error = slot->MapFrom(this);

    // On success
    if(error==IOData::IODATA_ERR_NONE){
      // Update the mapping list
      mMappedTo.push_back(slot);
      
      Spread(GetMapOrigin()->mHiddenConnectedFrom);
    }    
    return error;
  }  
  return IOData::IODATA_ERR_NULLDATA;    
}

IOData::ConnectionError   InputSlot::MapFrom(InputSlot *slot){
  if(slot!=NULL){    
    MDEBUG(cout <<"DEBUG: "<<GetName()<<" maps from "<<slot->GetName()<<endl;);
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE;
    
    // Already mapped
    if(mMappedFrom!=NULL){
      // Same as requested?
      if(mMappedFrom == slot) return IOData::IODATA_ERR_NONE;
      // Otherwise unmap
      mMappedFrom->UnmapTo(this);
    } 

    // Disconnect if any output is connected
    if(mConnectedFrom!=NULL){
      mConnectedFrom->DisconnectTo(this);
    }
     
    // check types
    if(GetData()->GetObjectType() != slot->GetData()->GetObjectType())
      return IOData::IODATA_ERR_BADTYPES;
    // Then map
    mMappedFrom = slot;

    return error;
  }  
  return IOData::IODATA_ERR_NULLDATA;    
}

// **************************************************************************

IOData::ConnectionError InputSlot::UnmapTo(InputSlot *slot){
  if(slot!=NULL){
    MDEBUG(cout <<"DEBUG: "<<GetName()<<" unmaps to "<<slot->GetName()<<endl;);
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE;

    // Check if already mapped
    bool bFound = false;
    for(unsigned int i=0;i<mMappedTo.size();i++){
      if(mMappedTo[i] == slot){
        // Remove from the list
        for(unsigned int j=i;j<mMappedTo.size()-1;j++)
          mMappedTo[j] = mMappedTo[j+1];        
        mMappedTo.pop_back();
        bFound = true;
        break;  
      } 
    }
    if(bFound){
      error = slot->UnmapFrom(this);
      
      // Spread the null word...
      Spread(NULL);
    }else{
      error = IOData::IODATA_ERR_NONE;
    }
    return error;      
  }    
  return IOData::IODATA_ERR_NULLDATA;    
}

IOData::ConnectionError InputSlot::UnmapFrom(InputSlot *slot){
  if(slot!=NULL){
    MDEBUG(cout <<"DEBUG: "<<GetName()<<" unmaps from "<<slot->GetName()<<endl;);
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE;

    // Unmap
    mMappedFrom = NULL;
    
    return error; 
  }  
  return IOData::IODATA_ERR_NULLDATA;    
}

IOData::ConnectionError InputSlot::ClearAll(){
  while(mMappedTo.size()>0){
    UnmapTo(mMappedTo[0]);
  }
  /*
  for(unsigned int i=0;i<mMappedTo.size();i++){
    UnmapTo(mMappedTo[i]);
  }*/ 
  if(mMappedFrom!=NULL){
    mMappedFrom->UnmapTo(this);
  }
  if(mConnectedFrom!=NULL){
    mConnectedFrom->DisconnectTo(this);  
  }  
  return IOData::IODATA_ERR_NONE;
}

// **************************************************************************

IOData::ConnectionError   OutputSlot::MapTo     (OutputSlot *slot){
  if(slot!=NULL){    

    IOData::ConnectionError error = IOData::IODATA_ERR_NONE;
    
    // Already connected flag (uselful for reconnect)
    bool bConnected=false;            
    // Check if already mapped
    for(unsigned int i=0;i<mMappedTo.size();i++){
      if(mMappedTo[i] == slot){bConnected=true; break;}//return IOData::IODATA_ERR_NONE; 
    }
    
    // Map from the input side
    error = slot->MapFrom(this);

    // On success
    if(error==IOData::IODATA_ERR_NONE){
      // Update the mapping list
      if(!bConnected)
        mMappedTo.push_back(slot);
      
      // Spread the word...
      Spread(GetMapOrigin());
    }    
  
    return error;
  }  
  return IOData::IODATA_ERR_NULLDATA;  
}

IOData::ConnectionError   OutputSlot::MapFrom   (OutputSlot *slot){
  if(slot!=NULL){    
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE;
    
    // Already mapped
    if(mMappedFrom!=NULL){
      // Same as requested?
      if(mMappedFrom == slot){
        //return IOData::IODATA_ERR_NONE;
      }else{
        // Otherwise unmap
        mMappedFrom->UnmapTo(this);
      }
    } 
     
    // check types
    if(GetData()->GetObjectType() != slot->GetData()->GetObjectType())
      return IOData::IODATA_ERR_BADTYPES;

    mMappedFrom = slot;

    return error;
  }  
  return IOData::IODATA_ERR_NULLDATA;  
}

IOData::ConnectionError   OutputSlot::UnmapTo   (OutputSlot *slot){
  if(slot!=NULL){
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE;

    // Check if already mapped
    bool bFound = false;
    for(unsigned int i=0;i<mMappedTo.size();i++){
      if(mMappedTo[i] == slot){
        // Remove from the list
        for(unsigned int j=i;j<mMappedTo.size()-1;j++)
          mMappedTo[j] = mMappedTo[j+1];        
        mMappedTo.pop_back();
        bFound = true;
        break;  
      } 
    }
    if(bFound){
      error = slot->UnmapFrom(this);

      // Spread the word...
      slot->Spread(slot);

    }else{
      error = IOData::IODATA_ERR_NONE;
    }

    return error;      
  }    
  return IOData::IODATA_ERR_NULLDATA;    
}
IOData::ConnectionError   OutputSlot::UnmapFrom (OutputSlot *slot){
  if(slot!=NULL){
    IOData::ConnectionError error = IOData::IODATA_ERR_NONE;

    // Unmap
    mMappedFrom = NULL;

    return error; 
  }    
  return IOData::IODATA_ERR_NULLDATA;    
}

IOData::ConnectionError OutputSlot::ClearAll(){
  while(mMappedTo.size()>0){
    UnmapTo(mMappedTo[0]);
  }
  /*
  for(unsigned int i=0;i<mMappedTo.size();i++){
    UnmapTo(mMappedTo[i]);
  } */
  while(mConnectedTo.size()>0){
    DisconnectTo(mConnectedTo[0]);
  }
   /*
  for(unsigned int i=0;i<mConnectedTo.size();i++){
    DisconnectTo(mConnectedTo[i]);
  }  */
  if(mMappedFrom!=NULL){
    mMappedFrom->UnmapTo(this);
  }
  return IOData::IODATA_ERR_NONE;
}


pInputSlot  InputSlot::GetMapOrigin(){
  pInputSlot origin = this;
  while(origin->mMappedFrom!=NULL) origin = origin->mMappedFrom;
  return origin;   
}

pOutputSlot OutputSlot::GetMapOrigin(){
  pOutputSlot origin = this;
  while(origin->mMappedFrom!=NULL) origin = origin->mMappedFrom;
  return origin; 
}

IOData::ConnectionError   InputSlot::Spread(OutputSlot *slot){
  IOData::ConnectionError error = IOData::IODATA_ERR_NONE;

  MDEBUG(
  if(slot!=NULL)
    cout <<"DEBUG: "<<GetName()<<" spreads "<<slot->GetName()<<endl;
  else
    cout <<"DEBUG: "<<GetName()<<" spreads NULL"<<endl;
  );
  
  // Self settings
  mHiddenConnectedFrom = slot;
  if(slot==NULL)
    error = IOData::Disconnect(GetData());
  else  
    error = IOData::Connect(slot->GetData(),GetData());
    
  // Spreading
  for(unsigned int i=0;i<mMappedTo.size();i++){
    mMappedTo[i]->Spread(slot);
  }
  
  return error;
}
IOData::ConnectionError OutputSlot::Spread(OutputSlot *slot){
  IOData::ConnectionError error = IOData::IODATA_ERR_NONE;

  MDEBUG(
  if(slot!=NULL)
    cout <<"DEBUG: "<<GetName()<<" spreads "<<slot->GetName()<<endl;
  else
    cout <<"DEBUG: "<<GetName()<<" spreads NULL"<<endl;
  );
  
  // For each connected inputs: spread...
  for(unsigned int i=0;i<mConnectedTo.size();i++){
    mConnectedTo[i]->Spread(slot);
  }
  
  // For each mapped outputs: spread...
  for(unsigned int i=0;i<mMappedTo.size();i++){
    mMappedTo[i]->Spread(slot);
  }

  return error;
}

pOutputSlot InputSlot::GetConnectedFrom(){
  return mConnectedFrom;
}
pInputSlot InputSlot::GetMappedFrom(){
  return mMappedFrom;  
}
pOutputSlot OutputSlot::GetMappedFrom(){
  return mMappedFrom;    
}


void  OutputSlot::Print(){
  cout << "| ("<<(mMappedFrom?mMappedFrom->GetName():"   ")<<") ";
  cout <<  mName<< " -> ";// << (mOutputSlot?mOutputSlot->GetName():"   ");
  cout <<"(";
  for(unsigned int i=0;i<mConnectedTo.size();i++){
    cout << mConnectedTo[i]->GetName()<<" ";  
  }
  cout <<") |";
}

// **************************************************************************
// **************************************************************************
// **************************************************************************



  



void  InputSlot::Print(){
  cout << "| " << (mHiddenConnectedFrom?mHiddenConnectedFrom->GetName():"   ");
  cout << " -> " << mName<<" ("<<(mMappedFrom?mMappedFrom->GetName():"   ")<<")|";
}















BASEOBJECT_TOPDEF(SlotsBox);

SlotsBox::SlotsBox(){
  mBoxName                   = "";
  mBoxType                = CIOTYPE_NONE;

  mInputSlots.clear();
  mOutputSlots.clear();
  mBoxes.clear();

  bValidUpdateSequence    = false;
  bBoxIsEnabled           = true;
  mDepsMatrix.clear();
  mOutputBoxes.clear();
  mInputBoxes.clear();
  mDelayedBoxes.clear();
  mUpdateSequence.clear();
  
  mBoxPriority = 0;    
}

SlotsBox::~SlotsBox(){
  Free();  
}

void SlotsBox::Free(){
  mBoxName                   = "";
  mBoxType                = CIOTYPE_NONE;
  bValidUpdateSequence    = false;

  for(unsigned int i=0;i<mBoxes.size();i++)
    if(mBoxes[i]!=NULL) delete mBoxes[i];
  mBoxes.clear();
  
  for(unsigned int i=0;i<mInputSlots.size();i++)
    if(mInputSlots[i]!=NULL) delete mInputSlots[i];
  mInputSlots.clear();
  
  for(unsigned int i=0;i<mOutputSlots.size();i++)
    if(mOutputSlots[i]!=NULL) delete mOutputSlots[i];
  mOutputSlots.clear();
  
  mDepsMatrix.clear();
  mOutputBoxes.clear();
  mInputBoxes.clear();
  mDelayedBoxes.clear();
  mUpdateSequence.clear();    
} 

string SlotsBox::GetName(){
  return mBoxName;
}
void   SlotsBox::SetName(string name){
  mBoxName = name;  
}
void    SlotsBox::SetData(string dataKey, void* data){
}
void*   SlotsBox::GetData(string dataKey){
  return NULL;  
}

void  SlotsBox::AddInputSlot(IOData *data, string name){
  if(data!=NULL){
    if(FindInputSlot(name)==NULL){
      pInputSlot slot = new InputSlot();
      slot->SetName(name);
      slot->SetData(data);      
      mInputSlots.push_back(slot);
    }      
  }
}

void  SlotsBox::AddOutputSlot(IOData *data, string name){
  if(data!=NULL){
    if(FindOutputSlot(name)==NULL){
      pOutputSlot slot = new OutputSlot();
      slot->SetName(name);
      slot->SetData(data);      
      mOutputSlots.push_back(slot);
    }      
  }
}
void  SlotsBox::AddBox(SlotsBox *box, string name){
  if(box!=NULL){
    if(FindBox(name)==NULL){
      box->SetName(name);
      mBoxes.push_back(box);  
    }  
  }
}

void  SlotsBox::RemoveBox(string name){
  int index = 0;
  if(FindBox(name,&index)==NULL){
    delete mBoxes[index];
    for(int i=index;i<int(mBoxes.size())-1;i++){
      mBoxes[i] = mBoxes[i+1];
    }    
    mBoxes.pop_back();
    bValidUpdateSequence = false;
  }
}

void SlotsBox::RemoveInputSlot (string name){
  int index = 0;
  if(FindInputSlot(name,&index)==NULL){
    delete mInputSlots[index];
    for(int i=index;i<int(mInputSlots.size())-1;i++){
      mInputSlots[i] = mInputSlots[i+1];
    }    
    mInputSlots.pop_back();
    bValidUpdateSequence = false;
  }
}
void SlotsBox::RemoveOutputSlot(string name){
  int index = 0;
  if(FindOutputSlot(name,&index)==NULL){
    delete mOutputSlots[index];
    for(int i=index;i<int(mOutputSlots.size())-1;i++){
      mOutputSlots[i] = mOutputSlots[i+1];
    }    
    mOutputSlots.pop_back();
    bValidUpdateSequence = false;
  }  
}


int SlotsBox::FindInputSlot (pInputSlot      slot){
  if(slot==NULL) return -1;  
  for(unsigned int i=0;i<mInputSlots.size();i++)
    if(mInputSlots[i] == slot) return i;
  return -1;
}
int SlotsBox::FindOutputSlot(pOutputSlot     slot){
  if(slot==NULL) return -1;  
  for(unsigned int i=0;i<mOutputSlots.size();i++)
    if(mOutputSlots[i] == slot) return i;
  return -1;
}
int SlotsBox::FindBox (pSlotsBox box){
  if(box==NULL) return -1;  
  for(unsigned int i=0;i<mBoxes.size();i++)
    if(mBoxes[i] == box) return i;
  return -1;  
}

pInputSlot  SlotsBox::FindInputSlot(string name, int *index){
  for(unsigned int i=0;i<mInputSlots.size();i++){
    if(mInputSlots[i]->GetName()==name){
      if(index!=NULL) *index = i;
      return mInputSlots[i];
    }
  }  
  if(index!=NULL) *index = -1;
  return NULL;  
}

pOutputSlot SlotsBox::FindOutputSlot(string name, int *index){
  for(unsigned int i=0;i<mOutputSlots.size();i++){
    if(mOutputSlots[i]->GetName()==name){
      if(index!=NULL) *index = i;
      return mOutputSlots[i];
    }
  }  
  if(index!=NULL) *index = -1;
  return NULL;
}

pSlotsBox SlotsBox::FindBox(string name, int *index){
  for(unsigned int i=0;i<mBoxes.size();i++){
    if(mBoxes[i]->GetName()==name){
      if(index!=NULL) *index = i;
      return mBoxes[i];
    }
  }  
  if(index!=NULL) *index = -1;
  return NULL;  
}

bool SlotsBox::IsConnectedTo(pSlotsBox box){
  if(box==NULL) return false;
  for(unsigned i=0;i<box->mInputSlots.size();i++){
    if(FindOutputSlot(box->mInputSlots[i]->GetConnectedFrom())>=0)
      return true;
  }
  return false;
}
bool SlotsBox::IsMappingTo(pSlotsBox box){
  if(box==NULL) return false;
  for(unsigned i=0;i<box->mInputSlots.size();i++){
    if(FindInputSlot(box->mInputSlots[i]->GetMappedFrom())>=0)
      return true;
  }
  return false;  
}
bool SlotsBox::IsMappedFrom(pSlotsBox box){
  if(box==NULL) return false;
  for(unsigned i=0;i<mOutputSlots.size();i++){
    if(box->FindOutputSlot(mOutputSlots[i]->GetMappedFrom())>=0)
      return true;
  }
  return false;  
}

IOData::ConnectionError SlotsBox::Connect(string outputBoxName,
                                                string outputBoxSlotName,
                                                string inputBoxName,                                           
                                                string inputBoxSlotName){
  int outIndex = -1;
  int inIndex  = -1;

  pSlotsBox outputBox = FindBox(outputBoxName, &outIndex);
  if(outputBox==NULL){
    MWARN( cout<<"Connection failed ("<<outputBoxName<<")"<<outputBoxSlotName<<"->("<<inputBoxName<<")"<< inputBoxSlotName <<endl;);
    MWARN( cout<<"  ("<<outputBoxName<<") not found"<<endl;);
    return IOData::IODATA_ERR_NULLDATA;
  }

  pOutputSlot     outputSlot      = outputBox->FindOutputSlot(outputBoxSlotName);
  if(outputSlot==NULL){
    MWARN( cout<<"Connection failed ("<<outputBoxName<<")"<<outputBoxSlotName<<"->("<<inputBoxName<<")"<< inputBoxSlotName <<endl;);
    MWARN( cout<<"  ("<<outputBoxSlotName<<") not found"<<endl;);
    MWARN(
      cout <<"  Available output slots are: ("<< outputBoxName <<")";
      for(unsigned int i=0;i<outputBox->mOutputSlots.size();i++){
        cout <<  "<"<<outputBox->mOutputSlots[i]->GetName()<<"> ";
      }
      cout << endl;
    );
    return IOData::IODATA_ERR_NULLDATA;
  }
  
  pSlotsBox inputBox  = FindBox(inputBoxName, &inIndex);
  if(inputBox==NULL){
    MWARN( cout<<"Connection failed ("<<outputBoxName<<")"<<outputBoxSlotName<<"->("<<inputBoxName<<")"<< inputBoxSlotName <<endl;);
    MWARN( cout<<"  ("<<inputBoxName<<") not found"<<endl;);
    return IOData::IODATA_ERR_NULLDATA;
  }

  pInputSlot      inputSlot       = inputBox->FindInputSlot(inputBoxSlotName);
  if(inputSlot==NULL){
    MWARN( cout<<"Connection failed ("<<outputBoxName<<")"<<outputBoxSlotName<<"->("<<inputBoxName<<")"<< inputBoxSlotName <<endl;);
    MWARN( cout<<"  ("<<inputBoxSlotName<<") not found"<<endl;);
    MWARN(
      cout <<"  Available input slots are: ("<< inputBoxName <<")";
      for(unsigned int i=0;i<inputBox->mInputSlots.size();i++){
        cout <<  "<"<<inputBox->mInputSlots[i]->GetName()<<"> ";
      }
      cout << endl;
    );
    return IOData::IODATA_ERR_NULLDATA;
  }

  MDEBUG(
  cout << "***********************************"<<endl;
  cout <<"DEBUG: Box "<<outputSlot->GetName()<<" connects "<<inputSlot->GetName()<<endl;
  cout << "***********************************"<<endl;
  );
  bValidUpdateSequence = false;
  
  IOData::ConnectionError error = outputSlot->ConnectTo(inputSlot);
  MWARN(if(error) cout<<"Connection failed ("<<outputBoxName<<")"<<outputBoxSlotName<<"->("<<inputBoxName<<")"<< inputBoxSlotName <<" Error code: "<<error<<endl;); 
  return error;
}                                

IOData::ConnectionError SlotsBox::Disconnect(string outputBoxName,
                                                   string outputBoxSlotName,
                                                   string inputBoxName,                                           
                                                   string inputBoxSlotName){
  int outIndex = -1;
  int inIndex  = -1;
  
  pSlotsBox outputBox = FindBox(outputBoxName, &outIndex);
  if(outputBox==NULL) return IOData::IODATA_ERR_NULLDATA;

  pOutputSlot     outputSlot      = outputBox->FindOutputSlot(outputBoxSlotName);
  if(outputSlot==NULL)      return IOData::IODATA_ERR_NULLDATA;
  
  pSlotsBox inputBox  = FindBox(inputBoxName, &inIndex);
  if(inputBox==NULL)  return IOData::IODATA_ERR_NULLDATA;

  pInputSlot      inputSlot       = inputBox->FindInputSlot(inputBoxSlotName);
  if(inputSlot==NULL)       return IOData::IODATA_ERR_NULLDATA;
  MDEBUG(
  cout << "***********************************"<<endl;
  cout <<"DEBUG: Box "<<outputSlot->GetName()<<" disconnects "<<inputSlot->GetName()<<endl;
  cout << "***********************************"<<endl;
  );
  bValidUpdateSequence = false;

  IOData::ConnectionError error = outputSlot->DisconnectTo(inputSlot);  
  MWARN(if(error) cout<<"Disconnection failed: "<<error<<endl;); 
  return error;
}                                








IOData::ConnectionError SlotsBox::MapInputSlot(string inputSlotName, 
                                                     string inputBoxName,
                                                     string inputBoxSlotName){

  int index = -1;
  
  pSlotsBox inputBox     = FindBox(inputBoxName,&index);
  if(inputBox==NULL)      return IOData::IODATA_ERR_NULLDATA;

  pInputSlot      inputBoxSlot = inputBox->FindInputSlot(inputBoxSlotName);
  if(inputBoxSlot==NULL)  return IOData::IODATA_ERR_NULLDATA;

  pInputSlot      inputSlot          = FindInputSlot(inputSlotName);
  if(inputSlot==NULL)           return IOData::IODATA_ERR_NULLDATA;
  
  MDEBUG(
  cout << "***********************************"<<endl;
  cout <<"DEBUG: Box "<<inputSlot->GetName()<<" maps to "<<inputBoxSlot->GetName()<<endl;
  cout << "***********************************"<<endl;
  );
  bValidUpdateSequence = false;

  IOData::ConnectionError error = inputSlot->MapTo(inputBoxSlot);
  return error;
}

IOData::ConnectionError SlotsBox::UnmapInputSlot(string inputSlotName, 
                                                       string inputBoxName,
                                                       string inputBoxSlotName){

  int index = -1;
  
  pSlotsBox inputBox     = FindBox(inputBoxName,&index);
  if(inputBox==NULL)      return IOData::IODATA_ERR_NULLDATA;

  pInputSlot      inputBoxSlot = inputBox->FindInputSlot(inputBoxSlotName);
  if(inputBoxSlot==NULL)  return IOData::IODATA_ERR_NULLDATA;

  pInputSlot      inputSlot          = FindInputSlot(inputSlotName);
  if(inputSlot==NULL)           return IOData::IODATA_ERR_NULLDATA;
  
  MDEBUG(
  cout << "***********************************"<<endl;
  cout <<"DEBUG: Box "<<inputSlot->GetName()<<" unmaps to "<<inputBoxSlot->GetName()<<endl;
  cout << "***********************************"<<endl;
  );
  bValidUpdateSequence = false;
  
  IOData::ConnectionError error = inputSlot->UnmapTo(inputBoxSlot);
  return error;
}


IOData::ConnectionError SlotsBox::MapOutputSlot(string outputBoxName,
                                                      string outputBoxSlotName,
                                                      string outputSlotName){
  int index = -1;
  
  pSlotsBox outputBox     = FindBox(outputBoxName,&index);
  if(outputBox==NULL)      return IOData::IODATA_ERR_NULLDATA;

  pOutputSlot      outputBoxSlot = outputBox->FindOutputSlot(outputBoxSlotName);
  if(outputBoxSlot==NULL)  return IOData::IODATA_ERR_NULLDATA;

  pOutputSlot      outputSlot          = FindOutputSlot(outputSlotName);
  if(outputSlot==NULL)           return IOData::IODATA_ERR_NULLDATA;
  
  MDEBUG(
  cout << "***********************************"<<endl;
  cout <<"DEBUG: Box "<<outputBoxSlot->GetName()<<" maps to "<<outputSlot->GetName()<<endl;
  cout << "***********************************"<<endl;
  );
  bValidUpdateSequence = false;
  IOData::ConnectionError error = outputBoxSlot->MapTo(outputSlot);
  return error;
}

IOData::ConnectionError SlotsBox::UnmapOutputSlot(string outputBoxName,
                                                        string outputBoxSlotName,
                                                        string outputSlotName){
  int index = -1;
  
  pSlotsBox outputBox     = FindBox(outputBoxName,&index);
  if(outputBox==NULL)      return IOData::IODATA_ERR_NULLDATA;

  pOutputSlot      outputBoxSlot = outputBox->FindOutputSlot(outputBoxSlotName);
  if(outputBoxSlot==NULL)  return IOData::IODATA_ERR_NULLDATA;

  pOutputSlot      outputSlot          = FindOutputSlot(outputSlotName);
  if(outputSlot==NULL)           return IOData::IODATA_ERR_NULLDATA;
  
  MDEBUG(
  cout << "***********************************"<<endl;
  cout <<"DEBUG: Box "<<outputBoxSlot->GetName()<<" unmaps to "<<outputSlot->GetName()<<endl;
  cout << "***********************************"<<endl;
  );
  bValidUpdateSequence = false;  
  IOData::ConnectionError error = outputBoxSlot->UnmapTo(outputSlot);
  return error;
}


void SlotsBox::Print(){
  for(unsigned int i=0;i<mInputSlots.size();i++){
    mInputSlots[i]->Print();
    cout <<"            ";
  }  
  cout <<endl;
//  for(unsigned int i=0;i<mOutputSlots.size();i++){
//    mOutputSlots[i]->Print();
//    cout <<"            ";
//  } 
//  cout <<endl;
  
  for(unsigned int i=0;i<mBoxes.size();i++){
    mBoxes[i]->Print();
  }

  /*
  */
  /*
  printf("InpCont:\n");
  printf("|");
  for(int i=0;i<mDependenciesMatrixSize;i++){
    printf("%c",(mInputBoxes[i]==0?' ':'X'));
  }
  printf("|\n");
  
  printf("OutCont:\n");
  printf("|");
  for(int i=0;i<mDependenciesMatrixSize;i++){
    printf("%c",(mOutputBoxes[i]==0?' ':'X'));
  }
  printf("|\n");
  */    
}

void SlotsBox::SetBoxType(string name, SlotsBox::BoxType type){
  int index = -1;
  
  pSlotsBox box = FindBox(name,&index);
  if(box==NULL) return;  
  box->mBoxType = type;  
  
  bValidUpdateSequence = false;
}

int SlotsBox::BuildUpdateSequence(){

  int size = int(mBoxes.size());

  vector<int> sequence;

  if(size>0){
    UpdateDependencies();
      
    vector<int> deps;
    vector<int> visited;
    vector<int> updated;
    vector<int> tmp;


    for(int k=0;k<size;k++){
      visited.push_back(0);
      updated.push_back(0);
      tmp.push_back(0);
    }
    
    // Prepare final deps
    // Inputs first
    for(int k=0;k<size;k++){
      if((!tmp[k])&&(mBoxes[k]->mBoxType==CIOTYPE_INPUT)){
        deps.push_back(k);
        tmp[k] = true;
      } 
    }        
    // Then delayed
    for(int k=0;k<size;k++){
      if((!tmp[k])&&(mBoxes[k]->mBoxType==CIOTYPE_DELAYED)){
        deps.push_back(k); 
        tmp[k] = true;
      } 
    }    
    // Then set outputs
    for(int k=0;k<size;k++){
      if((!tmp[k])&&(mBoxes[k]->mBoxType==CIOTYPE_OUTPUT)){
        deps.push_back(k);
        tmp[k] = true;
      } 
    }        
    // Then outputs
    for(int k=0;k<size;k++){
      if((!tmp[k])&&(mOutputBoxes[k])){
        deps.push_back(k); 
        tmp[k] = true;
      } 
    }
    
    if(deps.size()>0){
    
      ResolveDependencies(deps,visited,updated,sequence);
      MDEBUG(
      if(int(sequence.size())>0){
        for(int i=0;i<int(sequence.size())-1;i++){
          cout << sequence[i]<<"("<<mBoxes[sequence[i]]->GetName()<<") -> ";
        }
        cout << sequence[int(sequence.size())-1]<<"("<<mBoxes[sequence[int(sequence.size())-1]]->GetName()<<")"<<endl;
      }
      );
    }    
  }
  
  // Update sequence
  MDEBUG(
    cout << "Update sequence: "<<mBoxName<<endl;
  );
  mUpdateSequence.clear();
  for(int i=0;i<int(sequence.size());i++){
    mUpdateSequence.push_back(sequence[i]);
    
  }
  MDEBUG(
    if(mUpdateSequence.size()>0){
      cout <<" ->";
      for(unsigned int i=0;i<mUpdateSequence.size();i++){
        cout << mBoxes[mUpdateSequence[i]]->GetName()<< " ";
      }
      cout <<endl;
    }
  );
  
  bValidUpdateSequence = true;
  return true;
}

int SlotsBox::ResolveDependencies(vector<int> &deps, vector<int> &visited, vector<int> &updated, vector<int> &sequence){
  
  int size = int(mBoxes.size());
  
  MDEBUG(
    cout <<"Entering resolver..."<<endl<<"  ";
    for(int k=0;k<int(deps.size());k++){
      cout << deps[k]<<" ";
    }
    cout <<endl;
  );
  int result = true;
    
  // For each deps
  for(int i=0;i<int(deps.size());i++){
    int j = deps[i];
    if(j<0) continue;
    
    // Check if already updated
    if(updated[j]) continue;
    
    // Mark j as visited
    visited[j] = true;
  
    MDEBUG(cout << "Resolving "<<j<<endl;);
  
    // Mark dependencies as requests
    vector<int> newDeps;
    for(int k=0;k<size;k++){
      if(mDepsMatrix[j*size+k])
        newDeps.push_back(k);
    }

    // Checks for
    for(int k=0;k<int(newDeps.size());k++){
      // Updatedness
      if(updated[newDeps[k]])
        newDeps[k] = -1;
      // Recursivity
      if(visited[newDeps[k]]){ 
        newDeps[k]            = -1;
        result                = false;
        mDelayedBoxes[j] = true;
        MDEBUG(cout << j <<" should be delayed..."<<endl;);
        // SHOULD MARK j AS DELAYED
      }
    }
    
    // Count deps/
    int depsCount = 0;
    for(int k=0;k<int(newDeps.size());k++) depsCount += (newDeps[k]>=0?1:0);
    
    if(depsCount>0){      
      // Resolve dependencies
      vector<int> newVisited;
      for(int k=0;k<size;k++) newVisited.push_back(visited[k]);
      
      result = ResolveDependencies(newDeps,newVisited,updated,sequence);
    }
    MDEBUG(cout<< " Pushing "<<j<<endl;);
    sequence.push_back(j);
    updated[j] = true;
  }
  MDEBUG(cout <<"Exiting resolver..."<<endl;);
  return result;
}




void  SlotsBox::UpdateDependencies(){
  int size;
  
  size = int(mBoxes.size());  
  // Compute the dependency matrix
  mDepsMatrix.resize(size*size);
  for(int i=0;i<size;i++){
    for(int j=0;j<size;j++){
      mDepsMatrix[i*size+j] = (mBoxes[j]->IsConnectedTo(mBoxes[i])?true:false);       
    }
  }  
  
  mOutputBoxes.resize(size);
  for(int i=0;i<size;i++){
    mOutputBoxes[i] = (IsMappedFrom(mBoxes[i])?true:false);
  }

  mInputBoxes.resize(size);
  for(int i=0;i<size;i++){
    mInputBoxes[i] = (IsMappingTo(mBoxes[i])?true:false);
  }

  mDelayedBoxes.resize(size);
  for(int i=0;i<size;i++){
    mDelayedBoxes[i] = (mBoxes[i]->mBoxType==CIOTYPE_DELAYED?true:false);
  }
  
  MDEBUG(  
    printf("DepMatrix:\n");
    for(int i=0;i<size;i++){
      printf("|");
      for(int j=0;j<size;j++){
        printf("%c",(mDepsMatrix[i*size+j]==0?' ':'X'));
      }
      printf("|\n");
    }
    printf("Input Boxes:\n");
    printf("|");
    for(int i=0;i<size;i++){
      printf("%c",(mInputBoxes[i]==0?' ':'X'));
    }
    printf("|\n");
    printf("Output Boxes:\n");
    printf("|");
    for(int i=0;i<size;i++){
      printf("%c",(mOutputBoxes[i]==0?' ':'X'));
    }
    printf("|\n");
  );
}

void SlotsBox::EnableBox(bool enable){
  bBoxIsEnabled = enable;
}

void SlotsBox::EnableBox(string name, bool enable){
  int index = -1;  
  pSlotsBox inputBox     = FindBox(name,&index);
  if(inputBox) inputBox->EnableBox(enable);
}

void  SlotsBox::UpdateBox(int level){
  if(bBoxIsEnabled){
    //cout << "Updating box <"<< mBoxName <<"> **********"<<endl;
    if(mBoxPriority == level){
      PreUpdate(level);
    }    

    if(!bValidUpdateSequence){
      BuildUpdateSequence();
    }
    for(int i=0;i<int(mUpdateSequence.size());i++){
      mBoxes[mUpdateSequence[i]]->UpdateBox(level);  
    }

    if(mBoxPriority == level){
      Update(level);
    }
    //cout << "End update <"<< mBoxName <<"> **********"<<endl;
  }
}

void SlotsBox::SetPriority(int priority, bool recursive){
  mBoxPriority = priority;
  if(recursive){
    for(int i=0;i<int(mBoxes.size());i++){
      mBoxes[i]->SetPriority(priority,recursive);
    }  
  }
}

void SlotsBox::PreUpdate(int level){
}
void SlotsBox::Update(int level){
}

void SlotsBox::Init(pXmlTree config){
}








