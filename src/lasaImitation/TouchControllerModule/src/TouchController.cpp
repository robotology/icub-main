#define TOUCHCONTROLLER_CPP_
#include "TouchController.h"

TouchController::TouchController(){
    impl = new TouchControllerImpl();
}
TouchController::TouchController(MMiceDeviceDriver *miceDriver){
    impl = new TouchControllerImpl();
    SetMMiceDriver(miceDriver);
}
TouchController::~TouchController(){
    delete impl;
}
    
void    TouchController::Update(){
    impl->Update();
}
void    TouchController::Get6DOFSensingOutput(yarp::sig::Vector &output){
    impl->Get6DOFSensingOutput(output);
}
void    TouchController::Get2DOFSensingOutput(yarp::sig::Vector &output){
    impl->Get2DOFSensingOutput(output);    
}
void    TouchController::Get6DOFSensingOutputMask(yarp::sig::Vector &mask){
    impl->Get6DOFSensingOutputMask(mask);    
}
void    TouchController::SetMMiceDriver(MMiceDeviceDriver *miceDriver){
    impl->SetMMiceDriver(miceDriver);   
}
void    TouchController::Set6DOFSensingCoefs(yarp::sig::Vector &coefs){
    impl->Set6DOFSensingCoefs(coefs);
}
void    TouchController::GetOrientationFrameOutput(yarp::sig::Vector &output){
    impl->GetOrientationFrameOutput(output);
}
void    TouchController::SetOrientationFrame(yarp::sig::Matrix &frame){   
    impl->SetOrientationFrame(frame);
}    
void    TouchController::SetOutputLimits(double transLimit, double orientLimit){
    impl->SetOutputLimits(transLimit,orientLimit);        
}    
void    TouchController::SetDecayFactor(double factor){
    impl->SetDecayFactor(factor);    
}
bool    TouchController::TouchDetected(){
    return impl->TouchDetected();    
}
bool    TouchController::ResetDetected(){
    return impl->ResetDetected();    
}
void    TouchController::GetOrientationFrameIntegratedOutput(yarp::sig::Vector &output){
    impl->GetOrientationFrameIntegratedOutput(output);
}    
void    TouchController::ResetIntegratedOutput(){
    impl->ResetIntegratedOutput();
}    
bool    TouchController::GetLastMouseTouch(){
    return impl->GetLastMouseTouch();
}




TouchControllerImpl::TouchControllerImpl(){
    mMiceDriver = NULL;
    mMiceLimitX = 100.0;
    mMiceLimitY = 100.0;    
    mMiceLimitRelX = 20.0;
    mMiceLimitRelY = 20.0;
    bLastBtnState = false;    
    for(int i=0;i<6;i++){
        mCoefs[i] = 1;
        m6DofSensorsRelOutputMem[i] = 0.0;
    }    
    mOrientFrame.Identity();
    mTransComp.Zero();
    mOrientComp.Zero();
    mTransCompOut.Zero();
    mOrientCompOut.Zero();
    
    mLastTouchTime      = Time::now();
    mTouchResetTime     = 0.5;            
    mLastBigMoveStatus  = false;
    mTransLimit  = 0;
    mOrientLimit = 0;
    mDecayFactor = -1.0;
    
    bLastMiceTouch = false;
}

#define TC_VERSION_2

void    TouchControllerImpl::Update(){
   //return;
    
    double timeNow = Time::now();
    double dt      = timeNow - mLastTime;
    if(dt>0.1) dt = 0.0;
    
    for(int i=0;i<6;i++){
        m6DofSensorsOutput[i] = 0.0;    
        m6DofSensorsOutputMask[i] = 0.0;    
    }
    MMiceDeviceDriver::MouseEventSummary *mice[5];
    for(int i=0;i<5;i++){
        mice[i] = mMiceDriver->GetMouseData(i);
        //if(mMiceDriver->HasMouseChanged(i)) cout << "Mouse "<<i<<endl;
    }
    
    Vector absX(5);
    Vector absY(5);
    Vector relX(5);
    Vector relY(5);
    bool   bBtn=true;
    bool activeMice[5];
    for(int i=0;i<5;i++){
        activeMice[i] = (mMiceDriver->HasMouseChanged(i))||(mice[0]->btnLinkState>0);
        absX(i) = mice[i]->mAbs.X;
        absY(i) = mice[i]->mAbs.Y;
        relX(i) = mice[i]->mRel.X;
        relY(i) = mice[i]->mRel.Y;
        bBtn   &= mice[i]->btnLinkState;
    }
    absX.Trunc(-mMiceLimitX,mMiceLimitX); absX /= mMiceLimitX; 
    absY.Trunc(-mMiceLimitY,mMiceLimitY); absY /= mMiceLimitY;
    relX.Trunc(-mMiceLimitRelX,mMiceLimitRelX); relX /= mMiceLimitRelX; 
    relY.Trunc(-mMiceLimitRelY,mMiceLimitRelY); relY /= mMiceLimitRelY;


#ifdef TC_VERSION_2
    double tau   = 0.1;
    int  btnDown  = 0;
    bool bChanged    = false;
    bool bBigRelMove = false;
    for(int i=0;i<5;i++){
        bBigRelMove |= (mice[i]->mInt.X*mice[i]->mInt.X + mice[i]->mInt.Y*mice[i]->mInt.X > (200*200));
        bChanged |= mice[i]->hasChanged;
        btnDown  += mice[i]->btnDown;
    }
    bLastMiceTouch = (mice[4]->btnDown>0);
    
    bResetDetected = false;
    if(bChanged){
        //tau = 0.1;
        double now = Time::now();
        if((bBigRelMove&&(!mLastBigMoveStatus))||(btnDown>0)){
        //if(((now - mLastTouchTime) > mTouchResetTime)||(btnDown>0)){
            if(mDecayFactor<=0.0){
                mTransComp.Zero();
                mOrientComp.Zero();
            }
            if(btnDown){
                mTransComp.Zero();
                mOrientComp.Zero();
                mTransCompOut.Zero();
                mOrientCompOut.Zero();
                cout << "hard reset"<<endl;
                bResetDetected = true;
            }else{
                cout << "reset"<<endl;
            }
        } 
        mLastTouchTime = Time::now();
    }
    mLastBigMoveStatus = bBigRelMove;

    if(bResetDetected){
        absX.Zero();
        absY.Zero();
        relX.Zero();
        relY.Zero();
    }

    // LeftRight
    m6DofSensorsRelOutput[0] = 0.5*(relY(0)-relY(2));
    m6DofSensorsRelOutput[3] =-0.5*(relX(0)-relX(2));

    // UpDown
    m6DofSensorsRelOutput[2] =-0.5*(relY(1)-relY(3));
    m6DofSensorsRelOutput[5] = 0.5*(relX(1)-relX(3));        

    m2DofSensorsOutput[0] = -relY(4);        
    m2DofSensorsOutput[1] = relX(4);        
    relY(4) = 0;        
    relX(4) = 0;        

    // Forward
    IndicesVector ids;
    relX.AbsSort(&ids);         
    m6DofSensorsRelOutput[1] = 0.5*(relX(0)+relX(1));
    
    relY.AbsSort(&ids);         
    m6DofSensorsRelOutput[4] = 0.5*(relY(0)+relY(1));        
    
    /*
    // LeftRight
    m6DofSensorsRelOutput[0] = 0.5*(relY(0)-relY(2)) + 0.5*(fabs(relX(3))+fabs(relY(3))) - 0.5*(fabs(relX(1))+fabs(relY(1)));
    m6DofSensorsRelOutput[3] =-0.5*(relX(0)-relX(2)) + 0.25*(relY(1)-relY(3));

    // UpDown
    m6DofSensorsRelOutput[2] =-0.5*(relY(1)-relY(3)) + 0.5*(fabs(relX(2))+fabs(relY(2))) - 0.5*(fabs(relX(0))+fabs(relY(0)));;
    m6DofSensorsRelOutput[5] = 0.5*(relX(1)-relX(3)) + 0.25*(relY(0)-relY(2));        

    // Forward
    IndicesVector ids;
    relX.AbsSort(&ids);         
    m6DofSensorsRelOutput[1] = 0.5*(relX(0)+relX(1));
    
    relY.AbsSort(&ids);         
    m6DofSensorsRelOutput[4] = 0.5*(relY(0)+relY(1));        
    */  
    for(int i=0;i<6;i++){
        m6DofSensorsOutput[i] = m6DofSensorsRelOutput[i];
    }
        
#endif    

#ifdef TC_VERSION_1
    if(bBtn){    
        // LeftRight
        m6DofSensorsOutput[0] = 0.5*(absY(0)-absY(2)) + 0.5*(fabs(absX(3))+fabs(absY(3))) - 0.5*(fabs(absX(1))+fabs(absY(1)));;
        m6DofSensorsOutput[3] =-0.5*(absX(0)-absX(2));

        // UpDown
        m6DofSensorsOutput[2] =-0.5*(absY(1)-absY(3)) + 0.5*(fabs(absX(2))+fabs(absY(2))) - 0.5*(fabs(absX(0))+fabs(absY(0)));;
        m6DofSensorsOutput[5] = 0.5*(absX(1)-absX(3));        

        // Forward
        IndicesVector ids;
        absX.AbsSort(&ids);         
        m6DofSensorsOutput[1] = 0.5*(absX(0)+absX(1));
        
        absY.AbsSort(&ids);         
        m6DofSensorsOutput[4] = 0.5*(absY(0)+absY(1));

    }else{
        
        // LeftRight
        m6DofSensorsRelOutput[0] = 0.5*(relY(0)-relY(2)) + 0.5*(fabs(relX(3))+fabs(relY(3))) - 0.5*(fabs(relX(1))+fabs(relY(1)));
        m6DofSensorsRelOutput[3] =-0.5*(relX(0)-relX(2));

        // UpDown
        m6DofSensorsRelOutput[2] =-0.5*(relY(1)-relY(3)) + 0.5*(fabs(relX(2))+fabs(relY(2))) - 0.5*(fabs(relX(0))+fabs(relY(0)));;
        m6DofSensorsRelOutput[5] = 0.5*(relX(1)-relX(3));        

        // Forward
        IndicesVector ids;
        relX.AbsSort(&ids);         
        m6DofSensorsRelOutput[1] = 0.5*(relX(0)+relX(1));
        
        relY.AbsSort(&ids);         
        m6DofSensorsRelOutput[4] = 0.5*(relY(0)+relY(1));        
        
        
        if(bLastBtnState){
            for(int i=0;i<6;i++){
                m6DofSensorsRelOutputMem[i] = 0.0;
            }            
        }else{
            for(int i=0;i<6;i++){
                double tau;
                if(fabs(m6DofSensorsRelOutput[i])>1e-3){
                    tau = 0.5;
                }else{
                    tau = 0.08;    
                }
                m6DofSensorsRelOutputMem[i] = m6DofSensorsRelOutputMem[i] + tau *(-m6DofSensorsRelOutputMem[i]+m6DofSensorsRelOutput[i]);
                if(fabs(m6DofSensorsRelOutputMem[i])<1e-4)
                    m6DofSensorsRelOutputMem[i] = 0;
                    
                m6DofSensorsOutput[i] = m6DofSensorsRelOutputMem[i];
                 
            }
        }        
    }
    for(int i=0;i<6;i++){
        if(fabs(m6DofSensorsOutput[i])>0.05){
            m6DofSensorsOutputMask[i] = 1.0;    
        }
        m6DofSensorsOutput[i]*=mCoefs[i];
    }
#endif    



#ifdef TC_VERSION_2    
    for(int i=0;i<6;i++){
        if(fabs(m6DofSensorsOutput[i])>0.05){
            m6DofSensorsOutputMask[i] = 1.0;    
        }
    }

    Vector3 transC0, orientC0,transC,orientC;
    for(int i=0;i<3;i++){
        transC0(i)  = m6DofSensorsOutput[i]   * mCoefs[i];
        orientC0(i) = m6DofSensorsOutput[i+3] * mCoefs[i+3];
    }

    Matrix3 src; 
    Matrix3 off; 
    Matrix3 dst; 
    Vector3 dw;

    mOrientFrame.Mult(transC0,transC);
    mOrientFrame.Mult(orientC0,orientC);
    
    mTransComp  += transC;
    
    //mOrientComp += orientC;
    src.RotationV(mOrientComp);
    off.RotationV(orientC);
    off.Mult(src,dst);
    dst.GetExactRotationAxis(mOrientComp);
    double dtTau;
    if(mDecayFactor>0.0){
        dtTau = MIN(0.5,(dt/mDecayFactor));
        mTransComp  += (-mTransComp)  * dtTau; 
        mOrientComp += (-mOrientComp) * dtTau; 
    }    
    

    double norm;
    norm = mTransComp.Norm();
    if(norm>mTransLimit)
        mTransComp *= mTransLimit/norm;     
    norm = mTransComp.Norm();
    //cout << norm / mTransLimit<<" ";
    
    norm = mOrientComp.Norm();
    if(norm>mOrientLimit)
        mOrientComp *= mOrientLimit/norm;     
    norm = mOrientComp.Norm();
    //cout << norm / mOrientLimit<<" ";
    //cout << endl;    


    dtTau = dt/tau;
    dtTau = MIN(0.5,dtTau);
    mTransCompOut  += (-mTransCompOut  + mTransComp )*dtTau;
    for(int i=0;i<2;i++){
        m2DofCompOut[i] += (-m2DofCompOut[i] + m2DofSensorsOutput[i])*dtTau;
    } 
    
    
    //mOrientCompOut += (-mOrientCompOut + mOrientComp)*dtTau;
    src.RotationV(mOrientCompOut); src.STranspose();
    off.RotationV(mOrientComp);
    off.Mult(src,dst);
    dst.GetExactRotationAxis(dw);
    dw *= dt/tau;
    src.RotationV(mOrientCompOut);
    off.RotationV(dw);
    off.Mult(src,dst);
    dst.GetExactRotationAxis(mOrientCompOut);
        
    
    mTransCompIntOut  += mTransCompOut*dt; 

    src.RotationV(mOrientCompIntOut);
    off.RotationV(mOrientCompOut*dt);
    off.Mult(src,dst);
    dst.GetExactRotationAxis(mOrientCompIntOut);
    //mOrientCompIntOut += mOrientCompOut*dt; 
#endif
    
    bLastBtnState   = bBtn;
    mLastTime       = timeNow;
}
void    TouchControllerImpl::Get6DOFSensingOutput(yarp::sig::Vector &output){
    if(output.size()!=6) output.resize(6);
    for(int i=0;i<6;i++){
        output(i) = m6DofSensorsOutput[i];
    }
}
void    TouchControllerImpl::Get6DOFSensingOutputMask(yarp::sig::Vector &mask){
    if(mask.size()!=6) mask.resize(6);
    for(int i=0;i<6;i++){
        mask(i) = m6DofSensorsOutputMask[i];
    }    
}

void    TouchControllerImpl::SetMMiceDriver(MMiceDeviceDriver *miceDriver){
    mMiceDriver = miceDriver;
}

void    TouchControllerImpl::Set6DOFSensingCoefs(yarp::sig::Vector &coefs){
    for(int i=0;i<6;i++) mCoefs[i] = 1;
    int len = MIN(coefs.size(),6);
    for(int i=0;i<len;i++){
        mCoefs[i] = coefs(i);    
    }    
}

void    TouchControllerImpl::Get2DOFSensingOutput(yarp::sig::Vector &output){
    if(output.size()!=2) output.resize(2);
    for(int i=0;i<2;i++){
        output(i) = m2DofCompOut[i];
    }
}


void    TouchControllerImpl::GetOrientationFrameOutput(yarp::sig::Vector &output){
    if(output.size()!=6) output.resize(6);
    for(int i=0;i<3;i++){
        output(i)   = mTransCompOut(i);
        output(i+3) = mOrientCompOut(i);
    }    
}


void    TouchControllerImpl::SetOrientationFrame(yarp::sig::Matrix &frame){   
    mOrientFrame.Identity();    
    int r = MIN(frame.rows(),3);
    int c = MIN(frame.cols(),3);
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            mOrientFrame(i,j) = frame(i,j);
        }
    }
}    
void    TouchControllerImpl::SetOutputLimits(double transLimit, double orientLimit){
    mTransLimit  = fabs(transLimit);
    mOrientLimit = fabs(orientLimit);
}    
bool    TouchControllerImpl::TouchDetected(){
    return ((mOrientCompOut.Norm2()+mTransCompOut.Norm2())>1e-6);
}
bool    TouchControllerImpl::ResetDetected(){
    return bResetDetected;
}
void    TouchControllerImpl::SetDecayFactor(double factor){
    mDecayFactor = TRUNC(factor,0.0,1.0);
}
void    TouchControllerImpl::GetOrientationFrameIntegratedOutput(yarp::sig::Vector &output){
    if(output.size()!=6) output.resize(6);
    for(int i=0;i<3;i++){
        output(i)   = mTransCompIntOut(i);
        output(i+3) = mOrientCompIntOut(i);
    }            
}    
void    TouchControllerImpl::ResetIntegratedOutput(){
    mTransCompIntOut.Zero();
    mOrientCompIntOut.Zero();
}    
bool    TouchControllerImpl::GetLastMouseTouch(){
    return bLastMiceTouch;
}

