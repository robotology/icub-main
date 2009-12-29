// Header file for base MotionTracker class
// Contains all external MT functions

#ifndef __MOTIONTRACKER_H
#define __MOTIONTRACKER_H



/**
 * \class MotionTracker
 * 
 * \brief A wrapper for the XSens API
 * 
 * Base class for MotionTracker Filter
 */
class MotionTracker {
	public:
		virtual short MT_SetOutputMode(short nMode)=0;
		virtual short MT_SetSampleFrequency(short nSampleFreq)=0;
		virtual short MT_SetCalibratedOutput(short nEnabled)=0;
		virtual short MT_SetCOMPort(short nPort, int nBaudrate=115200)=0;
		virtual short MT_SetCOMPort_DeviceName(char *pchDeviceName, int nBaudrate=115200)=0;
		virtual short MT_SetxmuLocation(char *pchName)=0;
		virtual short MT_SetFilterSettings(float fGain, short nCorInterval, float fRho)=0;
		virtual short MT_SetDoAMD(short nDoAMD)=0;
		virtual short MT_GetOrientationData(short *pnRetVal, float fOutputArray[], short nLatest=1)=0;
		virtual short MT_GetCalibratedData(short *pnRetVal, float fOutputArray[], short nLatest=1)=0;
		virtual short MT_GetFilterSettings(float *pfGain, short *pnCorInterval, float *pfRho)=0;
		virtual void  MT_StartProcess(void)=0;
		virtual void  MT_StopProcess(void)=0;
		virtual void  MT_ResetOrientation(short nResetType, short bSaveAfterStop=0)=0;
		virtual short MT_SetInputPacket(unsigned char *puchInputPacket)=0;
		virtual short MT_SetInputPacketB(unsigned char *puchInputPacket)=0;
		virtual short MT_SetMTSData(unsigned char *puchMTSData0,unsigned char *puchMTSData1=0,unsigned char *puchMTSData2=0)=0;
		virtual short MT_GetMTSData(unsigned char *puchMTSData)=0;
	  virtual short MT_SetTimeout(short nTimeout)=0;
    virtual short MT_QueryMotionTrackerB(char *pchDeviceID)=0;
    virtual short MT_SetTimeStampOutput(short nEnabled)=0;
    virtual short MT_SaveToMTS(char chDeviceID[])=0;
    virtual short MT_SetMotionTrackerLocation(short nLocation)=0;
    virtual short MT_GetMotionTrackerLocation(short *pnLocation)=0;
    virtual short MT_SetMotionTrackerSampleFrequency(int nSampleFrequency)=0;
    virtual short MT_GetMotionTrackerSampleFrequency(int *pnSampleFrequency)=0;
    virtual short MT_SetMotionTrackerHeading(float fHeading)=0;
    virtual short MT_GetMotionTrackerHeading(float *pfHeading)=0;
    virtual short MT_SetMotionTrackerBaudrate(int nBaudrate)=0;
    virtual short MT_GetMotionTrackerBaudrate(int *pnBaudrate)=0;
        
    virtual short XM_SetxmuLocation(char chDeviceID[], char *pchName)=0;
		virtual short XM_SetFilterSettings(char chDeviceID[], float fGain, short nCorInterval, float fRho)=0;
		virtual short XM_SetDoAMD(char chDeviceID[], short nDoAMD)=0;
		virtual short XM_GetFilterSettings(char chDeviceID[], float *pfGain, short *pnCorInterval, float *pfRho)=0;
		virtual short XM_QueryXbusMaster(short *pnNumSensors,char *pchDeviceID0, char *pchDeviceID1, char *pchDeviceID2, char *pchDeviceID3, char *pchDeviceID4, char *pchDeviceID5)=0;
		virtual short XM_QueryXbusMasterB(short *pnNumSensors, char *pchDeviceIDs)=0;
		virtual short XM_SetOutputMode(short nMode)=0;
		virtual short XM_SetSampleFrequency(short nSampleFreq)=0;
		virtual short XM_SetCalibratedOutput(short nEnabled)=0;
		virtual short XM_SetCOMPort(short nPort, int nBaudrate=115200)=0;
		virtual short XM_SetCOMPort_DeviceName(char *pchDeviceName, int nBaudrate=115200)=0;
		virtual short XM_SetTimeStampOutput(short nEnabled)=0;					
		virtual short XM_GetOrientationData(short *pnRetVal, float fOutputArray[], short nLatest=1)=0;					
		virtual short XM_GetCalibratedData(short *pnRetVal, float fOutputArray[], short nLatest=1)=0;					
		virtual void  XM_StartProcess()=0;
		virtual void  XM_StopProcess()=0;
		virtual void  XM_ResetOrientation(short nResetType, short bSaveAfterStop=0)=0;
		virtual short XM_SetTimeout(short nTimeOut)=0;
		virtual short XM_SetMotionTrackerLocation(char chDeviceID[], short nLocation)=0;
		virtual short XM_GetMotionTrackerLocation(char chDeviceID[], short *nLocation)=0;
		virtual short XM_SetMotionTrackerHeading(char chDeviceID[], float fHeading)=0;
		virtual short XM_GetMotionTrackerHeading(char chDeviceID[], float *fHeading)=0;
		virtual short XM_SaveToMTS(char chDeviceID[])=0;		
		virtual short XM_SetXbusMasterSampleFrequency(int nSampleFrequency)=0;
		virtual short XM_SetXbusMasterBaudrate(int nBaudrate)=0;
		virtual short XM_GetXbusMasterSampleFrequency(int *nSampleFrequency)=0;
		virtual short XM_GetXbusMasterBaudrate(int *nBaudrate)=0;
		
		
		virtual void  GetVersionNumber(float *fVersionNumber)=0;
};

// The types of the class factories
typedef MotionTracker* create_t();
typedef void destroy_t(MotionTracker*);

#endif // __MOTIONTRACKER_H
