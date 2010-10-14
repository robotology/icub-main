// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_INTERFACE_GUI_H__
#define __GTKMM_ICUB_INTERFACE_GUI_H__

#include <string>
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>

////////////////////////////////////

class iCubBoardChannel
{
public:
    iCubBoardChannel(int ch) : mChannel(ch)
    {
    }

    virtual ~iCubBoardChannel()
    {
    }

    virtual bool write(int index,double d)=0;
    virtual bool write(int index,bool d)=0;
    virtual bool write(int index,int d)=0;

    virtual bool read(int index,double& d,bool rst=true)=0;
    virtual bool read(int index,bool& d,bool rst=true)=0;
    virtual bool read(int index,int& d,bool rst=true)=0;

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)=0;

    virtual yarp::os::Bottle toBottle()=0;
    virtual void fromBottle(yarp::os::Bottle& bot)=0;

protected:
    const int mChannel;
};

class iCubBLLChannel : public iCubBoardChannel
{
public:
    iCubBLLChannel(int ch,int j) : iCubBoardChannel(ch),mJoint(j)
    {
        for (int i=0; i<(int)DOUBLE_NUM; ++i)
        {
            mDoubleFlag[i]=true;
        }

        for (int i=0; i<(int)BOOL_NUM; ++i)
        {
            mBoolFlag[i]=true;
        }

        for (int i=0; i<(int)INT_NUM; ++i)
        {
            mIntFlag[i]=true;
        }
    }
    
    virtual ~iCubBLL()
    {
    }    

    enum DoubleIndex
    {
        // interface generated
        DOUBLE_Status_messages_latency,       // Keep track of the time the last status message has been received (seconds)
        DOUBLE_Encoder_latency,               // Keep track of the time the last encoder reading has been received
        DOUBLE_NUM
    };

    enum BoolIndex
    {
        // interface generated
        BOOL_Status_messages_latency_timeout; // If status messages latency > threshold (5s) raise an error
        BOOL_Encoder_latency_timeout;         // If encoder latency > threshold (5s) raise an error

        // device generated
        BOOL_Is_Fault_Ok,               // Status of the fault pin, general error
        BOOL_Fault_undervoltage,        // Power supply voltage is below minimum
        BOOL_Fault_overload,            // Hardware fault triggered by the operational amplifier
        BOOL_Fault_overcurrent,	        // Current exceeds maximum value
        BOOL_Fault_external,            // External fault button is pressed
        BOOL_Hall_sensor_error,	        // Brushless hall effect sensor error
        BOOL_Absolute_encoder_error,    // Read error in absolute position sensor
        BOOL_BusOff,		
        BOOL_CanTx_Overflow,	        // Canbus Tx Buffer overflow (firmware)
        BOOL_Can_Rx_Overrun,            // Canbus Rx buffer overflow (firmware)
        BOOL_Main_loop_overflow,        // Main loop exceeded requested period (>1ms, typically)
        BOOL_Over_temperature,	
        BOOL_Temp_sensor_error,         // Read error in temperature sensor
        BOOL_NUM
    };

    enum IntIndex
    {
        // device denerated
        INT_Can_Tx_Error_counter,	
        INT_Can_Rx_Error_counter,
        INT_Control_mode,               // Status of the controller. This enumeration is illustrated below.
        INT_NUM
    };

    enum ControlMode
    { 
        MODE_IDLE,                  // no pwm enabled
        MODE_POSITION,              // position control
        MODE_VELOCITY,              // velocity control
        MODE_TORQUE,                // torque control (generates a given joint level torque) 
        MODE_IMPEDANCE,             // impedance control (generates a given stiffness around an equilibrium point)
        MODE_CALIB_ABS_POS_SENS,    // calibrate joint using an absolute position sensor
        MODE_CALIB_HARD_STOPS,      // calibrate joint using hardware limit
        MODE_HANDLE_HARD_STOPS,     // trying to get out from an hardware limit
        MODE_MARGIN_REACHED,        // trying to get within limits when the margin to limits is too small
        MODE_OPENLOOP               // receiving PWM values via canbus
    };

    virtual bool write(int index,double d)
    {
        if (index<0 || index>=DOUBLE_NUM) return false;

        if (mDoubleData[index]!=d)
        {
            mDoubleData[index]=d;
            mDoubleFlag[index]=true;
        }

        return true;
    }

    virtual write(int index,bool b)
    {
        if (index<0 || index>=BOOL_NUM) return false;

        if (mBoolData[index]!=d)
        {
            mBoolData[index]=d;
            mBoolFlag[index]=true;
        }

        return true;
    }
    virtual write(int index,int d)
    {
        if (index<0 || index>=INT_NUM) return false;

        if (mIntData[index]!=d)
        {
            mIntData[index]=d;
            mIntFlag[index]=true;
        }

        return true;
    }

    virtual bool read(int index,double& d,bool rst=true)
    {
        if (index<0 || index>=DOUBLE_NUM) return false;
        
        d=mDoubleData[index];
        
        bool tmp=mDoubleFlag[index];

        if (rst) mDoubleFlag[index]=false;

        return tmp;
    }
    
    virtual bool read(int index,bool& d,bool rst=true)
    {
        if (index<0 || index>=BOOL_NUM) return false;
        
        d=mBoolData[index];
        
        bool tmp=mBoolFlag[index];

        if (rst) mBoolFlag[index]=false;

        return tmp;
    }

    virtual bool read(int index,int& d,bool rst=true)
    {
        if (index<0 || index>=INT_NUM) return false;
        
        d=mIntData[index];
        
        bool tmp=mIntFlag[index];

        if (rst) mIntFlag[index]=false;

        return tmp;
    }

    virtual yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle bot;

        bot.addInt(mChannel);
        bot.addInt(mJoint);

        double d;
        for (int i=0; i<(int)DOUBLE_NUM; ++i)
        {
            if (read(i,d))
            {
                bot.addInt(i);
                bot.addDouble(d);
            }
        }

        bool d;
        for (int i=0; i<(int)BOOL_NUM; ++i)
        {
            if (read(i,d))
            {
                bot.addInt(i);
                bot.addVocab(d?'T':'F');
            }
        }

        int d;
        for (int i=0; i<(int)INT_NUM; ++i)
        {
            if (read(i,d))
            {
                bot.addInt(i);
                bot.addInt(d);
            }
        }
    }

    virtual void fromBottle(yarp::os:Bottle& bot)
    {
        double d;
        for (int i=2; i<bot.size(); i+=2)
        {
            int index=bot.get(i).asInt();
            yarp::os:Value data=bot.get(i+1);

            if (data.isDouble())
            {
                mDoubleData[index]=data.asDouble();
                mDoubleFlag[index]=true;
            }
            else if (data.isVocab())
            {
                mBoolData[index]=data.asVocab()=='T';
                mBoolFlag[index]=true;
            }
            else if (data.isInt())
            {
                mIntData[index]=data.asInt();
                mIntFlag[index]=true;
            }
        }
    }

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)
    {
        int index=addr.find(",");

        if (index<0) return false;
        
        std::string sCh=addr.substr(0,index);

        int ch=atoi(sCh.c_str());

        if (ch!=mChannel) return false;

        for (int i=0; i<(int)DOUBLE_NUM; ++i)
        {
            write(i,dataDouble[i]);
        }

        for (int i=0; i<(int)BOOL_NUM; ++i)
        {
            write(i,dataBool[i]);
        }

        for (int i=0; i<(int)INT_NUM; ++i)
        {
            write(i,dataInt[i]);
        }
    }

protected:
    const int mJoint; // Corresponding joint (for readability)

    double mDoubleData[DOUBLE_NUM];
    bool mDoubleFlag[DOUBLE_NUM];

    bool mBoolData[BOOL_NUM];
    bool mBoolFlag[BOOL_NUM];

    int mIntData[INT_NUM];
    bool mIntFlag[INT_NUM];
};

////////////////////////////////////

class iCubBoard
{
public:
    iCubBoard(int ID) : mID(ID)
    {
    }

    virtual ~iCubBoard()
    {
    }

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)=0;

protected:
    const int mID;
};

class iCubBLLBoard : iCubBoard
{
public:
    iCubBLLBoard(int ID) : iCubBoard(ID)
    {
        mChannel[0]=new iCubBLLChannel(0);
        mChannel[1]=new iCubBLLChannel(1);
    }

    virtual ~iCubBoard()
    {
        delete mChannel[0];
        delete mChannel[1];
    }

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)
    {
        int index=addr.find(",");

        if (index<0) return false;
        
        std::string sID=addr.substr(0,index);

        int ID=atoi(sID.c_str());

        if (ID!=mID) return false;

        ++index;

        addr=addr.substr(index,addr.length()-index);

        for (int i=0; i<2; ++i)
        {
            if (mChannel[i]->findAndWrite(addr,dataDouble,dataBool,dataInt))
            {
                return true;
            }
        }

        return false;
    }

protected:
    iCubBLLChannel *mChannel[2];
};

/////////////////////////////////

class iCubNetwork
{
public:
    iCubNetwork(std::string &name,
              std::string &file,
              std::string &device,
              std::string &canBusDevice,
              int threadRate)
        : 
            mName(name),
            mFile(file),
            mDevice(device),
            mCanBusDevice(canBusDevice),
            mThreadRate(threadRate) 
    {
    }

    ~iCubNetwork()
    {
        for (int i=0; i<mBoards.size(); ++i)
        {
            delete mBoards[i];
        }
    }

    inline bool operator==(iCubNetwork& n)
    {
        return name==n.mName;
    }

    void addBoard(iCubBoard *board)
    {
        mBoards.push_back(board);
    }

    bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)
    {
        int index=addr.find(",");

        if (index<0) return false;

        std::string name=addr.substr(0,index);

        if (name!=mName) return false;

        ++index;

        addr=addr.substr(index,addr.length()-index);

        for (int i=0; i<mBoards.size(); ++i)
        {
            if (mBoards[i]->findAndWrite(addr,dataDouble,dataBool,dataInt)
            {
                return true;
            }
        }

        return false;
    }

    std::string mName;
    std::string mFile;
    std::string mDevice;
    std::string mCanBusDevice;
    int mThreadRate;

protected:
    std::vector<iCubBoard*> mBoards;
};
























class iCubInterfaceGui : public Gtk::Window, public yarp::os::Thread
{
public:
    iCubInterfaceGui(yarp::os::Property &robot);
    virtual ~iCubInterfaceGui();

    void run()
    {

        // ask for configuration
        while (true)
        {
        } 
    }

    yarp::os::Bottle toBottle();
    void fromBottle(yarp::os::Bottle &bot);

protected:
    ModelColumns g_columns;
    //Signal handlers:
    void on_treeview_row_activated(const Gtk::TreeModel::Path& path, Gtk::TreeViewColumn* column);

    //Child widgets:
    Gtk::VBox m_VBox;

    Gtk::ScrolledWindow m_scrolledWindow;
    Gtk::TreeView m_treeView;
    Glib::RefPtr<Gtk::TreeStore> m_refTreeModel;

    std::vector<iCubNetwork*> m_networks;

    void addNetwork(iCubNetwork* net)
    {
        for (unsigned int i=0; i<m_networks.size(); ++i)
        {
            if (*m_networks[i]==*net)
            {
                if (m_networks[i]->threadRate>net->threadRate)
                {
                    m_networks[i]->threadRate=net->threadRate;
                }

                delete net;

                return;
            }
        }

        m_networks.push_back(net);
    }
};

#endif //__GTKMM_ICUB_INTERFACE_GUI_H__
