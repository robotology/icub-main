// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_INTERFACE_GUI_CLIENT_H__
#define __GTKMM_ICUB_INTERFACE_GUI_CLIENT_H__

#include <gtkmm.h>
#include <iCubInterfaceGuiServer.h>

inline std::string toString(int i)
{
    char buff[16];
    sprintf(buff,"%d",i);
    return std::string(buff);
}

inline std::string toString(bool b)
{
    return b ? std::string("true") : std::string("false");
}

inline std::string toString(double d)
{
    char buff[64];
    sprintf(buff,"%f",d);
    return std::string(buff);
}

inline std::string toString(char *s)
{
    return std::string(s);
}

//Tree model columns
class ModelColumns : public Gtk::TreeModel::ColumnRecord
{
public:
    ModelColumns()
    {
        add(mColName);
        add(mColValue);
        add(mColStatus);
    }

    Gtk::TreeModelColumn<Glib::ustring> mColName;
    Gtk::TreeModelColumn<Glib::ustring> mColValue;
    Gtk::TreeModelColumn<Glib::RefPtr<Gdk::Pixbuf> > mColStatus;
};

///////////////////////////////////////////////////

class iCubBoardChannelGui
{
public:
    iCubBoardChannelGui()
    {
    }

    void createRows(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,char *rowNames)
    {
        mRefTreeModel=refTreeModel;

        mRows[0]=*(mRefTreeModel->append(parent.children()));
        maRows[0][mColumns.mColName]=rowNames[0];
        mRows[0][mColumns.mColValue]="";

        for (int i=1; i<NNAMES; ++i)
        {
            mRows[i]=*(mRefTreeModel->append(mRows[0].children()));
            mRows[i][gColumns.mColName]=rowNames[i];
            mRows[i][gColumns.mColValue]="";
        }
    }

    ~iCubBoardChannelGui()
    {
        delete [] mRows;
    }

protected:
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    Gtk::TreeModel::Row *mRows;
    ModelColumns mColumns;
};

class iCubBLLChannelGui : public iCubBLLChannel, public iCubBoardChannelGui
{

};


class iCubBoardGui : public iCubBoard
{
public:
    iCubBoardGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,int ID)
    {
        m_refTreeModel=refTreeModel;

        m_ID=*(m_refTreeModel->append(parent.children()));
        m_ID[g_columns.m_colName]="ID";
        m_ID[g_columns.m_colValue]=toString(ID).c_str();

        m_apChannel[0]=new iCubChannel(m_refTreeModel,m_ID,0);
        m_apChannel[1]=new iCubChannel(m_refTreeModel,m_ID,1);
    }

    ~iCubBoardGui()
    {
        //delete m_apChannel[0];
        //delete m_apChannel[1];
    }

protected:
    Glib::RefPtr<Gtk::TreeStore> m_refTreeModel;
    Gtk::TreeModel::Row m_ID;
    ModelColumns g_columns;
    iCubChannel *m_apChannel[2];
};

class iCubNetworkGui : public iCubNetwork
{
public:
    iCubNetworkGui(std::string &name,
              std::string &file,
              std::string &device,
              std::string &canBusDevice,
              int threadRate)
        : 
            name(name),
            file(file),
            device(device),
            canBusDevice(canBusDevice),
            threadRate(threadRate) 
    {
    }

    void configGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,
                   Gtk::TreeModel::Row& parent,
                   int ID)
    {
        m_refTreeModel=refTreeModel;

        m_aRows[0]=*(m_refTreeModel->append(parent.children()));
        m_aRows[0][g_columns.m_colName]=name.c_str();
        m_aRows[0][g_columns.m_colValue]=canBusDevice.c_str();

        /////////////////////////////////////////////////////

        m_aRows[1]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[1][g_columns.m_colName]="CanDeviceNum";
        m_aRows[1][g_columns.m_colValue]=toString(ID).c_str();

        m_aRows[2]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[2][g_columns.m_colName]="Driver Rx ovf";
        m_aRows[2][g_columns.m_colValue]="0";

        m_aRows[3]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[3][g_columns.m_colName]="Driver Tx ovf";
        m_aRows[3][g_columns.m_colValue]="0";

        m_aRows[4]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[4][g_columns.m_colName]="Rx Can errors";
        m_aRows[4][g_columns.m_colValue]="0";

        m_aRows[5]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[5][g_columns.m_colName]="Tx Can errors";
        m_aRows[5][g_columns.m_colValue]="0";

        m_aRows[6]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[6][g_columns.m_colName]="Rx Buffer ovf";
        m_aRows[6][g_columns.m_colValue]="0";

        m_aRows[7]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[7][g_columns.m_colName]="Bus off";
        m_aRows[7][g_columns.m_colValue]="false";

        m_aRows[8]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[8][g_columns.m_colName]="Requested rate";
        m_aRows[8][g_columns.m_colValue]=toString(threadRate).c_str();

        m_aRows[9]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[9][g_columns.m_colName]="Est. avg rate";
        m_aRows[9][g_columns.m_colValue]=toString(threadRate).c_str();

        m_aRows[10]=*(m_refTreeModel->append(m_aRows[0].children()));
        m_aRows[10][g_columns.m_colName]="Est. std rate";
        m_aRows[10][g_columns.m_colValue]="0";

        /////////////////////////////////////////////////////

        boards=*(m_refTreeModel->append(m_aRows[0].children()));
        boards[g_columns.m_colName]="Boards";
        boards[g_columns.m_colValue]="";
    }

    inline bool operator==(iCubNetwork& n)
    {
        return name==n.name;
    }

    ~iCubNetworkGui(){}

    void addBoard(int ID)
    {
        m_boards.push_back(new iCubBoard(m_refTreeModel,boards,ID));
    }

    std::string name;
    std::string file;
    std::string device;
    std::string canBusDevice;
    int threadRate;

protected:
    std::vector<iCubBoard*> m_boards;
    Gtk::TreeModel::Row m_aRows[11],boards;
    Glib::RefPtr<Gtk::TreeStore> m_refTreeModel;
    ModelColumns g_columns;
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
