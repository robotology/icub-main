// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

////////////////////////////////////

#include "iCubNetwork.h"

////////////////////////////////////

bool iCubNetwork::findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)
{
    int index=addr.find(",");

    std::string name=index<0?addr:addr.substr(0,index);

    if (name.length()==0) return false; // should never happen

    if (name!=mName) return false;

    // is the message for the network or for a board channel?
    if (index<0)
    {
        // for the network
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

        return true;
    }

    // for a board channel

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

