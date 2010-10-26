// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_INTERFACE_GUI_CLIENT_H__
#define __GTKMM_ICUB_INTERFACE_GUI_CLIENT_H__

#include <gtkmm.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include "iCubNetworkGui.h"

class iCubInterfaceGuiClient : public Gtk::Window, public yarp::os::RateThread
{
public:
    iCubInterfaceGuiClient() : RateThread(1000)
    {
        set_title("iCubInterface GUI");
        set_border_width(5);
        set_default_size(600,400);

        add(mVBox);
        mScrolledWindow.add(mTreeView);
        mScrolledWindow.set_policy(Gtk::POLICY_AUTOMATIC,Gtk::POLICY_AUTOMATIC);
        mVBox.pack_start(mScrolledWindow);

        mRefTreeModel=Gtk::TreeStore::create(mColumns);
        mTreeView.set_model(mRefTreeModel);

        mTreeView.set_reorderable();

        ////////////////////////////

        Gtk::CellRendererPixbuf* cell=Gtk::manage(new Gtk::CellRendererPixbuf);

        int colsNum=mTreeView.append_column("Status",*cell);

        Gtk::TreeViewColumn* pColumn=mTreeView.get_column(colsNum-1);
        pColumn->add_attribute(cell->property_pixbuf(),mColumns.mColStatus);

	    //Add the TreeView's view columns:
        mTreeView.append_column("Name",mColumns.mColName);
        mTreeView.append_column("ID",mColumns.mColValue);
        mTreeView.append_column("Status",mColumns.mColStatus);

        //Fill the TreeView's model
        mRowLev0=*(mRefTreeModel->append());
        mRowLev0[mColumns.mColName]="Networks"; //partName.c_str();
        mRowLev0[mColumns.mColValue]=""; //partName.c_str();
        //mRowLev0[mColumns.mColStatus]=Gdk::Pixbuf::create_from_file("warning_icon.png");

        
        mPort.open("/icubinterfacegui/client");
        // need connection
        for (int i=0; i<20; ++i)
		{
            if (yarp::os::NetworkBase::connect("/icubinterfacegui/client","/icubinterfacegui/server")) break;
			yarp::os::Time::delay(1.0);
		}

        yarp::os::Bottle bot;
        yarp::os::Bottle rep;
        bot.addString("GET_CONF");
        mPort.write(bot,rep);

        for (int i=0; i<(int)bot.size(); ++i)
        {
            mNetworks.push_back(new iCubNetworkGui(mRefTreeModel,mRowLev0,*(bot.get(i).asList())));
        }
        
        /////////////////////////////////////////////////////////////////////////////

        //mTreeView.signal_row_activated().connect(sigc::mem_fun(*this,&iCubInterfaceGuiClient::onTreeViewRowActivated));

        show_all_children();
    }

    void run(){}

    ~iCubInterfaceGuiClient()
    {
        for (int i=0; i<(int)mNetworks.size(); ++i)
        {
            delete mNetworks[i];
        }
    }

    void fromBottle(yarp::os::Bottle &bot)
    {
        int i=0;
        int bConfig=bot.get(i++).asInt();

        for (int j=0; i<(int)bot.size(); ++j)
        {
            mNetworks[j]->fromBottle(*(bot.get(i++).asList()));
        }
    }

protected:
    Gtk::VBox mVBox;
    Gtk::TreeView mTreeView;
    Gtk::TreeModel::Row mRowLev0;
    Gtk::ScrolledWindow mScrolledWindow;
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    ModelColumns mColumns;

    yarp::os::Port mPort;
    std::vector<iCubNetwork*> mNetworks;

    void onTreeViewRowActivated(const Gtk::TreeModel::Path& path,Gtk::TreeViewColumn* /* column */)
    {
        Gtk::TreeModel::iterator iter=mRefTreeModel->get_iter(path);
        
        if (iter)
        {
            Gtk::TreeModel::Row row=*iter;
            //std::cout << "Row activated: ID=" << row[gColumns.mColValue] << ", Name=" << row[g_columns.m_colName] << std::endl;
        }
    }
};

#endif
