// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __GTKMM_ICUB_INTERFACE_GUI_CLIENT_H__
#define __GTKMM_ICUB_INTERFACE_GUI_CLIENT_H__

#include <gtkmm.h>
#include <yarp/os/all.h>
#include "iCubNetworkGui.h"

class iCubInterfaceGuiThread : public yarp::os::RateThread
{
public:
    iCubInterfaceGuiThread() : RateThread(1000)
    {
    }

    void run()
    {
        mSigWindow();
    }

    Glib::Dispatcher mSigWindow;
};

class iCubInterfaceGuiWindow : public Gtk::Window
{
public:
    iCubInterfaceGuiWindow()
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

        //mTreeView.set_reorderable();
        //mTreeView.set_double_buffered(false);

        ////////////////////////////

	    //Add the TreeView's view columns:
        
        mColumns.mColIconID=mTreeView.append_column("",mColumns.mColIcon)-1;
        mColumns.mColNameID=mTreeView.append_column("",mColumns.mColName)-1;
        mColumns.mColValueID=mTreeView.append_column("",mColumns.mColValue)-1;

        /*
        Gtk::CellRendererPixbuf* cell=Gtk::manage(new Gtk::CellRendererPixbuf);
        mColumns.mColIconID=mTreeView.append_column("",*cell)-1;
        Gtk::TreeViewColumn* pColumn=mTreeView.get_column(mColumns.mColIconID);
        pColumn->add_attribute(cell->property_pixbuf(),mColumns.mColIcon);
        */

        //Fill the TreeView's model
        mRowLev0=*(mRefTreeModel->append());
        mRowLev0[mColumns.mColName]="Networks"; //partName.c_str();
        mRowLev0[mColumns.mColValue]=""; //partName.c_str();
        mRowLev0[mColumns.mColIcon]="";
        //mRowLev0[mColumns.mColIcon]=Glib::RefPtr<Gdk::Pixbuf>(NULL);
        //mRowLev0[mColumns.mColIcon]=Gdk::Pixbuf::create_from_file("C:/Documents and Settings/Administrator/My Documents/IIT/iCub/app/iCubGenova01/conf/warning.png");
        
        /////////////////////////////////////////////////////////////////////////////

        //mTreeView.signal_row_activated().connect(sigc::mem_fun(*this,&iCubInterfaceGuiClient::onTreeViewRowActivated));

        //show_all_children();
        show_all();

        yarp::os::Time::delay(1.0);

        mPort.open("/icubinterfacegui/client");
        
        int att;
        const int MAX_ATT=10;

        // need connection
        for (att=0; att<MAX_ATT; ++att)
		{
            if (yarp::os::NetworkBase::connect("/icubinterfacegui/client","/icubinterfacegui/server")) 
            {            
                break;
            }

            yarp::os::Time::delay(1.0);
		}

        if (att==MAX_ATT)
        {
            printf("ERROR: no connection\n");
            return;
        }

        yarp::os::Bottle bot;
        yarp::os::Bottle rep;
        bot.addString("GET_CONF");
        mPort.write(bot,rep);

        //printf("%s\n",rep.toString().c_str());
        //fflush(stdout);

        for (int i=1; i<(int)rep.size(); ++i)
        {
            yarp::os::Bottle *list=rep.get(i).asList();

            if (list)
            {
                mNetworks.push_back(new iCubNetworkGui(mRefTreeModel,mRowLev0,*(rep.get(i).asList())));
            }
        }

        if (hasAlarm())
        {
            mRowLev0[mColumns.mColIcon]="(!)";
        }
        else
        {
            mRowLev0[mColumns.mColIcon]="";
        }

        mThread=new iCubInterfaceGuiThread();

        mThread->mSigWindow.connect(sigc::mem_fun(*this,&iCubInterfaceGuiWindow::run));

        mThread->start();
    }

    virtual ~iCubInterfaceGuiWindow()
    {
        if (mThread!=NULL)
        {
            mThread->stop();
            delete mThread;
            mThread=NULL;
        }

        mPort.interrupt();
        mPort.close();
    }

    void run()
    {
        yarp::os::Bottle bot;
        yarp::os::Bottle rep;
        bot.addString("GET_DATA");
        
        if (mPort.write(bot,rep))
        {
            //printf("%s\n",rep.toString().c_str());
            //fflush(stdout);

            if (rep.size()>1)
            {
                fromBottle(rep);
            }

            if (hasAlarm())
            {
                mRowLev0[mColumns.mColIcon]="(!)";
            }
            else
            {
                mRowLev0[mColumns.mColIcon]="";
            }
        }
    }

    void fromBottle(yarp::os::Bottle &bot)
    {
        for (int i=1; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *net=bot.get(i).asList();
            mNetworks[net->get(0).asInt()]->fromBottle(*net);
        }
    }

    bool hasAlarm()
    {
        bool alarm=false;

        for (int i=0; i<(int)mNetworks.size(); ++i)
        {
            if (mNetworks[i]->hasAlarm())
            {
                alarm=true;
            }
        }

        return alarm;
    }

protected:
    Gtk::VBox mVBox;
    Gtk::TreeView mTreeView;
    Gtk::TreeModel::Row mRowLev0;
    Gtk::ScrolledWindow mScrolledWindow;
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    ModelColumns mColumns;

    iCubInterfaceGuiThread* mThread;

    yarp::os::RpcClient mPort;
    std::vector<iCubNetwork*> mNetworks;
};

#endif

