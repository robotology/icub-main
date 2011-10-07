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
    void onTreeViewRowActivated(const Gtk::TreeModel::Path &path,Gtk::TreeViewColumn* column)
    {
        Gtk::TreeModel::iterator iter=mRefTreeModel->get_iter(path);
        const Gtk::TreeModel::Row row=*iter;
        findAndReset(row);
    }

    iCubInterfaceGuiWindow(yarp::os::ResourceFinder& config)
    {
        GuiRawData::setIcons(config);

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
        /*
        Gtk::CellRendererPixbuf* cellIcon=Gtk::manage(new Gtk::CellRendererPixbuf);
        mColumns.mColIconID=mTreeView.append_column("",*cellIcon)-1;
        Gtk::TreeViewColumn* pColumnIcon=mTreeView.get_column(mColumns.mColIconID);
        pColumn->add_attribute(cell->property_pixbuf(),mColumns.mColIcon);
        */

        //mColumns.mColNameID=mTreeView.append_column("",mColumns.mColName)-1;
        Gtk::CellRendererText* cellName=Gtk::manage(new Gtk::CellRendererText);
        mColumns.mColNameID=mTreeView.append_column("",*cellName)-1;
        Gtk::TreeViewColumn* pColumnName=mTreeView.get_column(mColumns.mColNameID);
        pColumnName->add_attribute(cellName->property_text(),mColumns.mColName);
        pColumnName->add_attribute(cellName->property_background_gdk(),mColumns.mColColor);
        pColumnName->add_attribute(cellName->property_foreground_gdk(),mColumns.mColColorFg);

        //mColumns.mColValueID=mTreeView.append_column("",mColumns.mColValue)-1;
        Gtk::CellRendererText* cellValue=Gtk::manage(new Gtk::CellRendererText);
        mColumns.mColValueID=mTreeView.append_column("",*cellValue)-1;
        Gtk::TreeViewColumn* pColumnValue=mTreeView.get_column(mColumns.mColValueID);
        pColumnValue->add_attribute(cellValue->property_text(),mColumns.mColValue);
        pColumnValue->add_attribute(cellValue->property_background_gdk(),mColumns.mColColor);
        pColumnValue->add_attribute(cellValue->property_foreground_gdk(),mColumns.mColColorFg);

        //Fill the TreeView's model
        mRowLev0=*(mRefTreeModel->append());
        mRowLev0[mColumns.mColName]="Networks";
        mRowLev0[mColumns.mColColor]=GuiRawData::mColorEmpty;

        mRowLev0[mColumns.mColValue]="";
        mRowLev0[mColumns.mColIcon]=GuiRawData::mIconEmpty;
        //mRowLev0[mColumns.mColIcon]=Glib::RefPtr<Gdk::Pixbuf>(NULL);
        
        /////////////////////////////////////////////////////////////////////////////

        mTreeView.signal_row_activated().connect(sigc::mem_fun(*this,&iCubInterfaceGuiWindow::onTreeViewRowActivated));
        //show_all_children();

        show_all();

        yarp::os::Time::delay(1.0);

        mWaitConnection=true;
        mPort.open("/icubinterfacegui/client");
        
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

        for (int i=0; i<(int)mNetworks.size(); ++i)
        {
            if (mNetworks[i])
            {
                delete mNetworks[i];
                mNetworks[i]=NULL;
            }
        }
    }

    void run()
    {
        if (mPort.getOutputCount()<=0)
        {
            mWaitConnection=true;
            return;
        }

        if (mWaitConnection)
        {
            mWaitConnection=false;

            yarp::os::Bottle config;
            yarp::os::Bottle get;
            get.addString("GET_CONF");
            mPort.write(get,config);

            for (int i=0; i<(int)config.size(); ++i)
            {
                yarp::os::Bottle *netConf=config.get(i).asList();

                if (netConf!=NULL)
                {
                    mNetworks.push_back(new iCubNetworkGui(mRefTreeModel,mRowLev0,*netConf));
                }
            }
            
            return;
        }

        yarp::os::Bottle bot;
        yarp::os::Bottle rep;
        bot.addString("GET_DATA");
        
        if (mPort.write(bot,rep))
        {
            if (rep.size()>0)
            {
                fromBottle(rep);
                alarmLevel();
            }
        }
    }

    void fromBottle(yarp::os::Bottle &bot)
    {
        for (int i=0; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *net=bot.get(i).asList();
            mNetworks[net->get(0).asInt()]->fromBottle(net->tail());
        }
    }

    void alarmLevel()
    {
        int alarm=0;

        for (int i=0; i<(int)mNetworks.size(); ++i)
        {
            if (mNetworks[i])
            {
                int childAlarmLevel=mNetworks[i]->alarmLevel();
                
                if (childAlarmLevel>alarm)
                {
                    alarm=childAlarmLevel;
                }
            }
        }

        switch(alarm)
        {
        case 0:
            mRowLev0[mColumns.mColIcon]=GuiRawData::mIconEmpty;
            mRowLev0[mColumns.mColColor]=GuiRawData::mColorEmpty;
            mRowLev0[mColumns.mColColorFg]=GuiRawData::mColorBlack;
            break;
        case 1:
            mRowLev0[mColumns.mColIcon]=GuiRawData::mIconWarning;
            mRowLev0[mColumns.mColColor]=GuiRawData::mColorWarning;
            mRowLev0[mColumns.mColColorFg]=GuiRawData::mColorBlack;
            break;
        case 2:
            mRowLev0[mColumns.mColIcon]=GuiRawData::mIconError;
            mRowLev0[mColumns.mColColor]=GuiRawData::mColorError;
            mRowLev0[mColumns.mColColorFg]=GuiRawData::mColorEmpty;
            break;
        }
    }

    void findAndReset(const Gtk::TreeModel::Row row)
    {
        if (row==mRowLev0)
        {
            reset();
            return;
        }
        
        for (int i=0; i<(int)mNetworks.size(); ++i)
        {
            if (mNetworks[i])
            {
                if (*(mNetworks[i]->getRoot())==row)
                {
                    mNetworks[i]->reset();
                    return;
                }
                else
                {
                    if (mNetworks[i]->findAndReset(row)) return;
                }
            }
        }
    }

    void reset()
    {
        for (int i=0; i<(int)mNetworks.size(); ++i)
        {
            if (mNetworks[i])
            {
                mNetworks[i]->reset();
            }
        }
    }

protected:
    Gtk::VBox mVBox;
    Gtk::TreeView mTreeView;
    Gtk::TreeModel::Row mRowLev0;
    Gtk::ScrolledWindow mScrolledWindow;
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    ModelColumns mColumns;

    iCubInterfaceGuiThread* mThread;

    bool mWaitConnection;
    yarp::os::RpcClient mPort;
    std::vector<iCubNetworkGui*> mNetworks;
};

#endif

