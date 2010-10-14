#include <iostream>
#include "iCubInterfaceGuiClient.h"

#ifdef notdef
iCubInterfaceGui::iCubInterfaceGui(yarp::os::Property &robot)
{
    yarp::os::Bottle general=robot.findGroup("GENERAL");
    yarp::os::Bottle *parts=general.find("parts").asList();

    for (int t=0; t<parts->size(); ++t)
    {
        yarp::os::ConstString partName=parts->get(t).asString();
        yarp::os::Bottle part=robot.findGroup(partName.c_str());
            
        yarp::os::Bottle *networks=part.find("networks").asList();

        for (int n=0; n<networks->size(); ++n)
        {
            yarp::os::ConstString netName=networks->get(n).asString();
            yarp::os::Bottle net=robot.findGroup(netName.c_str());

            addNetwork(new iCubNetwork(std::string(netName.c_str()),
                                   std::string(net.find("file").asString().c_str()),
                                   std::string(net.find("device").asString().c_str()),
                                   std::string(net.find("canbusdevice").asString().c_str()),
                                   part.find("threadrate").asInt()));
        }
    }

    // we have now the networks list

    set_title((yarp::os::ConstString("iCubInterface GUI - ")+robot.find("name").asString()).c_str());
    set_border_width(5);
    set_default_size(600,400);

    add(m_VBox);
    
    //Add the TreeView, inside a ScrolledWindow, with the button underneath:
    m_scrolledWindow.add(m_treeView);
    //Only show the scrollbars when they are necessary:
    m_scrolledWindow.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);

    m_VBox.pack_start(m_scrolledWindow);

    //Create the Tree model:
    m_treeView.set_model(m_refTreeModel=Gtk::TreeStore::create(g_columns));
    //All the items to be reordered with drag-and-drop:
    m_treeView.set_reorderable();

    Gtk::CellRendererPixbuf* cell=Gtk::manage(new Gtk::CellRendererPixbuf);
    
    ///////////////////////////////////////////////////////////////////////////

    int cols_count=m_treeView.append_column("Status",*cell);
    Gtk::TreeViewColumn* pColumn=m_treeView.get_column(cols_count-1);
    pColumn->add_attribute(cell->property_pixbuf(),g_columns.m_colStatus);

	//Add the TreeView's view columns:
    m_treeView.append_column("Name",g_columns.m_colName);
    m_treeView.append_column("ID",g_columns.m_colValue);
    //m_treeView.append_column("Status",g_columns.m_colStatus);

    //Fill the TreeView's model
    Gtk::TreeModel::Row rowLev0;
    rowLev0=*(m_refTreeModel->append());
    rowLev0[g_columns.m_colName]="Networks"; //partName.c_str();
    rowLev0[g_columns.m_colValue]=""; //partName.c_str();
    //rowLev0[m_Columns.m_colStatus]=Gdk::Pixbuf::create_from_file("delete.png");

    for (unsigned int n=0; n<m_networks.size(); ++n)
    {
        yarp::os::Property netConf;
        netConf.fromConfigFile(m_networks[n]->file.c_str());
        yarp::os::Bottle canConf=netConf.findGroup("CAN");

        m_networks[n]->configGui(m_refTreeModel,rowLev0,canConf.find("CanDeviceNum").asInt());

        /////////////////////////////////////////////////////////////
        
        yarp::os::Bottle devices=canConf.findGroup("CanAddresses");
        for (int d=1; d<devices.size(); ++d)
        {
            m_networks[n]->addBoard(devices.get(d).asInt());
        }
    }

    //Connect signal:
    m_treeView.signal_row_activated().connect(sigc::mem_fun(*this,&iCubInterfaceGui::on_treeview_row_activated) );

    show_all_children();
}

iCubInterfaceGui::~iCubInterfaceGui()
{
}

void iCubInterfaceGui::on_treeview_row_activated(const Gtk::TreeModel::Path& path,Gtk::TreeViewColumn* /* column */)
{
    Gtk::TreeModel::iterator iter = m_refTreeModel->get_iter(path);
    if(iter)
    {
        Gtk::TreeModel::Row row = *iter;
        std::cout << "Row activated: ID=" << row[g_columns.m_colValue] << ", Name=" << row[g_columns.m_colName] << std::endl;
    }
}
#endif