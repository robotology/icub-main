#include <gtkmm.h>
#include <yarp/os/Network.h>

#include "iCubInterfaceGuiClient.h"

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;

    Gtk::Main kit(argc, argv);

	iCubInterfaceGuiClient client;

    client.set_size_request(320,480);
	client.move(64,64);

    //window.start();

	Gtk::Main::run(client);

    //window.stop();

    return 0;
}

/*
///////////////////////////////////////////


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
*/
