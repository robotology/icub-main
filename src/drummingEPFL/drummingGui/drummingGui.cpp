#include "drummingGui.h"
#include <iostream>
using namespace std;
using namespace Gtk;

#include <gtkmm.h>

DrummingGui::DrummingGui() 
{
    set_title(WINDOW_TITLE);
    set_default_size(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);

	builder = LoadFromFile();
	if(!builder)
	{
		hide();
	}

	Gtk::VBox *mainBox = 0;
	builder->get_widget("boxMain", mainBox);

	add(*mainBox);

	show_all_children();

	
	Gtk::Button* btnQuit = 0;
    builder->get_widget("btnQuit", btnQuit);
    if(btnQuit)
    {
		btnQuit->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnQuitClicked));
    }

	Gtk::Button* btnStopCustom = 0;
    builder->get_widget("btnStopCustom", btnStopCustom);
    if(btnStopCustom)
    {
		btnStopCustom->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnStopCustomClicked));
    }

	Gtk::Button* btnPlayCustom = 0;
    builder->get_widget("btnPlayCustom", btnPlayCustom);
    if(btnPlayCustom)
    {
		btnPlayCustom->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnPlayCustomClicked));
    }

	Gtk::Button* btnLeftArm1 = 0;
    builder->get_widget("btnLeftArm1", btnLeftArm1);
    if(btnLeftArm1)
    {
		btnLeftArm1->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnLeftArm1Clicked));
    }

	Gtk::Button* btnLeftArm2 = 0;
    builder->get_widget("btnLeftArm2", btnLeftArm2);
    if(btnLeftArm2)
    {
		btnLeftArm2->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnLeftArm2Clicked));
    }

	Gtk::Button* btnLeftArmHold = 0;
    builder->get_widget("btnLeftArm2", btnLeftArmHold);
    if(btnLeftArmHold)
    {
		btnLeftArmHold->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnLeftArmHoldClicked));
    }


	Gtk::Button* btnRightArm1 = 0;
    builder->get_widget("btnRightArm1", btnRightArm1);
    if(btnRightArm1)
    {
		btnRightArm1->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnRightArm1Clicked));
    }

	Gtk::Button* btnRightArm2 = 0;
    builder->get_widget("btnRightArm2", btnRightArm2);
    if(btnRightArm2)
    {
		btnRightArm2->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnRightArm2Clicked));
    }

	Gtk::Button* btnRightArmHold = 0;
    builder->get_widget("btnRightArm2", btnRightArmHold);
    if(btnRightArmHold)
    {
		btnRightArmHold->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnRightArmHoldClicked));
    }

	
    builder->get_widget("tvwScores", tvwScores);
    
	UpdatePartitionFolder();

	drummingControl.Init();
}

DrummingGui::~DrummingGui(void)
{
}




Glib::RefPtr<Gtk::Builder> DrummingGui::LoadFromFile()
{
	Glib::RefPtr<Gtk::Builder> refBuilder = Gtk::Builder::create();
	
#ifdef GLIBMM_EXCEPTIONS_ENABLED
	try
	{
		refBuilder->add_from_file(GUI_FILE);
	}
	catch(const Glib::FileError& ex)
	{
		std::cerr << "FileError: " << ex.what() << std::endl;
	}
		catch(const Gtk::BuilderError& ex)
	{
		std::cerr << "BuilderError: " << ex.what() << std::endl;
	}	
#else
	std::auto_ptr<Glib::Error> error;

	if (!refBuilder->add_from_file(GUI_FILE, error))
	{
		std::cerr << error->what() << std::endl;
	}
#endif /* !GLIBMM_EXCEPTIONS_ENABLED */ 

	return refBuilder;
}

void DrummingGui::UpdatePartitionFolder(void)
{
	Glib::Dir dir (PARTITION_FOLDER);

	cout << "stuff" << endl;
	for(Glib::Dir::iterator p = dir.begin(); p != dir.end(); ++p)
	{
		const std::string path = Glib::build_filename(PARTITION_FOLDER, *p);
		
		if (path.find(".csv") != string::npos)
		{
			Glib::RefPtr<Glib::Object> obj = builder->get_object("lstScores"); 

			Glib::RefPtr<Gtk::ListStore> liststore = Glib::RefPtr<Gtk::ListStore>::cast_static(obj); 

			cout << "ncol:"<<liststore->get_n_columns()<<endl;

			//Gtk::TreeModel m_col_fileName = liststore->get_column_type(0);
			
			Gtk::TreeModelColumn<Glib::ustring> file_name;

			
			Gtk::TreeModel::Row row; 
			row = *(liststore->append());
			Glib::ustring value = path;
			row->set_value(0, value); 
		}
	}
}


void DrummingGui::OnBtnQuitClicked()
{
	drummingControl.Close();
	hide();
}

void DrummingGui::OnBtnStopCustomClicked(void)
{
	drummingControl.StopCustomPlay();
}

void DrummingGui::OnBtnPlayCustomClicked(void)
{
	drummingControl.PlayAllPartsCustom();
}

void DrummingGui::OnBtnLeftArm1Clicked(void)
{
	drummingControl.PlayPartCustom(0, 1);
}

void DrummingGui::OnBtnLeftArm2Clicked(void)
{
	drummingControl.PlayPartCustom(0, 2);
}

void DrummingGui::OnBtnLeftArmHoldClicked(void)
{
	drummingControl.PlayPartCustom(0, 0);
}

void DrummingGui::OnBtnRightArm1Clicked(void)
{
	drummingControl.PlayPartCustom(1, 1);
}

void DrummingGui::OnBtnRightArm2Clicked(void)
{
	drummingControl.PlayPartCustom(1, 2);
}

void DrummingGui::OnBtnRightArmHoldClicked(void)
{
	drummingControl.PlayPartCustom(1, 0);
}