#include "drummingGui.h"
#include <iostream>
using namespace std;
using namespace Gtk;

#include <gtkmm.h>

DrummingGui::DrummingGui() 
{
    set_title(WINDOW_TITLE);
    set_default_size(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);

    drummingControl.Init();

	builder = LoadFromFile();
	if(!builder)
	{
		hide();
	}

	Gtk::VBox *mainBox = 0;
	builder->get_widget("boxMain", mainBox);

	add(*mainBox);

	show_all_children();

    GetAndConnectWidgets();
    
	UpdatePartitionFolder();
}

DrummingGui::~DrummingGui(void)
{
    
}


void DrummingGui::GetAndConnectWidgets(void)
{
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
    builder->get_widget("btnLeftArmHold", btnLeftArmHold);
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
    builder->get_widget("btnRightArmHold", btnRightArmHold);
    if(btnRightArmHold)
    {
		btnRightArmHold->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnRightArmHoldClicked));
    }

    Gtk::Button* btnLeftLeg = 0;
    builder->get_widget("btnLeftLeg", btnLeftLeg);
    if(btnLeftLeg)
    {
		btnLeftLeg->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnLeftLegClicked));
    }

    Gtk::Button* btnLeftLegHold = 0;
    builder->get_widget("btnLeftLegHold", btnLeftLegHold);
    if(btnLeftLegHold)
    {
		btnLeftLegHold->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnLeftLegHoldClicked));
    }

    Gtk::Button* btnRightLeg = 0;
    builder->get_widget("btnRightLeg", btnRightLeg);
    if(btnRightLeg)
    {
		btnRightLeg->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnRightLegClicked));
    }

    Gtk::Button* btnRightLegHold = 0;
    builder->get_widget("btnRightLegHold", btnRightLegHold);
    if(btnRightLegHold)
    {
		btnRightLegHold->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnRightLegHoldClicked));
    }

    Gtk::Button* btnHead1 = 0;
    builder->get_widget("btnHead1", btnHead1);
    if(btnHead1)
    {
		btnHead1->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnHead1Clicked));
    }

	Gtk::Button* btnHead2 = 0;
    builder->get_widget("btnHead2", btnHead2);
    if(btnHead2)
    {
		btnHead2->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnHead2Clicked));
    }

    Gtk::Button* btnHead3 = 0;
    builder->get_widget("btnHead3", btnHead3);
    if(btnHead3)
    {
		btnHead3->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnHead3Clicked));
    }

    Gtk::Button* btnHead4 = 0;
    builder->get_widget("btnHead4", btnHead4);
    if(btnHead4)
    {
		btnHead4->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnHead4Clicked));
    }

	Gtk::Button* btnHeadHold = 0;
    builder->get_widget("btnHeadHold", btnHeadHold);
    if(btnHeadHold)
    {
		btnHeadHold->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnHeadHoldClicked));
    }

    Gtk::Button* btnHeadScan = 0;
    builder->get_widget("btnHeadScan", btnHeadScan);
    if(btnHeadScan)
    {
		btnHeadScan->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnHeadScanClicked));
    }

    hscTempo = 0;
    builder->get_widget("hscTempo", hscTempo);
    if(hscTempo)
    {
        hscTempo->signal_value_changed().connect(sigc::mem_fun(*this, &DrummingGui::OnHscTempoValueChanged));
        hscTempo->set_value(0.5);
    }

    hscCoordArms = 0;
    builder->get_widget("hscCoordArms", hscCoordArms);
    if(hscCoordArms)
    {
        hscCoordArms->signal_value_changed().connect(sigc::mem_fun(*this, &DrummingGui::OnHscCoordArmsValueChanged));
    }

    hscCoordLegs = 0;
    builder->get_widget("hscCoordLegs", hscCoordLegs);
    if(hscCoordLegs)
    {
        hscCoordLegs->signal_value_changed().connect(sigc::mem_fun(*this, &DrummingGui::OnHscCoordLegsValueChanged));
    }

    hscCoordArmsLegs = 0;
    builder->get_widget("hscCoordArmsLegs", hscCoordArmsLegs);
    if(hscCoordArmsLegs)
    {
        hscCoordArmsLegs->signal_value_changed().connect(sigc::mem_fun(*this, &DrummingGui::OnHscCoordArmsLegsValueChanged));
    }

    Gtk::Button* btnPlayScore = 0;
    builder->get_widget("btnPlayScore", btnPlayScore);
    if(btnPlayScore)
    {
		btnPlayScore->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnPlayScoreClicked));
    }

    Gtk::Button* btnStopScore = 0;
    builder->get_widget("btnStopScore", btnStopScore);
    if(btnStopScore)
    {
		btnStopScore->signal_clicked().connect(sigc::mem_fun(*this, &DrummingGui::OnBtnStopScoreClicked));
    }


	
    builder->get_widget("tvwScores", tvwScores);
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
#ifdef _DEBUG
    cout << "Clicked btnQuit" << endl;
#endif
	drummingControl.Close();
	hide();
}

void DrummingGui::OnBtnStopCustomClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnStopCustom" << endl;
#endif
	drummingControl.StopCustomPlay();
}

void DrummingGui::OnBtnPlayCustomClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnPlayCustom" << endl;
#endif
	drummingControl.PlayAllPartsCustom();
}

void DrummingGui::OnBtnLeftArm1Clicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnLeftArm1" << endl;
#endif
	drummingControl.PlayPartCustom(LEFT_ARM, 1);
}

void DrummingGui::OnBtnLeftArm2Clicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnLeftArm2" << endl;
#endif
	drummingControl.PlayPartCustom(LEFT_ARM, 2);
}

void DrummingGui::OnBtnLeftArmHoldClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnLeftArmHold" << endl;
#endif
	drummingControl.PlayPartCustom(LEFT_ARM, 0);
}

void DrummingGui::OnBtnRightArm1Clicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnRightArm1" << endl;
#endif
	drummingControl.PlayPartCustom(RIGHT_ARM, 1);
}

void DrummingGui::OnBtnRightArm2Clicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnRightArm2" << endl;
#endif
	drummingControl.PlayPartCustom(RIGHT_ARM, 2);
}

void DrummingGui::OnBtnRightArmHoldClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnRightHold" << endl;
#endif
	drummingControl.PlayPartCustom(RIGHT_ARM, 0);
}

void DrummingGui::OnBtnLeftLegClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnLeftLeg" << endl;
#endif
    drummingControl.PlayPartCustom(LEFT_LEG, 1);
}

void DrummingGui::OnBtnLeftLegHoldClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnLeftHold" << endl;
#endif
    drummingControl.PlayPartCustom(LEFT_LEG, 0);
}

void DrummingGui::OnBtnRightLegClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnRightLeg" << endl;
#endif
    drummingControl.PlayPartCustom(RIGHT_LEG, 1);
}

void DrummingGui::OnBtnRightLegHoldClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnRightLegHold" << endl;
#endif
    drummingControl.PlayPartCustom(RIGHT_LEG, 0);
}

void DrummingGui::OnBtnHead1Clicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnHead1" << endl;
#endif
    drummingControl.PlayPartCustom(HEAD, 1);
}

void DrummingGui::OnBtnHead2Clicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnHead2" << endl;
#endif
    drummingControl.PlayPartCustom(HEAD, 2);
}

void DrummingGui::OnBtnHead3Clicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnHead3" << endl;
#endif
    drummingControl.PlayPartCustom(HEAD, 3);
}

void DrummingGui::OnBtnHead4Clicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnHead4" << endl;
#endif
    drummingControl.PlayPartCustom(HEAD, 4);
}

void DrummingGui::OnBtnHeadHoldClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnHeadHold" << endl;
#endif
    drummingControl.PlayPartCustom(HEAD, 0);
}

void DrummingGui::OnBtnHeadScanClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnHeadScan" << endl;
#endif
    drummingControl.PlayPartCustom(HEAD, 5);
}

void DrummingGui::OnHscTempoValueChanged(void)
{
#ifdef _DEBUG
    cout << "Changed hscTempo" << endl;
#endif
    drummingControl.SetTempo(hscTempo->get_value());
    drummingControl.SendParameters();
}

void DrummingGui::OnHscCoordArmsValueChanged(void)
{
#ifdef _DEBUG
    cout << "Changed hscCoordsArms" << endl;
#endif
    drummingControl.SetPhase(0, hscCoordArms->get_value() * M_PI);
    drummingControl.SendParameters();
}

void DrummingGui::OnHscCoordLegsValueChanged(void)
{
#ifdef _DEBUG
    cout << "Changed hscCoordLegs" << endl;
#endif
    drummingControl.SetPhase(1, hscCoordLegs->get_value() * M_PI);
    drummingControl.SendParameters();
}

void DrummingGui::OnHscCoordArmsLegsValueChanged(void)
{
#ifdef _DEBUG
    cout << "Changed hscCoordsArmsLegs" << endl;
#endif
    drummingControl.SetPhase(2, hscCoordArmsLegs->get_value() * M_PI);
    drummingControl.SendParameters();
}


void DrummingGui::OnBtnPlayScoreClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnPlayScore" << endl;
#endif

    //TODO : handle the treeview thingy
    drummingControl.PlayPartition("part1.csv");
}

void DrummingGui::OnBtnStopScoreClicked(void)
{
#ifdef _DEBUG
    cout << "Clicked btnStopScore" << endl;
#endif
    drummingControl.StopPartition();
}