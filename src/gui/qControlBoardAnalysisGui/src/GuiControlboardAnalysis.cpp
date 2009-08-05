/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/GuiControlboardAnalysis.h>

GuiControlboardAnalysis::GuiControlboardAnalysis(Property propControlboard, QWidget* parent, const char* name, bool modal, WFlags fl)
  : GuiControlboardAnalysisBase( parent, name, fl )
{
  lneRefreshTime->setValidator(new QIntValidator(this));
    
  // configuration / properties
  cout << "property axis: " << propControlboard.toString() << endl;

  _refreshTime = propControlboard.check("refreshTime",yarp::os::Value(25),
					"Update time in miliseconds (int).").asInt();
  bool chbPos = (bool)propControlboard.check("togglePosition", yarp::os::Value(1),
					     "Set checkbox 'position' (int 0/1).").asInt();
  bool chbVel = (bool)propControlboard.check("toggleVelocity", yarp::os::Value(1),
					     "Set checkbox 'velocity' (int 0/1).").asInt();
  bool chbAcc = (bool)propControlboard.check("toggleAcceleration", yarp::os::Value(1),
					     "Set checkbox 'acceleration' (int 0/1).").asInt();
  bool chbErr = (bool)propControlboard.check("toggleError", yarp::os::Value(1),
					     "Set checkbox 'error' (int 0/1).").asInt();
  bool chbOut = (bool)propControlboard.check("toggleOutput", yarp::os::Value(1),
					     "Set checkbox 'output' (int 0/1).").asInt();
  double scalePos = propControlboard.check("scalePosition", yarp::os::Value(1.0),
					   "Set position scaling (double > 0.0).").asDouble();
  double scaleVel = propControlboard.check("scaleVelocity", yarp::os::Value(1.0),
					   "Set velocity scaling (double > 0.0).").asDouble();
  double scaleAcc = propControlboard.check("scaleAcceleration", yarp::os::Value(1.0),
					   "Set acceleration scaling (double > 0.0).").asDouble();
  double scaleErr = propControlboard.check("scaleError", yarp::os::Value(1.0),
					   "Set error scaling (double > 0.0).").asDouble();
  double scaleOut = propControlboard.check("scaleOutput", yarp::os::Value(1.0),
					   "Set output scaling (double > 0.0).").asDouble();
  bool activateAll = !propControlboard.check("axes");

  frmMainLayout = new QVBoxLayout( frmMain, 11, 6, "frmMainLayout"); 
    
  // scroll view
  _scrlView = new QScrollView(frmMain);
  frmMainLayout->addWidget(_scrlView);
  _scrlView->setResizePolicy(QScrollView::AutoOneFit);

  // vbox for sub configurable buttons
  _scrlBox = new QVBox(_scrlView->viewport());
  _scrlBox->setSpacing(0);
  _scrlBox->setMargin(0);
  _scrlView->addChild(_scrlBox);
    
  // create the controlboard
  _dd.open(propControlboard);
    
  if(!_dd.view(_ienc) || !_dd.view(_ipid))
    cout << "Device does not implement IEncoders or IPidControl!" << endl;
  else{	
    GuiControlboardAnalysisAxis *axisGui;
    _ienc->getAxes(&_numAxes);
    fprintf(stderr, "Initializing number of plots!\n");    
    if (_numAxes > 0 && activateAll){
      _numActiveAxes = _numAxes;
      _activeAxes = new int[_numActiveAxes];
      for (int i =0; i < _numAxes; i++)
	_activeAxes[i] = i;
    }
    else{
      fprintf(stderr, "Getting a reduced number of joints\n");
      if (_numAxes > 0 && !activateAll) {
	Value& axes =  propControlboard.find("axes");
	Bottle *pAxes = axes.asList();
	_numActiveAxes = pAxes->size();
	_activeAxes = new int[_numActiveAxes];
	fprintf(stderr, "Getting the reduced number of joints: %d \n", _numActiveAxes);
	if ( _numActiveAxes > _numAxes || _numActiveAxes == 0)
	  fprintf(stderr, "The 'nJoints' and 'joints' params are incompatible");
	else {
	  for(int i = 0; i < _numActiveAxes; i++)
	    _activeAxes[i] = pAxes->get(i).asInt();
	}
      }
      else
	fprintf(stderr, "Something wrong with the number of axis\n");
    }
  }
  
  fprintf(stderr, "Initializing guis!\n");
  _axisGuis = new GuiControlboardAnalysisAxis*[_numActiveAxes];
  _pos = new double[_numAxes];
  _vel = new double[_numAxes];
  _acc = new double[_numAxes];
  _err = new double[_numAxes];
  _out = new double[_numAxes];
  for (int i = 0; i < _numActiveAxes; i++){
    //frmMainLayout->addWidget(new GuiControlboardAnalysisAxis(_dd, i, frmMain));
    _axisGuis[i] = new GuiControlboardAnalysisAxis(propControlboard, i, _scrlBox);
    _axisGuis[i]->togglePosition(chbPos);
    _axisGuis[i]->toggleVelocity(chbVel);
    _axisGuis[i]->toggleAcceleration(chbAcc);
    _axisGuis[i]->toggleError(chbErr);
    _axisGuis[i]->toggleOutput(chbOut);
    _axisGuis[i]->scalePosition(scalePos);
    _axisGuis[i]->scaleVelocity(scaleVel);
    _axisGuis[i]->scaleAcceleration(scaleAcc);
    _axisGuis[i]->scaleError(scaleErr);
    _axisGuis[i]->scaleOutput(scaleOut);
  }	
  fprintf(stderr, "Initialization is done!\n");
    
  _guiAbout = new GuiCBAAbout();
    
  // position screen center
    QWidget* desk = QApplication::desktop();
    this->move(desk->width()/2 - this->width()/2,desk->height()/2 - this->height()/2);	
	

    // timer for data acquisition
    QVariant refreshTime(_refreshTime);
    lneRefreshTime->setText(refreshTime.asString());
    _timer = new QTimer();
    connect( _timer, SIGNAL(timeout()), this, SLOT(timerdone()) );
    _timeStart = yarp::os::Time::now();
    fprintf(stderr, "Starting the timer\n");
    _timer->start(_refreshTime);
  }

  GuiControlboardAnalysis::~GuiControlboardAnalysis()
  {
    _timer->stop();
    _dd.close();
    delete _timer;
    delete[] _activeAxes;
  }

  void GuiControlboardAnalysis::timerdone(){
 	
    double t = yarp::os::Time::now() - _timeStart;
    _ienc->getEncoders(_pos);
    _ienc->getEncoderSpeeds (_vel);
    _ienc->getEncoderAccelerations(_acc);
    _ipid->getErrors (_err);
    _ipid->getOutputs(_out);

    for (int i = 0; i < _numActiveAxes; i++){
      //cout << "Axis: " << i << ": " << t << "\t " << _pos[i] << "\t " << _vel[i] << "\t " << _acc[i] << endl;
      _axisGuis[i]->put(t, _pos[_activeAxes[i]], _vel[_activeAxes[i]], _acc[_activeAxes[i]], _err[_activeAxes[i]], _out[_activeAxes[i]]);
    }
  }

  void GuiControlboardAnalysis::lneRefreshTime_returnPressed(){
    QVariant refreshTime(lneRefreshTime->text());
    _timer->changeInterval(refreshTime.asInt());
    _refreshTime = refreshTime.asInt();
  }

  void GuiControlboardAnalysis::btnQuit_clicked()
  {
    this->close();
  }

  void GuiControlboardAnalysis::btnAbout_clicked(){
    _guiAbout->show();
  }


  void printHelp(){

    cout << endl;
    cout << "Help:" << endl;
    cout << "To connect the qcontrolboardanalysisgui to a server control board"  << endl;
    cout << "the port of the server control board has to be specified by" << endl;
    cout << "the commandline option '--remote'. Optionally a subset of the control" << endl;
    cout << "board axes can be specified with the option --axes" << endl;
    cout << "Example: " << endl;
    cout << "$> guicontrolboardanalysisgui --remote /port/of/servercontrolboard" << endl;
    cout << endl;
    cout << "Optionally you can specify the local port of the remote control" << endl;
    cout << "board as well." << endl;
    cout << "Example: " << endl;
    cout << "$> qcontrolboardanalysisgui --remote /port/of/servercontrolboard --local /port/of/remotecontrolboard"  << endl;
    cout << "Optionally you can specify a subset of controlled axes:" << endl;
    cout << "Example: " << endl;
    cout << "$> guicontrolboardanalysisgui --remote /port/of/servercontrolboard --axes \"(0 1 3)\" "  << endl;
    cout << endl;
  }

  /**
   * @ingroup icub_module
   *
   * \defgroup icub_qcontrolboardanalysisgui qcontrolboardanalysisgui
   *
   * Graphical user interface for observing a yarp control board. \n
   * Run without options to see help on usage. \n
   * \n
   * The module runs a remote control board and plots position, velocity and acceleration
   * for each motor axis. To connect to any server control board pass '--remote /yarp/port/servercontrolboard'
   * at the command line. If you wish to specify the port of the remote_controlboard of
   * qcontrolboardanalysisgui specify something like '--local /yarp/port/remotecontrolboard' at the commandline.\n
   * The server control board has to provide the interface IEncoders.
   *
   *  \image html qcontrolboardanalysisgui.jpg
   *  \image latex qcontrolboardanalysisgui.eps "qcontrolboardanalysisgui running on Linux" width=10cm
   *
   * \see iCub::contrib::GuiControlboardAnalysis
   *
   * \author Jonas Ruesch
   *
   */
 
  int main(int argc, char** argv)
  {

    cout << endl;
    cout << "************************************" << endl;
    cout << "***** QControlBoardAnalysisGui *****" << endl;
    cout << "************************************" << endl;
    
    string remote;
    string local;
    
    // no arguments
    if (argc == 1){
      printHelp();
      return 0;
    }
    
    // Property
    Property propBoard;
    Property propCmd;
    propCmd.fromCommand(argc,argv);
    propBoard.fromString(propCmd.toString().c_str());

    // remote port
    if (!propCmd.check("remote")){
      cout << "Server control board port not found. Please specify --remote value." << endl;
      return 0;
    }
    //propBoard.put("remote", propCmd.find("remote").asString().c_str());
    
    // local port
    if (!propCmd.check("local")){
      propBoard.put("local", "/qcontrolboardanalysisgui/controlboard");
    }
    /*else{
      propBoard.put("local", propCmd.find("local").asString().c_str());
      }*/
    
    // controlboard driver
    propBoard.put("device", "remote_controlboard");
    
    // initialize Qt
    QApplication app(argc,argv);
    
    // initialize yarp
    Network::init();
    
    GuiControlboardAnalysis *guiBoard = new GuiControlboardAnalysis(propBoard);
    app.setMainWidget(guiBoard);
    guiBoard->show();
    
    Network::fini();
    return app.exec();
    
  }


