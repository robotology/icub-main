/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/GuiControlboardAnalysisAxis.h>


GuiControlboardAnalysisAxis::GuiControlboardAnalysisAxis(yarp::os::Searchable &config, int axisIndex, QWidget* parent, const char* name, bool modal, WFlags fl)
	: GuiControlboardAnalysisAxisBase( parent, name, fl )
{
    // constants
    _bufSize = config.check("bufferSize", yarp::os::Value(300),
                            "Number of curve entries (int)").asInt();
    _scalePos = 1.0;
    _scaleVel = 1.0;
    _scaleAcc = 1.0;
    _scaleErr = 1.0;
    _scaleOut = 1.0;
    
    _axisIndex = axisIndex;
    QVariant varIndex(_axisIndex);
    gbxAxis->setTitle("Axis: " + varIndex.asString());

    // layout
    frmMainLayout = new QVBoxLayout( frmMain );
    frmMainLayout->setAlignment( Qt::AlignTop );

    // qwt plot widget
    _plot = new QwtPlot( frmMain, "_plot" );
    _plot->setMaximumSize( QSize( 32767, 200 ) );
    _plot->setMouseTracking( FALSE );
    _plot->setAutoReplot( FALSE );
    _plot->setCanvasBackground( QColor( 255, 255, 255 ) );
    _plot->setProperty( "xBottomAxis", QVariant( TRUE, 0 ) );
    QFont plot_titleFont(  _plot->font() );
    plot_titleFont.setFamily( "DejaVu Sans" );
    plot_titleFont.setPointSize( 12 );
    plot_titleFont.setBold( TRUE );
    _plot->setTitleFont( plot_titleFont );

    // curves
    _curvePos = _plot->insertCurve("position");
    _curveVel = _plot->insertCurve("velocity");
    _curveAcc = _plot->insertCurve("acceleration");
    _curveErr = _plot->insertCurve("error");
    _curveOut = _plot->insertCurve("output");

    // curve appearance
    QwtPlotCurve *_cvPos = _plot->curve(_curvePos);
    _cvPos->setPen(QPen(QColor(255,0,0),2));
    QwtPlotCurve *_cvVel = _plot->curve(_curveVel);
    _cvVel->setPen(QPen(QColor(0,255,0),2));
    QwtPlotCurve *_cvAcc = _plot->curve(_curveAcc);
    _cvAcc->setPen(QPen(QColor(0,0,255),2));
    QwtPlotCurve *_cvErr = _plot->curve(_curveErr);
    _cvErr->setPen(QPen(QColor(255,0,255),2));
    QwtPlotCurve *_cvOut = _plot->curve(_curveOut);
    _cvErr->setPen(QPen(QColor(0,255,255),2));

    // plot legend
    QwtLegend *legend = _plot->legend();
    legend->insertItem(new QwtLegendButton(QwtSymbol(),
        QPen(QColor(255,0,0),2),"position",_plot), _curvePos);
    legend->insertItem(new QwtLegendButton(QwtSymbol(),
        QPen(QColor(0,255,0),2),"velocity",_plot), _curveVel);
    legend->insertItem(new QwtLegendButton(QwtSymbol(),
        QPen(QColor(0,0,255),2),"acceleration", _plot), _curveAcc);
    legend->insertItem(new QwtLegendButton(QwtSymbol(),
        QPen(QColor(255,0,255),2),"error", _plot), _curveErr);
    legend->insertItem(new QwtLegendButton(QwtSymbol(),
        QPen(QColor(0,255,255),2),"output", _plot), _curveOut);
    _plot->setLegendPosition(QwtPlot::Bottom);

    frmMainLayout->addWidget( _plot );

    // init plot buffers
    _bufTime = new double[_bufSize];
    _bufPos = new double[_bufSize];
    _bufVel = new double[_bufSize];
    _bufAcc = new double[_bufSize];
    _bufErr = new double[_bufSize];
    _bufOut = new double[_bufSize];
    for (int i = 0; i < _bufSize; i++){
        _bufTime[i] = 0.0; //0.1*(double)i;
        _bufPos[i] = 0.0;
        _bufVel[i] = 0.0;
        _bufAcc[i] = 0.0;
	_bufErr[i] = 0.0;
	_bufOut[i] = 0.0;
    }

    // curve data
    _plot->setCurveRawData(_curvePos, _bufTime, _bufPos, _bufSize);
    _plot->setCurveRawData(_curveVel, _bufTime, _bufVel, _bufSize);
    _plot->setCurveRawData(_curveAcc, _bufTime, _bufAcc, _bufSize);
    _plot->setCurveRawData(_curveErr, _bufTime, _bufErr, _bufSize);
    _plot->setCurveRawData(_curveOut, _bufTime, _bufOut, _bufSize);
    
    lneScalePos->setValidator(new QDoubleValidator(this));
    lneScaleVel->setValidator(new QDoubleValidator(this));
    lneScaleAcc->setValidator(new QDoubleValidator(this));
    lneScaleErr->setValidator(new QDoubleValidator(this));
    lneScaleOut->setValidator(new QDoubleValidator(this));
}

GuiControlboardAnalysisAxis::~GuiControlboardAnalysisAxis()
{
    delete _plot;
    delete [] _bufTime;
    delete [] _bufPos;
    delete [] _bufVel;
    delete [] _bufAcc;
    delete [] _bufErr;
    delete [] _bufOut;
}

void GuiControlboardAnalysisAxis::put(double time,
                                      double pos, 
                                      double vel, 
                                      double acc,
				      double err,
				      double out){
    // shift array by -1
    memcpy(_bufTime, _bufTime+1, (_bufSize-1)*sizeof(double));
    memcpy(_bufPos, _bufPos+1, (_bufSize-1)*sizeof(double));
    memcpy(_bufVel, _bufVel+1, (_bufSize-1)*sizeof(double));
    memcpy(_bufAcc, _bufAcc+1, (_bufSize-1)*sizeof(double));
    memcpy(_bufErr, _bufErr+1, (_bufSize-1)*sizeof(double));
    memcpy(_bufOut, _bufOut+1, (_bufSize-1)*sizeof(double));

    // assign new value to last array position
    _bufTime[_bufSize-1] = time;
    _bufPos[_bufSize-1] = pos * _scalePos;
    _bufVel[_bufSize-1] = vel * _scaleVel;
    _bufAcc[_bufSize-1] = acc * _scaleAcc;
    _bufErr[_bufSize-1] = err * _scaleErr;
    _bufOut[_bufSize-1] = out * _scaleOut;

    _plot->replot();
}

void GuiControlboardAnalysisAxis::togglePosition(bool state){
    chbPos->setChecked(state);
}

void GuiControlboardAnalysisAxis::toggleVelocity(bool state){
    chbVel->setChecked(state);
}

void GuiControlboardAnalysisAxis::toggleAcceleration(bool state){
    chbAcc->setChecked(state);
}

void GuiControlboardAnalysisAxis::toggleError(bool state){
    chbErr->setChecked(state);
}

void GuiControlboardAnalysisAxis::toggleOutput(bool state){
    chbOut->setChecked(state);
}

void GuiControlboardAnalysisAxis::scalePosition(double scale){
     for (int i = 0; i < _bufSize; i++)
        _bufPos[i] = _bufPos[i] / _scalePos * scale;
    _scalePos = scale;
    QVariant scalePos(scale);
    lneScalePos->setText(scalePos.asString());
}

void GuiControlboardAnalysisAxis::scaleVelocity(double scale){
    for (int i = 0; i < _bufSize; i++)
        _bufVel[i] = _bufVel[i] / _scaleVel * scale;
    _scaleVel = scale;
    QVariant scaleVel(scale);
    lneScaleVel->setText(scaleVel.asString());
}

void GuiControlboardAnalysisAxis::scaleAcceleration(double scale){
    for (int i = 0; i < _bufSize; i++)
        _bufAcc[i] = _bufAcc[i] / _scaleAcc * scale;
    _scaleAcc = scale;
    QVariant scaleAcc(scale);
    lneScaleAcc->setText(scaleAcc.asString());
}

void GuiControlboardAnalysisAxis::scaleError(double scale){
    for (int i = 0; i < _bufSize; i++)
        _bufErr[i] = _bufErr[i] / _scaleErr * scale;
    _scaleErr = scale;
    QVariant scaleErr(scale);
    lneScaleErr->setText(scaleErr.asString());
}

void GuiControlboardAnalysisAxis::scaleOutput(double scale){
    for (int i = 0; i < _bufSize; i++)
        _bufOut[i] = _bufOut[i] / _scaleOut * scale;
    _scaleOut = scale;
    QVariant scaleOut(scale);
    lneScaleOut->setText(scaleOut.asString());
}

void GuiControlboardAnalysisAxis::chbPos_stateChanged( int value )
{
    QwtPlotCurve *_cvPos = _plot->curve(_curvePos);
    if (value == QButton::Off)
        _cvPos->setStyle(QwtCurve::NoCurve);
    if (value == QButton::On)
        _cvPos->setStyle(QwtCurve::Lines);
}


void GuiControlboardAnalysisAxis::chbVel_stateChanged( int value )
{
     QwtPlotCurve *_cvVel = _plot->curve(_curveVel);
    if (value == QButton::Off)
        _cvVel->setStyle(QwtCurve::NoCurve);
    if (value == QButton::On)
        _cvVel->setStyle(QwtCurve::Lines);
   
}


void GuiControlboardAnalysisAxis::chbAcc_stateChanged( int value )
{
     QwtPlotCurve *_cvAcc = _plot->curve(_curveAcc);
    if (value == QButton::Off)
        _cvAcc->setStyle(QwtCurve::NoCurve);
    if (value == QButton::On)
        _cvAcc->setStyle(QwtCurve::Lines);
   
}

void GuiControlboardAnalysisAxis::chbErr_stateChanged( int value )
{
     QwtPlotCurve *_cvErr = _plot->curve(_curveErr);
    if (value == QButton::Off)
        _cvErr->setStyle(QwtCurve::NoCurve);
    if (value == QButton::On)
        _cvErr->setStyle(QwtCurve::Lines);
   
}


void GuiControlboardAnalysisAxis::chbOut_stateChanged( int value )
{
     QwtPlotCurve *_cvOut = _plot->curve(_curveOut);
    if (value == QButton::Off)
        _cvOut->setStyle(QwtCurve::NoCurve);
    if (value == QButton::On)
        _cvOut->setStyle(QwtCurve::Lines);
   
}

void GuiControlboardAnalysisAxis::lneScalePos_returnPressed()
{
    QVariant scalePos(lneScalePos->text());
    scalePosition(scalePos.asDouble());
}


void GuiControlboardAnalysisAxis::lneScaleVel_returnPressed()
{
    QVariant scaleVel(lneScaleVel->text());
    scaleVelocity(scaleVel.asDouble());

}


void GuiControlboardAnalysisAxis::lneScaleAcc_returnPressed()
{
    QVariant scaleAcc(lneScaleAcc->text());
    scaleAcceleration(scaleAcc.asDouble());
}

void GuiControlboardAnalysisAxis::lneScaleErr_returnPressed()
{
    QVariant scaleErr(lneScaleErr->text());
    scaleError(scaleErr.asDouble());
}

void GuiControlboardAnalysisAxis::lneScaleOut_returnPressed()
{
    QVariant scaleOut(lneScaleOut->text());
    scaleOutput(scaleOut.asDouble());
}
