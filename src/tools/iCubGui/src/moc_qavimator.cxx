/****************************************************************************
** qavimator meta object code from reading C++ file 'qavimator.h'
**
** Created: mer 14. lug 13:59:01 2010
**      by: The Qt MOC ($Id: moc_yacc.cpp 2051 2007-02-21 10:04:20Z chehrlic $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "qavimator.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *qavimator::className() const
{
    return "qavimator";
}

QMetaObject *qavimator::metaObj = 0;
static QMetaObjectCleanUp cleanUp_qavimator( "qavimator", &qavimator::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString qavimator::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "qavimator", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString qavimator::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "qavimator", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* qavimator::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QMainWindow::staticMetaObject();
    static const QUMethod slot_0 = {"readSettings", 0, 0 };
    static const QUMethod slot_1 = {"configChanged", 0, 0 };
    static const QUMethod slot_2 = {"backgroundClicked", 0, 0 };
    static const QUMethod slot_3 = {"on_fileNewAction_triggered", 0, 0 };
    static const QUMethod slot_4 = {"on_fileOpenAction_triggered", 0, 0 };
    static const QUMethod slot_5 = {"on_fileSaveAction_triggered", 0, 0 };
    static const QUMethod slot_6 = {"on_fileSaveAsAction_triggered", 0, 0 };
    static const QUMethod slot_7 = {"on_fileExitAction_triggered", 0, 0 };
    static const QUParameter param_slot_8[] = {
	{ "on", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_8 = {"on_optionsJointLimitsAction_toggled", 1, param_slot_8 };
    static const QUParameter param_slot_9[] = {
	{ "on", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_9 = {"on_optionsShowTimelineAction_toggled", 1, param_slot_9 };
    static const QUMethod slot_10 = {"on_optionsConfigureiCubGUIAction_triggered", 0, 0 };
    static const QUMethod slot_11 = {"on_helpAboutAction_triggered", 0, 0 };
    static const QUMethod slot_12 = {"on_resetCameraAction_triggered", 0, 0 };
    static const QUParameter param_slot_13[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_13 = {"on_figureCombo_activated", 1, param_slot_13 };
    static const QUParameter param_slot_14[] = {
	{ "newValue", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_14 = {"on_scaleSpin_valueChanged", 1, param_slot_14 };
    static const QUMethod slot_15 = {"on_xRotationEdit_returnPressed", 0, 0 };
    static const QUMethod slot_16 = {"on_xRotationEdit_lostFocus", 0, 0 };
    static const QUParameter param_slot_17[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_17 = {"on_xRotationSlider_valueChanged", 1, param_slot_17 };
    static const QUMethod slot_18 = {"on_yRotationEdit_returnPressed", 0, 0 };
    static const QUMethod slot_19 = {"on_yRotationEdit_lostFocus", 0, 0 };
    static const QUParameter param_slot_20[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_20 = {"on_yRotationSlider_valueChanged", 1, param_slot_20 };
    static const QUMethod slot_21 = {"on_zRotationEdit_returnPressed", 0, 0 };
    static const QUMethod slot_22 = {"on_zRotationEdit_lostFocus", 0, 0 };
    static const QUParameter param_slot_23[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_23 = {"on_zRotationSlider_valueChanged", 1, param_slot_23 };
    static const QUMethod slot_24 = {"on_xPositionEdit_returnPressed", 0, 0 };
    static const QUMethod slot_25 = {"on_xPositionEdit_lostFocus", 0, 0 };
    static const QUParameter param_slot_26[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_26 = {"on_xPositionSlider_valueChanged", 1, param_slot_26 };
    static const QUMethod slot_27 = {"on_yPositionEdit_returnPressed", 0, 0 };
    static const QUMethod slot_28 = {"on_yPositionEdit_lostFocus", 0, 0 };
    static const QUParameter param_slot_29[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_29 = {"on_yPositionSlider_valueChanged", 1, param_slot_29 };
    static const QUMethod slot_30 = {"on_zPositionEdit_returnPressed", 0, 0 };
    static const QUMethod slot_31 = {"on_zPositionEdit_lostFocus", 0, 0 };
    static const QUParameter param_slot_32[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_32 = {"on_zPositionSlider_valueChanged", 1, param_slot_32 };
    static const QUMethod slot_33 = {"on_playButton_clicked", 0, 0 };
    static const QUParameter param_slot_34[] = {
	{ "num", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_34 = {"on_fpsSpin_valueChanged", 1, param_slot_34 };
    static const QMetaData slot_tbl[] = {
	{ "readSettings()", &slot_0, QMetaData::Protected },
	{ "configChanged()", &slot_1, QMetaData::Protected },
	{ "backgroundClicked()", &slot_2, QMetaData::Protected },
	{ "on_fileNewAction_triggered()", &slot_3, QMetaData::Protected },
	{ "on_fileOpenAction_triggered()", &slot_4, QMetaData::Protected },
	{ "on_fileSaveAction_triggered()", &slot_5, QMetaData::Protected },
	{ "on_fileSaveAsAction_triggered()", &slot_6, QMetaData::Protected },
	{ "on_fileExitAction_triggered()", &slot_7, QMetaData::Protected },
	{ "on_optionsJointLimitsAction_toggled(bool)", &slot_8, QMetaData::Protected },
	{ "on_optionsShowTimelineAction_toggled(bool)", &slot_9, QMetaData::Protected },
	{ "on_optionsConfigureiCubGUIAction_triggered()", &slot_10, QMetaData::Protected },
	{ "on_helpAboutAction_triggered()", &slot_11, QMetaData::Protected },
	{ "on_resetCameraAction_triggered()", &slot_12, QMetaData::Protected },
	{ "on_figureCombo_activated(int)", &slot_13, QMetaData::Protected },
	{ "on_scaleSpin_valueChanged(int)", &slot_14, QMetaData::Protected },
	{ "on_xRotationEdit_returnPressed()", &slot_15, QMetaData::Protected },
	{ "on_xRotationEdit_lostFocus()", &slot_16, QMetaData::Protected },
	{ "on_xRotationSlider_valueChanged(int)", &slot_17, QMetaData::Protected },
	{ "on_yRotationEdit_returnPressed()", &slot_18, QMetaData::Protected },
	{ "on_yRotationEdit_lostFocus()", &slot_19, QMetaData::Protected },
	{ "on_yRotationSlider_valueChanged(int)", &slot_20, QMetaData::Protected },
	{ "on_zRotationEdit_returnPressed()", &slot_21, QMetaData::Protected },
	{ "on_zRotationEdit_lostFocus()", &slot_22, QMetaData::Protected },
	{ "on_zRotationSlider_valueChanged(int)", &slot_23, QMetaData::Protected },
	{ "on_xPositionEdit_returnPressed()", &slot_24, QMetaData::Protected },
	{ "on_xPositionEdit_lostFocus()", &slot_25, QMetaData::Protected },
	{ "on_xPositionSlider_valueChanged(int)", &slot_26, QMetaData::Protected },
	{ "on_yPositionEdit_returnPressed()", &slot_27, QMetaData::Protected },
	{ "on_yPositionEdit_lostFocus()", &slot_28, QMetaData::Protected },
	{ "on_yPositionSlider_valueChanged(int)", &slot_29, QMetaData::Protected },
	{ "on_zPositionEdit_returnPressed()", &slot_30, QMetaData::Protected },
	{ "on_zPositionEdit_lostFocus()", &slot_31, QMetaData::Protected },
	{ "on_zPositionSlider_valueChanged(int)", &slot_32, QMetaData::Protected },
	{ "on_playButton_clicked()", &slot_33, QMetaData::Protected },
	{ "on_fpsSpin_valueChanged(int)", &slot_34, QMetaData::Protected }
    };
    static const QUParameter param_signal_0[] = {
	{ "state", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod signal_0 = {"enableRotation", 1, param_signal_0 };
    static const QUParameter param_signal_1[] = {
	{ "state", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod signal_1 = {"enablePosition", 1, param_signal_1 };
    static const QUParameter param_signal_2[] = {
	{ "state", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod signal_2 = {"enableProps", 1, param_signal_2 };
    static const QUParameter param_signal_3[] = {
	{ "state", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod signal_3 = {"enableEaseInOut", 1, param_signal_3 };
    static const QUMethod signal_4 = {"resetCamera", 0, 0 };
    static const QUParameter param_signal_5[] = {
	{ "state", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod signal_5 = {"protectFrame", 1, param_signal_5 };
    static const QMetaData signal_tbl[] = {
	{ "enableRotation(bool)", &signal_0, QMetaData::Public },
	{ "enablePosition(bool)", &signal_1, QMetaData::Public },
	{ "enableProps(bool)", &signal_2, QMetaData::Public },
	{ "enableEaseInOut(bool)", &signal_3, QMetaData::Public },
	{ "resetCamera()", &signal_4, QMetaData::Public },
	{ "protectFrame(bool)", &signal_5, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"qavimator", parentObject,
	slot_tbl, 35,
	signal_tbl, 6,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_qavimator.setMetaObject( metaObj );
    return metaObj;
}

void* qavimator::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "qavimator" ) )
	return this;
    if ( !qstrcmp( clname, "Ui::MainWindow" ) )
	return (Ui::MainWindow*)this;
    return QMainWindow::qt_cast( clname );
}

// SIGNAL enableRotation
void qavimator::enableRotation( bool t0 )
{
    activate_signal_bool( staticMetaObject()->signalOffset() + 0, t0 );
}

// SIGNAL enablePosition
void qavimator::enablePosition( bool t0 )
{
    activate_signal_bool( staticMetaObject()->signalOffset() + 1, t0 );
}

// SIGNAL enableProps
void qavimator::enableProps( bool t0 )
{
    activate_signal_bool( staticMetaObject()->signalOffset() + 2, t0 );
}

// SIGNAL enableEaseInOut
void qavimator::enableEaseInOut( bool t0 )
{
    activate_signal_bool( staticMetaObject()->signalOffset() + 3, t0 );
}

// SIGNAL resetCamera
void qavimator::resetCamera()
{
    activate_signal( staticMetaObject()->signalOffset() + 4 );
}

// SIGNAL protectFrame
void qavimator::protectFrame( bool t0 )
{
    activate_signal_bool( staticMetaObject()->signalOffset() + 5, t0 );
}

bool qavimator::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: readSettings(); break;
    case 1: configChanged(); break;
    case 2: backgroundClicked(); break;
    case 3: on_fileNewAction_triggered(); break;
    case 4: on_fileOpenAction_triggered(); break;
    case 5: on_fileSaveAction_triggered(); break;
    case 6: on_fileSaveAsAction_triggered(); break;
    case 7: on_fileExitAction_triggered(); break;
    case 8: on_optionsJointLimitsAction_toggled((bool)static_QUType_bool.get(_o+1)); break;
    case 9: on_optionsShowTimelineAction_toggled((bool)static_QUType_bool.get(_o+1)); break;
    case 10: on_optionsConfigureiCubGUIAction_triggered(); break;
    case 11: on_helpAboutAction_triggered(); break;
    case 12: on_resetCameraAction_triggered(); break;
    case 13: on_figureCombo_activated((int)static_QUType_int.get(_o+1)); break;
    case 14: on_scaleSpin_valueChanged((int)static_QUType_int.get(_o+1)); break;
    case 15: on_xRotationEdit_returnPressed(); break;
    case 16: on_xRotationEdit_lostFocus(); break;
    case 17: on_xRotationSlider_valueChanged((int)static_QUType_int.get(_o+1)); break;
    case 18: on_yRotationEdit_returnPressed(); break;
    case 19: on_yRotationEdit_lostFocus(); break;
    case 20: on_yRotationSlider_valueChanged((int)static_QUType_int.get(_o+1)); break;
    case 21: on_zRotationEdit_returnPressed(); break;
    case 22: on_zRotationEdit_lostFocus(); break;
    case 23: on_zRotationSlider_valueChanged((int)static_QUType_int.get(_o+1)); break;
    case 24: on_xPositionEdit_returnPressed(); break;
    case 25: on_xPositionEdit_lostFocus(); break;
    case 26: on_xPositionSlider_valueChanged((int)static_QUType_int.get(_o+1)); break;
    case 27: on_yPositionEdit_returnPressed(); break;
    case 28: on_yPositionEdit_lostFocus(); break;
    case 29: on_yPositionSlider_valueChanged((int)static_QUType_int.get(_o+1)); break;
    case 30: on_zPositionEdit_returnPressed(); break;
    case 31: on_zPositionEdit_lostFocus(); break;
    case 32: on_zPositionSlider_valueChanged((int)static_QUType_int.get(_o+1)); break;
    case 33: on_playButton_clicked(); break;
    case 34: on_fpsSpin_valueChanged((int)static_QUType_int.get(_o+1)); break;
    default:
	return QMainWindow::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool qavimator::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: enableRotation((bool)static_QUType_bool.get(_o+1)); break;
    case 1: enablePosition((bool)static_QUType_bool.get(_o+1)); break;
    case 2: enableProps((bool)static_QUType_bool.get(_o+1)); break;
    case 3: enableEaseInOut((bool)static_QUType_bool.get(_o+1)); break;
    case 4: resetCamera(); break;
    case 5: protectFrame((bool)static_QUType_bool.get(_o+1)); break;
    default:
	return QMainWindow::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool qavimator::qt_property( int id, int f, QVariant* v)
{
    return QMainWindow::qt_property( id, f, v);
}

bool qavimator::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
