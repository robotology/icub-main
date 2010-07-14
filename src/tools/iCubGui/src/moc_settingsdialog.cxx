/****************************************************************************
** SettingsDialog meta object code from reading C++ file 'settingsdialog.h'
**
** Created: mer 14. lug 13:59:01 2010
**      by: The Qt MOC ($Id: moc_yacc.cpp 2051 2007-02-21 10:04:20Z chehrlic $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "settingsdialog.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *SettingsDialog::className() const
{
    return "SettingsDialog";
}

QMetaObject *SettingsDialog::metaObj = 0;
static QMetaObjectCleanUp cleanUp_SettingsDialog( "SettingsDialog", &SettingsDialog::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString SettingsDialog::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "SettingsDialog", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString SettingsDialog::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "SettingsDialog", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* SettingsDialog::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QDialog::staticMetaObject();
    static const QUMethod slot_0 = {"on_applyButton_clicked", 0, 0 };
    static const QUMethod slot_1 = {"on_okButton_clicked", 0, 0 };
    static const QUMethod slot_2 = {"on_cancelButton_clicked", 0, 0 };
    static const QUParameter param_slot_3[] = {
	{ "state", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_3 = {"on_useFogCheckbox_toggled", 1, param_slot_3 };
    static const QUParameter param_slot_4[] = {
	{ "value", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_4 = {"on_floorTranslucencySpin_valueChanged", 1, param_slot_4 };
    static const QMetaData slot_tbl[] = {
	{ "on_applyButton_clicked()", &slot_0, QMetaData::Protected },
	{ "on_okButton_clicked()", &slot_1, QMetaData::Protected },
	{ "on_cancelButton_clicked()", &slot_2, QMetaData::Protected },
	{ "on_useFogCheckbox_toggled(bool)", &slot_3, QMetaData::Protected },
	{ "on_floorTranslucencySpin_valueChanged(int)", &slot_4, QMetaData::Protected }
    };
    static const QUMethod signal_0 = {"configChanged", 0, 0 };
    static const QMetaData signal_tbl[] = {
	{ "configChanged()", &signal_0, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"SettingsDialog", parentObject,
	slot_tbl, 5,
	signal_tbl, 1,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_SettingsDialog.setMetaObject( metaObj );
    return metaObj;
}

void* SettingsDialog::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "SettingsDialog" ) )
	return this;
    if ( !qstrcmp( clname, "Ui::SettingsDialogForm" ) )
	return (Ui::SettingsDialogForm*)this;
    return QDialog::qt_cast( clname );
}

// SIGNAL configChanged
void SettingsDialog::configChanged()
{
    activate_signal( staticMetaObject()->signalOffset() + 0 );
}

bool SettingsDialog::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: on_applyButton_clicked(); break;
    case 1: on_okButton_clicked(); break;
    case 2: on_cancelButton_clicked(); break;
    case 3: on_useFogCheckbox_toggled((bool)static_QUType_bool.get(_o+1)); break;
    case 4: on_floorTranslucencySpin_valueChanged((int)static_QUType_int.get(_o+1)); break;
    default:
	return QDialog::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool SettingsDialog::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: configChanged(); break;
    default:
	return QDialog::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool SettingsDialog::qt_property( int id, int f, QVariant* v)
{
    return QDialog::qt_property( id, f, v);
}

bool SettingsDialog::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
