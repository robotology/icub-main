/****************************************************************************
** AnimationView meta object code from reading C++ file 'animationview.h'
**
** Created: mer 14. lug 13:59:02 2010
**      by: The Qt MOC ($Id: moc_yacc.cpp 2051 2007-02-21 10:04:20Z chehrlic $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "animationview.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *AnimationView::className() const
{
    return "AnimationView";
}

QMetaObject *AnimationView::metaObj = 0;
static QMetaObjectCleanUp cleanUp_AnimationView( "AnimationView", &AnimationView::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString AnimationView::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "AnimationView", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString AnimationView::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "AnimationView", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* AnimationView::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QGLWidget::staticMetaObject();
    static const QUMethod slot_0 = {"resetCamera", 0, 0 };
    static const QUMethod slot_1 = {"timerTimeout", 0, 0 };
    static const QUMethod slot_2 = {"draw", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "resetCamera()", &slot_0, QMetaData::Public },
	{ "timerTimeout()", &slot_1, QMetaData::Public },
	{ "draw()", &slot_2, QMetaData::Protected }
    };
    static const QUMethod signal_0 = {"backgroundClicked", 0, 0 };
    static const QMetaData signal_tbl[] = {
	{ "backgroundClicked()", &signal_0, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"AnimationView", parentObject,
	slot_tbl, 3,
	signal_tbl, 1,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_AnimationView.setMetaObject( metaObj );
    return metaObj;
}

void* AnimationView::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "AnimationView" ) )
	return this;
    return QGLWidget::qt_cast( clname );
}

// SIGNAL backgroundClicked
void AnimationView::backgroundClicked()
{
    activate_signal( staticMetaObject()->signalOffset() + 0 );
}

bool AnimationView::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: resetCamera(); break;
    case 1: timerTimeout(); break;
    case 2: draw(); break;
    default:
	return QGLWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool AnimationView::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: backgroundClicked(); break;
    default:
	return QGLWidget::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool AnimationView::qt_property( int id, int f, QVariant* v)
{
    return QGLWidget::qt_property( id, f, v);
}

bool AnimationView::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
