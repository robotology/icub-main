#include "loadingwidget.h"
#include "ui_loadingwidget.h"
#include <QFontMetrics>

LoadingWidget::LoadingWidget(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LoadingWidget)
{
    ui->setupUi(this);

    setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    setAttribute(Qt::WA_TranslucentBackground);

    setModal(true);


    splashTimer.setInterval(50);
    splashTimer.setSingleShot(false);
    connect(&splashTimer,SIGNAL(timeout()),this,SLOT(onSplashTimer()),Qt::DirectConnection);
}

LoadingWidget::~LoadingWidget()
{
    delete ui;
}


void LoadingWidget::onSplashTimer()
{
    QString name = QString(":/images/splash/bg%1.png").arg(counter+1);
    counter++;
    counter = counter % 8;
    ui->loadingIco->setPixmap(QPixmap(name));
    ui->loadingIco->repaint();
}


int LoadingWidget::start(QString msg)
{
    if(!msg.isEmpty()){
        ui->text->setText(msg);
        QFont f = ui->text->font();
        QFontMetrics metric = QFontMetrics(f);

        int w = metric.width(msg);

        if(w > width()){
            setMinimumWidth(w);
            setMaximumWidth(w);
        }
    }
    QWidget *w  =((QWidget*)parent());
    int x = w->x() + ((w->width() - width()) / 2);
    int y = w->y() + ((w->height() - height()) / 2);
    this->setGeometry(x,y,width(),height());
    splashTimer.start();
    //return exec();
    show();
    return 0;
}

void LoadingWidget::stop()
{
    //accept();
    hide();
    splashTimer.stop();
}
