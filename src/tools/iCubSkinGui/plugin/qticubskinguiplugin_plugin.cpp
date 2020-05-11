#include "qticubskinguiplugin_plugin.h"
#include "qticubskinguiplugin.h"

#include <qqml.h>

void QtICubSkinGuiPluginPlugin::registerTypes(const char *uri)
{
    // @uri robotology.icub.skingui
    qmlRegisterType<QtICubSkinGuiPlugin>(uri, 1, 0, "QtICubSkinGuiPlugin");
}


