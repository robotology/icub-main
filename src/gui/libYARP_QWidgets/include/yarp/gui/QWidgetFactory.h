// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef _QWIDGETFACTORY_INC_
#define _QWIDGETFACTORY_INC_

// std
#include <vector>

// iCub
#include "QWidgetYarp.h"

// Qt
#include <qwidget.h>

namespace yarp{
	namespace gui{
		class QWidgetFactory;
		template <class T> class QWidgetFactoryOf;
		class QWidgetFactories;
	}
}

class yarp::gui::QWidgetFactory {
public:
	virtual yarp::gui::QWidgetYarp *create(yarp::os::Searchable& config, 
											QWidget* parent = 0) = 0;
    virtual yarp::os::ConstString getName() = 0;
};


template <class T>
class yarp::gui::QWidgetFactoryOf : public yarp::gui::QWidgetFactory {
private:
    yarp::os::ConstString name;
public:
    QWidgetFactoryOf(const char *name) : name(name) {
    }

    virtual yarp::gui::QWidgetYarp *create(yarp::os::Searchable& config, 
											QWidget* parent = 0) {
        return new T(config, parent, name.c_str());
    }
    
    virtual yarp::os::ConstString getName() {
        return name;
    }
};


class yarp::gui::QWidgetFactories {
public:
    virtual ~QWidgetFactories() {
        clear();
    }
    
    void add(QWidgetFactory *factory) {
        group.push_back(factory);
    }

    static QWidgetFactories& getPool() {
        return pool;
    }

    yarp::gui::QWidgetYarp *get(const char *name,
									yarp::os::Searchable &config,
									QWidget* parent = 0) {
        for (unsigned int i=0; i<group.size(); i++) {
            if (group[i]->getName() == name) {
                return group[i]->create(config, parent);
            }
        }
		return NULL;
    }

    std::vector<std::string> getNames() {
        std::vector<std::string> result;
        for (unsigned int i=0; i<group.size(); i++) {
            result.push_back(std::string(group[i]->getName().c_str()));
        }
        return result;
    }

private:
    void clear() {
        for (unsigned int i=0; i<group.size(); i++) {
            delete group[i];
            group[i] = NULL;
        }
        group.clear();
    }
    std::vector<QWidgetFactory*> group;
    static QWidgetFactories pool;
};

#endif


