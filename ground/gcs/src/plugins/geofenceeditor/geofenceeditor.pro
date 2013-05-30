TEMPLATE = lib
TARGET = GeofenceEditor 

include(../../taulabsgcsplugin.pri) 
include(../../plugins/coreplugin/coreplugin.pri) 
include(../../plugins/uavobjects/uavobjects.pri)

HEADERS += geofenceeditorgadget.h
HEADERS += geofencetable.h
HEADERS += geofenceeditorgadgetwidget.h
HEADERS += geofenceeditorgadgetfactory.h
HEADERS += geofenceeditorplugin.h

SOURCES += geofenceeditorgadget.cpp
SOURCES += geofencetable.cpp
SOURCES += geofenceeditorgadgetwidget.cpp
SOURCES += geofenceeditorgadgetfactory.cpp
SOURCES += geofenceeditorplugin.cpp

OTHER_FILES += geofenceeditor.pluginspec

FORMS += geofenceeditor.ui

RESOURCES += geofenceeditor.qrc


