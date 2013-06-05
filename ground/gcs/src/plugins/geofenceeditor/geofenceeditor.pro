QT += xml
TEMPLATE = lib
TARGET = GeoFenceEditor

include(../../taulabsgcsplugin.pri) 
include(../../plugins/coreplugin/coreplugin.pri) 
include(../../plugins/uavobjects/uavobjects.pri)

DEFINES += GEOFENCEEDITOR_LIBRARY

HEADERS += geofenceeditorgadget.h
#HEADERS += waypointdialog.h
#HEADERS += waypointdelegate.h
HEADERS += geofenceeditor_global.h
HEADERS += geofenceeditorgadgetwidget.h
HEADERS += geofenceeditorgadgetfactory.h
HEADERS += geofenceeditorplugin.h
HEADERS += geofenceverticesdatamodel.h \
    geofencefacesdatamodel.h \
    geofencemodelmapproxy.h \
    geofencemodeluavoproxy.h

SOURCES += geofenceeditorgadget.cpp
#SOURCES += waypointdialog.cpp
#SOURCES += waypointdelegate.cpp
SOURCES += geofenceeditorgadgetwidget.cpp
SOURCES += geofenceeditorgadgetfactory.cpp
SOURCES += geofenceeditorplugin.cpp
SOURCES += geofenceverticesdatamodel.cpp \
    geofencefacesdatamodel.cpp \
    geofencemodelmapproxy.cpp \
    geofencemodeluavoproxy.cpp

OTHER_FILES += GeoFenceEditor.pluginspec

FORMS += geofence_dialog.ui
FORMS += vertex_dialog.ui

RESOURCES += geofenceeditor.qrc


