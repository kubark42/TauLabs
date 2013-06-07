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



win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/release/ -lkmlbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/debug/ -lkmlbase
else:unix: LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/ -lkmlbase
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/release/ -lkmlconvenience
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/debug/ -lkmlconvenience
else:unix: LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/ -lkmlconvenience
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/release/ -lkmlengine
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/debug/ -lkmlengine
else:unix: LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/ -lkmlengine
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/release/ -lkmlregionator
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/debug/ -lkmlregionator
else:unix: LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/ -lkmlregionator
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/release/ -lkmlxsd
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/debug/ -lkmlxsd
else:unix: LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/ -lkmlxsd
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/release/ -lkmldom
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/debug/ -lkmldom
else:unix: LIBS += -L$$PWD/../../../../../../../../../usr/local/lib/ -lkmldom

INCLUDEPATH += $$PWD/../../../../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../../../../usr/local/include
