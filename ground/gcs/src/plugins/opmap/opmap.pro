QT += xml
TEMPLATE = lib
TARGET = OPMapGadget
include(../../taulabsgcsplugin.pri)
include(../../plugins/coreplugin/coreplugin.pri)
include(../../libs/opmapcontrol/opmapcontrol.pri)
include(../../plugins/uavobjects/uavobjects.pri)
include(../../plugins/uavobjectutil/uavobjectutil.pri)
include(../../plugins/uavtalk/uavtalk.pri)
include(../../plugins/pathplanner/pathplanner.pri)
include(../../libs/utils/utils.pri)

DEFINES += USE_PATHPLANNER \
    USE_GEOFENCE

HEADERS += opmapplugin.h \
    opmapgadgetoptionspage.h \
    opmapgadgetfactory.h \
    opmapgadgetconfiguration.h \
    opmapgadget.h \
    opmapgadgetwidget.h \
    opmap_zoom_slider_widget.h \
    opmap_statusbar_widget.h \
    modelmapproxy.h \
    geofenceverticesdatamodel.h \
    geofencefacesdatamodel.h \
    geofencemodelmapproxy.h \
    geofencedialog.h \
    geofencemodeluavoproxy.h \
    homeeditor.h
SOURCES += opmapplugin.cpp \
    opmapgadgetwidget.cpp \
    opmapgadgetoptionspage.cpp \
    opmapgadgetfactory.cpp \
    opmapgadgetconfiguration.cpp \
    opmapgadget.cpp \
    opmap_zoom_slider_widget.cpp \
    opmap_statusbar_widget.cpp \
    modelmapproxy.cpp \
    geofenceverticesdatamodel.cpp \
    geofencefacesdatamodel.cpp \
    geofencemodelmapproxy.cpp \
    geofencedialog.cpp \
    geofencemodeluavoproxy.cpp \
    homeeditor.cpp

OTHER_FILES += OPMapGadget.pluginspec

FORMS += opmapgadgetoptionspage.ui \
    opmap_widget.ui \
    opmap_zoom_slider_widget.ui \
    opmap_statusbar_widget.ui \
    opmap_overlay_widget.ui \
    geofencedialog.ui \
    homeeditor.ui

RESOURCES += opmap.qrc
