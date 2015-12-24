TEMPLATE = lib
TARGET = Qualcomm
include(../../taulabsgcsplugin.pri)
include(../../plugins/uavobjects/uavobjects.pri)
include(../../plugins/coreplugin/coreplugin.pri)

OTHER_FILES += Qualcomm.pluginspec

HEADERS += \
    qualcommplugin.h

SOURCES += \
    qualcommplugin.cpp

RESOURCES += \
    qualcomm.qrc
