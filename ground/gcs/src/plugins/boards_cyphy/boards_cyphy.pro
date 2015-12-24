TEMPLATE = lib
TARGET = CyPhy
include(../../taulabsgcsplugin.pri)
include(../../plugins/uavobjects/uavobjects.pri)
include(../../plugins/coreplugin/coreplugin.pri)

OTHER_FILES += CyPhy.pluginspec

HEADERS += \
    cyphyplugin.h

SOURCES += \
    cyphyplugin.cpp

RESOURCES += \
    cyphy.qrc
