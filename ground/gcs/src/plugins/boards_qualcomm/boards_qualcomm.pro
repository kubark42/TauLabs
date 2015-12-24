TEMPLATE = lib
TARGET = Qualcomm
include(../../taulabsgcsplugin.pri)
include(../../plugins/uavobjects/uavobjects.pri)
include(../../plugins/coreplugin/coreplugin.pri)

OTHER_FILES += Qualcomm.pluginspec

HEADERS += \
    qualcommplugin.h \
    snapdragonflight.h \

SOURCES += \
    qualcommplugin.cpp \
    snapdragonflight.cpp \

RESOURCES += \
    qualcomm.qrc
