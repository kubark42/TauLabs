TEMPLATE = lib
TARGET = LoggingGadget
DEFINES += LOGGING_LIBRARY
QT += svg
include(../../taulabsgcsplugin.pri)
include(logging_dependencies.pri)
HEADERS += loggingplugin.h \
    kmlexport.h \
    logfile.h \
    logginggadgetwidget.h \
    logginggadget.h \
    logginggadgetfactory.h \
    loggingdevice.h

SOURCES += loggingplugin.cpp \
    kmlexport.cpp \
    logfile.cpp \
    logginggadgetwidget.cpp \
    logginggadget.cpp \
    logginggadgetfactory.cpp \
    loggingdevice.cpp

OTHER_FILES += LoggingGadget.pluginspec
FORMS += logging.ui


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
