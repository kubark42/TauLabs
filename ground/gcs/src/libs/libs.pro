TEMPLATE  = subdirs
CONFIG   += ordered
QT += widgets
SUBDIRS   = \
    qscispinbox\
    aggregation \
    extensionsystem \
    utils \
    tlmapcontrol \
    qwt \
    libcrashreporter-qt
win32 {
SUBDIRS   += \
    zlib
}
SUBDIRS   += \
    quazip
SDL {
SUBDIRS += sdlgamepad
}

!LIGHTWEIGHT_GCS {
    SUBDIRS += glc_lib
}

lib_qwt.subdir = qwt
SUBDIRS += lib_qwt

lib_qwtpolar.subdir = qwtpolar
lib_qwtpolar.depends = lib_qwt
SUBDIRS += lib_qwtpolar
