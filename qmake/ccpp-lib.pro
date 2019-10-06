TEMPLATE = lib
unset(QT)
CONFIG += staticlib c++11

include($$PWD/ccpp-lib.pri)

SOURCE_ROOT = $$PWD/../ccpplib

MOC_DIR = $$TARGET/.moc
UI_DIR = $$TARGET/.ui
OBJECTS_DIR = $$TARGET/.o
DESTDIR = lib

HEADERS += \
    $$SOURCE_ROOT/ccpplib/ccpp.h \
    $$SOURCE_ROOT/ccpplib/decomposition.h
