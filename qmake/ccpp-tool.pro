TEMPLATE = app
QT += gui widgets
CONFIG += console c++11
TARGET = ccpp-tool

include($$PWD/ccpp-lib.pri)

SOURCE_ROOT = $$PWD/../ccpptool

SOURCES += \
    $$SOURCE_ROOT/src/main.cpp

OBJECTS_DIR = $$TARGET/.o
MOC_DIR = $$TARGET/.moc
UI_DIR = $$TARGET/.ui
DESTDIR = bin

PRE_TARGETDEPS = lib/ccpp-lib.lib
LIBS += -L$$OUT_PWD/lib -lccpp-lib
