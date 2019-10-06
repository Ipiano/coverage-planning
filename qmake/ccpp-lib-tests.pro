TEMPLATE = app
unset(QT)
CONFIG += console c++11

include($$PWD/ccpp-lib.pri)

SOURCE_ROOT = $$PWD/..
TEST_ROOT = $$SOURCE_ROOT/ccpplib/tests

OBJECTS_DIR = $$TARGET/.o
MOC_DIR = $$TARGET/.moc
UI_DIR = $$TARGET/.ui
DESTDIR = bin

SOURCES += \
    $$TEST_ROOT/init_test.cpp

win32: PRE_TARGETDEPS = lib/ccpp-lib.lib
unix: PRE_TARGETDEPS = lib/libccpp-lib.a

LIBS += -L$$OUT_PWD/lib -lccpp-lib

# Sets up GTEST
GTEST_ROOT = $$SOURCE_ROOT/submodules/googletest

SOURCES += \
    $$GTEST_ROOT/googletest/src/gtest-all.cc \
    $$GTEST_ROOT/googlemock/src/gmock-all.cc \
    $$GTEST_ROOT/googlemock/src/gmock_main.cc

INCLUDEPATH += \
    $$GTEST_ROOT/googletest \
    $$GTEST_ROOT/googlemock \
    $$GTEST_ROOT/googletest/include \
    $$GTEST_ROOT/googlemock/include

