TEMPLATE = app
unset(QT)
CONFIG += console c++11

include($$top_qmakedir/ccpp.pri)
include($$top_qmakedir/output-dirs.pri)
include($$top_qmakedir/gtest.pri)

PROJECT_ROOT = $$top_srcdir
TEST_ROOT=$$PROJECT_ROOT/gtests

# Not sure why, but I can't get regular regex.a to link
# so we're linking boost's version instead, which exposes
# the POSIX interface
LIBS += -lboost_regex

SOURCES += $$files($$TEST_ROOT/*test.cpp, true)

