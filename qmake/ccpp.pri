include ($$PWD/boost.pri)

win32: PRE_TARGETDEPS = $$OUT_PWD/ccpp-lib.lib
unix: PRE_TARGETDEPS = $$OUT_PWD/libccpp-lib.a

LIBS += -L$$OUT_PWD -lccpp-lib
INCLUDEPATH += $$PWD/../src
