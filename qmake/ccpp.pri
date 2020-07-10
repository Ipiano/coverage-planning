include ($$PWD/boost.pri)

win32: PRE_TARGETDEPS *= $$OUT_PWD/../lib/ccpp.lib
unix: PRE_TARGETDEPS *= $$OUT_PWD/../lib/libccpp.a

LIBS *= -L$$OUT_PWD/../lib -lccpp
INCLUDEPATH *= $$PWD/../src
