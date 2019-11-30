GTEST_ROOT = $$PWD/../submodules/googletest

SOURCES += \
    $$GTEST_ROOT/googletest/src/gtest-all.cc \
    $$GTEST_ROOT/googlemock/src/gmock-all.cc \
    $$GTEST_ROOT/googlemock/src/gmock_main.cc

INCLUDEPATH += \
    $$GTEST_ROOT/googletest \
    $$GTEST_ROOT/googlemock \
    $$GTEST_ROOT/googletest/include \
    $$GTEST_ROOT/googlemock/include

