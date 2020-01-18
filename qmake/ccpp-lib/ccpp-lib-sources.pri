SOURCE_ROOT = $$top_srcdir/src
INCLUDEPATH += $$SOURCE_ROOT

CCPP_ROOT = $$SOURCE_ROOT/ads/ccpp

HEADERS += \
    $$CCPP_ROOT/ccpp.h \
    $$CCPP_ROOT/dcel.h \
    $$CCPP_ROOT/typedefs.h \
    $$CCPP_ROOT/sort-edges.h \
    $$CCPP_ROOT/angle-cost-sum.hpp \
    \
    $$CCPP_ROOT/interfaces/polygon-decomposer-if.h \
    $$CCPP_ROOT/interfaces/initial-cost-calculator-if.h \
    $$CCPP_ROOT/interfaces/turn-cost-calculator-if.h \
    $$CCPP_ROOT/interfaces/optimal-direction-calculator-if.h \
    \
    $$CCPP_ROOT/polygon-decomposer/modified-trapezoidal.h \
    $$CCPP_ROOT/initial-cost/min-across-angles.hpp \
    $$CCPP_ROOT/turn-cost/u-shaped.h \
    $$CCPP_ROOT/optimal-direction/min-across-angles.h


SOURCES += \
    $$CCPP_ROOT/dcel.cpp \
    $$CCPP_ROOT/sort-edges.cpp \
    $$CCPP_ROOT/optimal-direction/min-across-angles.cpp \
    \
    $$CCPP_ROOT/turn-cost/u-shaped.cpp \
    $$CCPP_ROOT/polygon-decomposer/modified-trapezoidal.cpp \
