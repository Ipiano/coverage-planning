CONFIG *= c++11

QMAKE_CXXFLAGS *= -Wno-deprecated-copy -Wno-unknown-warning-option

CONFIG(release, release|debug) {
    QMAKE_CXXFLAGS_RELEASE -= -O1 -O2
    QMAKE_CXXFLAGS_RELEASE *= -O3
}
