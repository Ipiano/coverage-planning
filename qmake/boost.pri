# Windows doesn't really have package management
# so just force users to list the boost dir in the environment
win32 {
    BOOST_DIR=$$(BOOST_DIR)

    isEmpty(BOOST_DIR): error("Please set BOOST_DIR in environment")
    INCLUDEPATH += $$BOOST_DIR
} else {
    CONFIG += link_pkgconfig
    PKGCONFIG += boost
}
