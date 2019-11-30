# Windows doesn't really have package management
# so just force users to list the boost dir in the environment

isEmpty(BOOST_DIR): BOOST_DIR=$$(BOOST_DIR)

unix:isEmpty(BOOST_DIR) {
    BOOST_DIR = /usr/include/boost
}

isEmpty(BOOST_DIR): error("Please set BOOST_DIR in environment or qmake variables")
INCLUDEPATH += $$BOOST_DIR
