MOC_DIR = .moc
UI_DIR = .ui
OBJECTS_DIR = .o
DESTDIR = $$top_builddir/bin
equals(TEMPLATE, lib):DESTDIR = $$top_builddir/lib
