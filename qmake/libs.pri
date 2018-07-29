
# fltk
INCLUDEPATH += $$MAIN_DIR/libs/fltk/include

unix {
    QMAKE_CXXFLAGS += -isystem $$MAIN_DIR/libs/fltk/include
    QMAKE_CFLAGS += -isystem $$MAIN_DIR/libs/fltk/include
}

unix {
    # fltk
    LIBS += $$MAIN_DIR/libs/fltk/bin/$$PLATFORM/$$COMPILE_MODE/libfltk_images.a
    LIBS += $$MAIN_DIR/libs/fltk/bin/$$PLATFORM/$$COMPILE_MODE/libfltk.a
}

macx {
    LIBS += -lc++ -lpthread -framework Cocoa
}

win32 {
    # fltk
    LIBS += /LIBPATH:$$MAIN_DIR/libs/fltk/bin/$$PLATFORM/$$COMPILE_MODE
    LIBS += fltkimages$${COMPILE_FLAG}.lib
    LIBS += fltk$${COMPILE_FLAG}.lib
    LIBS += fltkjpeg$${COMPILE_FLAG}.lib
    LIBS += fltkpng$${COMPILE_FLAG}.lib

    # windows
    LIBS += Gdi32.lib User32.lib Ole32.lib Advapi32.lib Shell32.lib Comdlg32.lib
}
