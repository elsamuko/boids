
FLUID_BIN = ../../libs/fltk/bin/$$PLATFORM/release/fluid

# fluid
fluid_cpp.output = ${QMAKE_FILE_PATH}$${DIR_SEPARATOR}${QMAKE_FILE_BASE}.cpp
fluid_cpp.commands = cd ${QMAKE_FILE_PATH} && $$FLUID_BIN -c -o ${QMAKE_FILE_BASE}.cpp -h ${QMAKE_FILE_BASE}.hpp ${QMAKE_FILE_BASE}.fl
fluid_cpp.input = FLUIDS
fluid_cpp.variable_out = SOURCES
QMAKE_EXTRA_COMPILERS += fluid_cpp

fluid_hpp.output = ${QMAKE_FILE_PATH}$${DIR_SEPARATOR}${QMAKE_FILE_BASE}.hpp
fluid_hpp.commands = cd ${QMAKE_FILE_PATH} && pwd && $$FLUID_BIN -c -o ${QMAKE_FILE_BASE}.cpp -h ${QMAKE_FILE_BASE}.hpp ${QMAKE_FILE_BASE}.fl
fluid_hpp.input = FLUIDS
fluid_hpp.variable_out = HEADERS
QMAKE_EXTRA_COMPILERS += fluid_hpp

INCLUDEPATH += $${SRC_DIR}/gui/fl
FLUIDS += $${SRC_DIR}/gui/mainwindow.fl
