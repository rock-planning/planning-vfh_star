find_package(Qt4 4.5.0 REQUIRED QtOpenGL)
add_definitions(${QT_DEFINITIONS})
include_directories(${QT_INCLUDE_DIR})
include_directories(${QT_QTCORE_INCLUDE_DIR})
include_directories(${QT_QTGUI_INCLUDE_DIR})
set(QT_USE_OPENGL TRUE)
include(${QT_USE_FILE})


rock_vizkit_plugin(vfh_star-viz
    VFHTreeVisualization.cpp
    TraversabilityMapGeneratorVisualization.cpp
    MOC VFHTreeVisualization.hpp
    DEPS vfh_star
    HEADERS VFHTreeVisualization.hpp TraversabilityMapGeneratorVisualization.h)

