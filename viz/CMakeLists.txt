

rock_vizkit_plugin(vfh_star-viz
    VFHTreeVisualization.cpp TraversabilityMapGeneratorVisualization.cpp
    PluginLoader.cpp
    MOC VFHTreeVisualization.cpp
    DEPS vfh_star
    HEADERS VFHTreeVisualization.hpp TraversabilityMapGeneratorVisualization.h)

