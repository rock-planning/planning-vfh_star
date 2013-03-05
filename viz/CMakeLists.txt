

rock_vizkit_plugin(vfh_star-viz
    VFHTreeVisualization.cpp TraversabilityMapGeneratorVisualization.cpp
    PluginLoader.cpp
    DEPS vfh_star
    DEPS_PKGCONFIG vizkit
    HEADERS VFHTreeVisualization.hpp TraversabilityMapGeneratorVisualization.h)

