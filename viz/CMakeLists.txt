rock_vizkit_plugin(vfh_star-viz
    VFHTreeVisualization.cpp TraversabilityMapGeneratorVisualization.cpp
    PluginLoader.cpp
    MOC VFHTreeVisualization.cpp
    DEPS vfh_star
    DEPS_PKGCONFIG base-viz
    HEADERS VFHTreeVisualization.hpp TraversabilityMapGeneratorVisualization.h)

