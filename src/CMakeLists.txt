rock_library(vfh_star
    SOURCES
        ElevationGrid.cpp Grid.cpp TraversabilityGrid.cpp TraversabilityMapGenerator.cpp
        TreeSearch.cpp VFH.cpp VFHStar.cpp HeuristicLUT.cpp
    DEPS_PKGCONFIG base-lib envire
    HEADERS
        Bresenham.h DebugTypes.h ElevationGrid.h Grid.h TraversabilityGrid.h
        TraversabilityMapGenerator.h TreeSearch.h VFH.h VFHStar.h Types.h
        HeuristicLUT.h)

rock_executable(vfh_star_test VFHStarTest.cpp
    DEPS vfh_star)

