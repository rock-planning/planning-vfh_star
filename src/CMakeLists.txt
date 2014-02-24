rock_library(vfh_star
    SOURCES
        ElevationGrid.cpp Grid.cpp TraversabilityGrid.cpp TraversabilityMapGenerator.cpp
        TreeSearch.cpp VFH.cpp VFHStar.cpp RadialLookUpTable.cpp HorizonPlanner.cpp
    DEPS_PKGCONFIG base-lib envire
    HEADERS
        Bresenham.h DebugTypes.h ElevationGrid.h Grid.h TraversabilityGrid.h
        TraversabilityMapGenerator.h TreeSearch.h VFH.h VFHStar.h Types.h 
	RadialLookUpTable.hpp HorizonPlanner.hpp)

rock_executable(nnlookup_test NNLookupTest.cpp
    DEPS vfh_star)

