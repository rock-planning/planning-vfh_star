rock_library(vfh_star
    SOURCES
        TreeSearch.cpp
        VFH.cpp
        VFHStar.cpp
        HorizonPlanner.cpp
    DEPS_PKGCONFIG base-lib envire
    HEADERS
        
        TreeSearch.h VFH.h VFHStar.h Types.h 
	HorizonPlanner.hpp)

rock_executable(nnlookup_test NNLookupTest.cpp
    DEPS vfh_star)

