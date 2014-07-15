rock_library(vfh_star
    SOURCES
        DriveMode.cpp
        HorizonPlanner.cpp
        NNLookup.cpp
        NNLookupBox.cpp
        Tree.cpp
        TreeSearch.cpp  
        TreeNode.cpp 
        VFH.cpp
        VFHStar.cpp
    DEPS_PKGCONFIG base-lib envire
    HEADERS
        DriveMode.hpp
        HorizonPlanner.hpp
        NNLookup.hpp 
        NNLookupBox.hpp
        Tree.hpp
        TreeSearch.h
        TreeNode.hpp
        Types.h 
        VFH.h 
        VFHStar.h 
	)

rock_executable(nnlookup_test 
        NNLookupTest.cpp
    DEPS 
        vfh_star
)

