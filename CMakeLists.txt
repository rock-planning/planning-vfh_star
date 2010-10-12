
PROJECT(vfh_star)

cmake_minimum_required(VERSION 2.6)

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    add_definitions (-Wall)
endif (CXX_SUPPORTS_WALL)

#list all source files here
#ADD_EXECUTABLE(driver main.cpp interface.cpp)
find_package(PkgConfig)

pkg_check_modules(BASE_TYPES REQUIRED "base-types")
INCLUDE_DIRECTORIES(${BASE_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_TYPES_LIBRARY_DIRS})


ADD_LIBRARY(vfh_star SHARED
	    VFHStar.cpp
	   )

target_link_libraries(vfh_star ${BASE_LIBRARIES})

add_executable(vfh_star_test VFHStarTest.cpp)
target_link_libraries(vfh_star_test vfh_star)
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS vfh_star
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES VFHStar.h
	DESTINATION include/vfh_star)

CONFIGURE_FILE(vfh_star.pc.in vfh_star.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/vfh_star.pc DESTINATION lib/pkgconfig)
