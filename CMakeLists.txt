# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
include(Rock)
rock_init(vfh_star 0.1)
rock_standard_layout()
