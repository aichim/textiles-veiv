cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project (VEIV_Textiles)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package (Eigen REQUIRED)
include_directories (${EIGEN_INCLUDE_DIR})


include_directories (include)



SET (LIB_INCS include/utils.h)

SET (LIB_IMPLS )

SET (LIB_SRCS src/utils.cpp)

ADD_LIBRARY (lib_textiles_veiv ${LIB_INCS} ${LIB_IMPLS} ${LIB_SRCS})
TARGET_LINK_LIBRARIES (lib_textiles_veiv )

ADD_EXECUTABLE (tests src/main_tests.cpp)
TARGET_LINK_LIBRARIES (tests lib_textiles_veiv)

