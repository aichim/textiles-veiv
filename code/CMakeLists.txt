cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project (VEIV_Textiles)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package (Eigen REQUIRED)
include_directories (${EIGEN_INCLUDE_DIR})

find_package (LibXml2 REQUIRED)
include_directories (${LIBXML2_INCLUDE_DIR})

find_package (Boost REQUIRED)
include_directories (${Boost_INCLUDE_DIR})

include_directories (include)

SET (LIB_INCS include/utils.h
              include/drawing.h
              include/svgtiny.h
              include/svgtiny_internal.h)

SET (LIB_IMPLS )

SET (LIB_SRCS src/utils.cpp
              src/drawing.cpp

              src/example.c
              src/svgtiny.c
              src/svgtiny_gradient.c
              src/svgtiny_list.c)

ADD_LIBRARY (lib_textiles_veiv ${LIB_INCS} ${LIB_IMPLS} ${LIB_SRCS})
TARGET_LINK_LIBRARIES (lib_textiles_veiv ${LIBXML2_LIBRARIES} ${Boost_LIBRARIES})

ADD_EXECUTABLE (tests src/main_tests.cpp)
TARGET_LINK_LIBRARIES (tests lib_textiles_veiv)

