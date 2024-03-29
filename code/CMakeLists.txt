cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project (VEIV_Textiles)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package (Eigen REQUIRED)
include_directories (${EIGEN_INCLUDE_DIR})

find_package (LibXml2 REQUIRED)
include_directories (${LIBXML2_INCLUDE_DIR})

find_package (Boost REQUIRED)
include_directories (${Boost_INCLUDE_DIR})

find_package (OpenMesh REQUIRED)
include_directories (${OPENMESH_INCLUDE_DIR})
link_directories (${OPENMESH_LIBRARY_DIR})

include_directories (include)
set (CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++ -g3 -Wall -O0")

SET (LIB_INCS include/utils.h
              include/drawing.h
              include/svgtiny.h
              include/svgtiny_internal.h
              include/SVGReader.h
              include/arc_generator.h
              include/mesh.h
              include/conewarp.h)

SET (LIB_IMPLS )

SET (LIB_SRCS src/utils.cpp
              src/drawing.cpp
              src/main_tests.cpp
              src/svgtiny.c
              src/svgtiny_gradient.c
              src/svgtiny_list.c
              src/SVGReader.cpp
              src/arc_generator.cpp
              src/mesh.cpp
              src/conewarp.cpp)

ADD_LIBRARY (lib_textiles_veiv ${LIB_INCS} ${LIB_IMPLS} ${LIB_SRCS})
TARGET_LINK_LIBRARIES (lib_textiles_veiv ${LIBXML2_LIBRARIES} ${Boost_LIBRARIES} ${OPENMESH_LIBRARY})

ADD_EXECUTABLE (tests src/main_tests.cpp)
TARGET_LINK_LIBRARIES (tests lib_textiles_veiv)

#ADD_EXECUTABLE (repeat src/main_repeat.cpp)
#TARGET_LINK_LIBRARIES (repeat lib_textiles_veiv)

#ADD_EXECUTABLE (conewarp src/main_conewarp.cpp)
#TARGET_LINK_LIBRARIES (conewarp lib_textiles_veiv)
