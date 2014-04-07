cmake_minimum_required(VERSION 2.4.6)
include(CheckIncludeFile)
include(CheckLibraryExists)

if(COMMAND cmake_policy)
  # Quash warnings about mixing library search paths
  cmake_policy(SET CMP0003 NEW)
  # Quash warning about pre-processor symbol escaping (which it doesn't
  # seem to actually do)
  cmake_policy(SET CMP0005 OLD)
endif(COMMAND cmake_policy)

###################
## CONFIGURATION ##
###################

# Not every platform has (or needs) librt
check_library_exists(rt clock_gettime "" HAVE_LIBRT)
if(HAVE_LIBRT)
  set(LIBRT "rt")
endif()

set(CMAKE_BUILD_TYPE Debug)

set(VERSION 1.2.1)

add_definitions(-DPACKAGE_VERSION=\\"${VERSION}\\" -D_XOPEN_SOURCE=600)
#add_definitions(-Wc++-compat -Wall -Wextra -Wconversion -Wpointer-arith -Wfloat-equal -Wshadow -Wwrite-strings -I.)

if(APPLE)
  add_definitions(-DHAVE_STRLEN)
endif()

include_directories(include)


#############
## HEADERS ##
#############
install (FILES include/ach.h include/Ach.hpp DESTINATION include)

#############
## TARGETS ##
#############

## libach ##
add_library(ach SHARED src/ach.c src/pipe.c)
target_link_libraries(ach pthread ${LIBRT})
install( TARGETS ach DESTINATION lib )

# This needs to manually kept in sync with the libtool versions
# Libtool-generated versions are authoratative
# Does not correspond to the package version
SET_TARGET_PROPERTIES( ach PROPERTIES
                       SOVERSION 1     # Major version
                       VERSION 1.2.1 ) # Major.minor.patch



add_library(achcpp SHARED cpp/achcpp.cpp)
target_link_libraries(achcpp pthread)
install( TARGETS achcpp DESTINATION lib )
SET_TARGET_PROPERTIES( achcpp PROPERTIES
                       SOVERSION 0     # Major version
                       VERSION 0.0.0 ) # Major.minor.patch


## achtool ##
add_executable(achtool src/achtool.c src/achutil.c)
target_link_libraries(achtool ach pthread ${LIBRT})
SET_TARGET_PROPERTIES(achtool PROPERTIES OUTPUT_NAME ach)
install( TARGETS achtool DESTINATION bin )


## achcop ##
add_executable(achcop src/achcop.c src/achutil.c)
target_link_libraries(achcop ach pthread ${LIBRT} m)
install( TARGETS achcop DESTINATION bin )

## achd ##
add_executable(achd src/achd/achd.c src/achd/client.c src/achd/io.c src/achd/transport.c src/achutil.c)
target_link_libraries(achd ach pthread ${LIBRT} m)
install( TARGETS achd DESTINATION bin )

## achtest ##
add_executable(achtest src/test/achtest.c)
target_link_libraries(achtest ach pthread ${LIBRT} m)

## achcat ##
add_executable(achcat src/achcat.c src/achutil.c)
target_link_libraries(achcat ach pthread ${LIBRT})
install( TARGETS achcat DESTINATION bin )

## achbench ##
add_executable(achbench src/ach-bench.c src/achutil.c)
target_link_libraries(achbench ach pthread ${LIBRT} m)
install( TARGETS achbench DESTINATION bin )

## ach-example ##
add_executable(ach-example src/ach-example.c)
target_link_libraries(ach-example ach pthread ${LIBRT} m)
