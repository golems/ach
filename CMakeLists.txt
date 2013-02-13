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

# Not every platform has (or needs) librt
check_library_exists(rt clock_gettime "" HAVE_LIBRT)
if(HAVE_LIBRT)
  set(LIBRT "rt")
endif()

set(CMAKE_BUILD_TYPE Debug)

set(VERSION 1.1.0)

add_definitions(-DPACKAGE_VERSION=\\"${VERSION}\\" -D_XOPEN_SOURCE=600)
#add_definitions(-Wc++-compat -Wall -Wextra -Wconversion -Wpointer-arith -Wfloat-equal -Wshadow -Wwrite-strings -I.)

if(APPLE)
  add_definitions(-DHAVE_STRLEN)
endif()

include_directories(include)

add_library(ach SHARED src/ach.c src/pipe.c)
target_link_libraries(ach pthread ${LIBRT})

# This needs to manually kept in sync with the libtool versions
# Libtool-generated versions are authoratative
# Does not correspond to the package version
SET_TARGET_PROPERTIES( ach PROPERTIES
                       SOVERSION 1     # Major version
                       VERSION 1.1.0 ) # Major.minor.patch

add_executable(achtool src/achtool.c src/achutil.c)
target_link_libraries(achtool ach pthread ${LIBRT})
SET_TARGET_PROPERTIES(achtool PROPERTIES OUTPUT_NAME ach)

add_executable(achtest src/achtest.c)
target_link_libraries(achtest ach pthread ${LIBRT} m)

add_executable(achpipe.bin src/achpipe-bin.c src/achutil.c)
target_link_libraries(achpipe.bin ach pthread ${LIBRT})

if(HAVE_LIBRT)
   add_executable(achcat src/achcat.c src/achutil.c)
   target_link_libraries(achcat ach pthread ${LIBRT})

   add_executable(achbench src/ach-bench.c src/achutil.c)
   target_link_libraries(achbench ach pthread ${LIBRT} m)

   add_executable(ach-example src/ach-example.c)
   target_link_libraries(ach-example ach pthread ${LIBRT} m)
endif()
