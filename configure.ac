#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([ach], [0.20111021], [ntd@gatech.edu])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([src/ach.c])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_LIBTOOL

AC_PROG_CC_C89
AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST

# Checks for libraries.

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h fcntl.h inttypes.h stdint.h stdlib.h string.h sys/socket.h syslog.h unistd.h time.h])


# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_TYPE_INT32_T
AC_TYPE_MODE_T
AC_TYPE_OFF_T
AC_TYPE_PID_T
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

AC_CHECK_SIZEOF(void *)
AC_CHECK_SIZEOF(uint64_t)
AC_CHECK_SIZEOF(unsigned long)
AC_CHECK_SIZEOF(unsigned long long)

AC_CHECK_DECLS([PRIuPTR,PRIu64,PRIx64])

AC_CHECK_DECL([PRIuPTR],
        [],
        [AC_DEFINE([PRIuPTR], ["ul"], [Assume this is unsigned long])])

# Check for CLOCK_MONOTONIC / CLOCK_HIGHRES
AC_CHECK_DECLS([CLOCK_REALTIME,CLOCK_MONOTONIC,CLOCK_HIGHRES],[],[],[[#include <time.h>]])
AC_CHECK_DECLS([CLOCK_MONOTONIC],
        [],
        [AC_CHECK_DECLS([CLOCK_HIGHRES],
                [AC_DEFINE([CLOCK_MONOTONIC],
                        [CLOCK_HIGHRES],
                        [Old Solaris lacks CLOCK_MONOTONIC,
                         CLOCK_HIGHRES looks the same])],
                [AC_CHECK_DECLS([CLOCK_REALTIME],
                        [AC_DEFINE([CLOCK_MONOTONIC],
                                [CLOCK_REALTIME],
                                [use CLOCK_REALTIME if we can])],
                        [],
                        [[#include <time.h>]])]
                [[#include <time.h>]])],
        [[#include <time.h>]])

# Checks for library functions.
AC_FUNC_FORK
AC_FUNC_MALLOC
AC_FUNC_MMAP
AC_FUNC_REALLOC
AC_FUNC_STRTOD
AC_CHECK_FUNCS([ftruncate isascii memmove memset munmap socket strcasecmp strchr strdup strerror strtol])
AC_SEARCH_LIBS([pthread_create],[pthread])
AC_SEARCH_LIBS([clock_gettime],[rt])
AC_SEARCH_LIBS([sin],[m])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
