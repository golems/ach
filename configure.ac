#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([ach], [1.0.2~GIT], [ntd@gatech.edu])
AM_INIT_AUTOMAKE([-Wall])
AC_CONFIG_SRCDIR([src/ach.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC
AC_PROG_CC_C89
AC_PROG_LIBTOOL

AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST

# Checks for header files.
dnl AC_CHECK_HEADERS([arpa/inet.h fcntl.h inttypes.h stdint.h stdlib.h string.h sys/socket.h syslog.h unistd.h time.h])

# Checks for typedefs, structures, and compiler characteristics.
dnl AC_HEADER_STDBOOL
dnl AC_TYPE_INT32_T
dnl AC_TYPE_MODE_T
dnl AC_TYPE_OFF_T
dnl AC_TYPE_PID_T
dnl AC_TYPE_SIZE_T
dnl AC_TYPE_SSIZE_T
dnl AC_TYPE_UINT32_T
dnl AC_TYPE_UINT64_T
dnl AC_TYPE_UINT8_T

dnl AC_CHECK_SIZEOF(void *)
dnl AC_CHECK_SIZEOF(uint64_t)
dnl AC_CHECK_SIZEOF(unsigned long)
dnl AC_CHECK_SIZEOF(unsigned long long)

# Check 32/64 safe format specifiers
dnl AC_CHECK_DECLS([PRIuPTR,PRIu64,PRIx64])
AC_CHECK_DECL([PRIuPTR],
        [],
        [AC_DEFINE([PRIuPTR], ["ul"], [Assume this is unsigned long])])

# Check for POSIX realtime extensions
AC_CHECK_DECL([CLOCK_REALTIME],
        AM_CONDITIONAL([HAVE_RT], true),
        AM_CONDITIONAL([HAVE_RT], false),
        [[#include <time.h>]])

# check pthread decls
dnl AC_CHECK_DECL([PTHREAD_MUTEX_ERRORCHECK_NP],
dnl        [], [],
dnl        [#include<pthread.h>])

dnl AC_CHECK_DECL( [PTHREAD_PRIO_INHERIT],
dnl               [], [], [#include<pthread.h>])

dnl AC_CHECK_DECL([_POSIX_THREAD_PROCESS_SHARED])

## Checks for library functions.

# If you don't have these functions, there's really no hope.
# Also, these checks seem to complicate cross-compiling.
# AC_FUNC_FORK
# AC_FUNC_MALLOC
# AC_FUNC_MMAP
# AC_FUNC_REALLOC
# AC_FUNC_STRTOD

dnl AC_CHECK_FUNCS([ftruncate isascii memmove memset munmap socket strcasecmp strchr strdup strerror strtol])
AC_SEARCH_LIBS([pthread_create],[pthread])
AC_SEARCH_LIBS([clock_gettime],[rt])


# Enable maximum warnings
m4_ifdef([AX_CFLAGS_WARN_ALL],
         [AX_CFLAGS_WARN_ALL])

dnl APPEND_FLAG( [FLAG] )
dnl Check if FLAG is supported, and if so, add it

dnl The AX_CHECK_COMPILER_FLAGS maco was renamed to AX_CHECK_COMPILE_FLAG
dnl Checking for both versions
m4_ifdef([AX_CHECK_COMPILE_FLAG],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CFLAGS="$1 $CFLAGS"])])])

m4_ifdef([AX_CHECK_COMPILER_FLAGS],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CFLAGS="$1 $CFLAGS"])])])

dnl If we found the flag checking macro, check some flags
m4_ifdef([APPEND_FLAG],
         [APPEND_FLAG([-Wextra])
          APPEND_FLAG([-Wconversion])
          APPEND_FLAG([-Wpointer-arith])
          APPEND_FLAG([-Wfloat-equal])
          APPEND_FLAG([-Wshadow])
          APPEND_FLAG([-Wwrite-strings])
          APPEND_FLAG([-Wc++-compat])])

# Doxygen
m4_ifdef([DX_INIT_DOXYGEN],
         [DX_HTML_FEATURE(ON)
          DX_PDF_FEATURE(OFF)
          DX_PS_FEATURE(OFF)
          DX_INIT_DOXYGEN(ach)],
         [AM_CONDITIONAL([DX_COND_doc],   false)
          AM_CONDITIONAL([DX_COND_html],  false)
          AM_CONDITIONAL([DX_COND_chm],   false)
          AM_CONDITIONAL([DX_COND_chi],   false)
          AM_CONDITIONAL([DX_COND_rtf],   false)
          AM_CONDITIONAL([DX_COND_xml],   false)
          AM_CONDITIONAL([DX_COND_ps],    false)
          AM_CONDITIONAL([DX_COND_pdf],   false)
          AM_CONDITIONAL([DX_COND_latex], false)
          AM_CONDITIONAL([DX_COND_man],   false)
         ])

AC_CHECK_PROGS([HELP2MAN], [help2man])
AM_CONDITIONAL([HAVE_HELP2MAN], [test x$HELP2MAN != x])

AC_CHECK_PROGS([MAN2HTML], [man2html])
AM_CONDITIONAL([HAVE_MAN2HTML], [test x$MAN2HTML != x])

AC_CONFIG_FILES([Makefile doc/Makefile])
AC_OUTPUT
