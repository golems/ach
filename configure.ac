#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([ipcbench], [0.0.0], [ntd@gatech.edu])
AM_INIT_AUTOMAKE([-Wall])
AC_CONFIG_SRCDIR([src/lcm/bench_lcm.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_LIBTOOL

AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST


AC_CHECK_HEADERS( [lcm/lcm.h], [have_lcm=yes] )
AC_CHECK_HEADERS( [ach.h], [have_ach=yes], [],
[
#include <time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/stat.h>
] )


AM_CONDITIONAL([HAVE_LCM], [test x$have_lcm != xyes])
AM_CONDITIONAL([HAVE_ACH], [test x$have_ach != xyes])

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
          dnl APPEND_FLAG([-Wconversion])
          APPEND_FLAG([-Wpointer-arith])
          APPEND_FLAG([-Wfloat-equal])
          APPEND_FLAG([-Wshadow])
          APPEND_FLAG([-Wwrite-strings])
          APPEND_FLAG([-Wc++-compat])])

AC_CONFIG_FILES([Makefile])

AC_OUTPUT



AC_MSG_NOTICE([])
AC_MSG_NOTICE([CONFIGURATION SUMMARY])
AC_MSG_NOTICE([=====================])
AC_MSG_NOTICE([LCM:      $have_lcm])
AC_MSG_NOTICE([ACH:      $have_ach])
