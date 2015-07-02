#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([ipcbench], [0.0.0], [ntd@gatech.edu])
AM_INIT_AUTOMAKE([-Wall])
AC_CONFIG_SRCDIR([src/ipcbench.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_LIBTOOL
AC_PROG_CXX

AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST

# Check POSIX
AC_SEARCH_LIBS([mq_send],[rt])

AC_ARG_WITH([lcm], AS_HELP_STRING([--without-lcm], [Disable LCM support]))
AC_ARG_WITH([ach], AS_HELP_STRING([--without-ach], [Disable Ach support]))
AC_ARG_WITH([tao], AS_HELP_STRING([--without-tao], [Disable TAO support]))

AS_IF( [test "x$with_lcm" != xno],
       [AC_CHECK_HEADERS( [lcm/lcm.h], [have_lcm=yes] )] )

AS_IF( [test "x$with_ach" != xno], [
AC_CHECK_HEADERS( [ach.h], [have_ach=yes], [],
[
#include <time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/stat.h>
] )
])


# Check CORBA

# Apply ACE_ROOT Variable
AS_IF( [test "x$with_tao" != xno], [
AC_LANG([C++])

AS_IF([test "x$ACE_ROOT" != "x"],
      [
       AC_MSG_NOTICE([Detected ACE_ROOT=$ACE_ROOT])
       CPPFLAGS="$CPPFLAGS -I$ACE_ROOT -I$ACE_ROOT/TAO -I$ACE_ROOT/TAO/orbsvcs"
       LDFLAGS="-L$ACE_ROOT/lib"
      ])

AC_SUBST([ACE_ROOT])
AC_CHECK_HEADERS( [tao/ORB.h],[have_tao=yes] )
])

AC_LANG([C])

AM_CONDITIONAL([HAVE_ACH], [test x$have_ach = xyes])
AM_CONDITIONAL([HAVE_LCM], [test x$have_lcm = xyes])
AM_CONDITIONAL([HAVE_TAO], [test x$have_tao = xyes])



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
AC_MSG_NOTICE([TAO:      $have_tao])
