#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([ach], [1.2.1], [ntd@gatech.edu])
AM_INIT_AUTOMAKE([-Wall subdir-objects])
AC_CONFIG_SRCDIR([src/ach.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_CXX

AM_PROG_AR
AM_PROG_CC_C_O
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

# check pthread decls
dnl AC_CHECK_DECL([PTHREAD_MUTEX_ERRORCHECK_NP],
dnl        [], [],
dnl        [#include<pthread.h>])

dnl AC_CHECK_DECL( [PTHREAD_PRIO_INHERIT],
dnl               [], [], [#include<pthread.h>])



dnl AC_CHECK_FUNCS([ftruncate isascii memmove memset munmap socket strcasecmp strchr strdup strerror strtol])
AC_SEARCH_LIBS([pthread_mutex_lock],[pthread])
AC_SEARCH_LIBS([pthread_mutexattr_setpshared],[pthread])
AC_SEARCH_LIBS([clock_gettime],[rt])
AC_SEARCH_LIBS([shm_open],[rt])
AC_SEARCH_LIBS([res_search],[resolv])
AC_SEARCH_LIBS([__res_search],[resolv])

# check process-shared mutex
AC_CHECK_DECL([_POSIX_THREAD_PROCESS_SHARED],
              [],
              [AC_MSG_ERROR([process-shared condition variables are required])])

# CHECK_SOURCE( checking-msg, prologue, body, symbol )
# Runs source if native compiling.  If cross-compiling, just checks that it compiles.
AC_DEFUN([CHECK_SOURCE],
         [AC_MSG_CHECKING([$1])
          AC_RUN_IFELSE([AC_LANG_PROGRAM([$2], [$3])],
                        [echo [yes]; $4],
                        [echo [no]],
                        [AC_TRY_COMPILE([$2],[$3],
                                        [echo [yes]; $4],
                                        [echo [no]])])])

# check priority inheritance mutex
CHECK_SOURCE([for priority-inherit mutex],
             [#include <pthread.h>],
             [
pthread_mutexattr_t mutex_attr;
pthread_mutex_t mutex;
if( pthread_mutexattr_init(&mutex_attr) )
    return -1;
if( pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED) )
    return -1;
if( pthread_mutexattr_setprotocol(&mutex_attr, PTHREAD_PRIO_INHERIT) )
    return -1;
if( pthread_mutex_init(&mutex, &mutex_attr) )
    return -1;
            ],
            [AC_DEFINE([HAVE_MUTEX_PRIORITY_INHERIT],[1],[Priority Inheritance Mutex])])


# check robust mutex
CHECK_SOURCE([for robust mutex],
             [#include <pthread.h>],
             [
pthread_mutexattr_t mutex_attr;
pthread_mutex_t mutex;
if( pthread_mutexattr_init(&mutex_attr) )
    return -1;
if( pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED) )
    return -1;
#ifdef HAVE_MUTEX_PRIORITY_INHERIT
if( pthread_mutexattr_setprotocol(&mutex_attr, PTHREAD_PRIO_INHERIT) )
    return -1;
#endif
if( pthread_mutexattr_setrobust(&mutex_attr, PTHREAD_MUTEX_ROBUST) )
    return -1;
if( pthread_mutex_init(&mutex, &mutex_attr) )
    return -1;
             ],
             [AC_DEFINE([HAVE_MUTEX_ROBUST],[1],[Robust Mutexes])])


# check error-checking mutex
CHECK_SOURCE([for error-checking mutex],
             [#include <pthread.h>],
             [
pthread_mutexattr_t mutex_attr;
pthread_mutex_t mutex;
if( pthread_mutexattr_init(&mutex_attr) )
    return -1;
if( pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED) )
    return -1;
#ifdef HAVE_MUTEX_PRIORITY_INHERIT
if( pthread_mutexattr_setprotocol(&mutex_attr, PTHREAD_PRIO_INHERIT) )
    return -1;
#endif
#ifdef HAVE_MUTEX_ROBUST
if( pthread_mutexattr_setrobust(&mutex_attr, PTHREAD_MUTEX_ROBUST) )
    return -1;
#endif
if( pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK) )
    return -1;
if( pthread_mutex_init(&mutex, &mutex_attr) )
    return -1;
             ],
             [AC_DEFINE([HAVE_MUTEX_ERROR_CHECK],[1],[Error Checking Mutexes])])

## Checks for library functions.

# If you don't have these functions, there's really no hope.
# Also, these checks seem to complicate cross-compiling.
# AC_FUNC_FORK
# AC_FUNC_MALLOC
# AC_FUNC_MMAP
# AC_FUNC_REALLOC
# AC_FUNC_STRTOD

# Check mutex options

# Enable maximum warnings
m4_ifdef([AX_CFLAGS_WARN_ALL],
         [AX_CFLAGS_WARN_ALL])

dnl APPEND_FLAG( [FLAG] )
dnl Check if FLAG is supported, and if so, add it


AC_DEFUN([APPEND_CFLAG],[])
AC_DEFUN([APPEND_CXXFLAG],[])

dnl The AX_CHECK_COMPILER_FLAGS maco was renamed to AX_CHECK_COMPILE_FLAG
dnl Checking for both versions
m4_ifdef([AX_CHECK_COMPILE_FLAG],
         [AC_DEFUN([APPEND_CFLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CFLAGS="$1 $CFLAGS"])])
          AC_DEFUN([APPEND_CXXFLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CXXFLAGS="$1 $CXXFLAGS"])])])

m4_ifdef([AX_CHECK_COMPILER_FLAGS],
         [AC_DEFUN([APPEND_CFLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CFLAGS="$1 $CFLAGS"])])
          AC_DEFUN([APPEND_CXXFLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CXXFLAGS="$1 $CXXFLAGS"])])])

dnl If we found the flag checking macro, check some flags

m4_ifdef([APPEND_CFLAG],
         [AC_LANG(C)
          APPEND_CFLAG([-Wextra])
          APPEND_CFLAG([-Wconversion])
          APPEND_CFLAG([-Wpointer-arith])
          APPEND_CFLAG([-Wfloat-equal])
          APPEND_CFLAG([-Wshadow])
          APPEND_CFLAG([-Wwrite-strings])
          APPEND_CFLAG([-Wc++-compat])])

m4_ifdef([APPEND_CXXFLAG],
         [AC_LANG(C++)
          APPEND_CXXFLAG([-Wextra])
          APPEND_CXXFLAG([-Wconversion])
          APPEND_CXXFLAG([-Wpointer-arith])
          APPEND_CXXFLAG([-Wfloat-equal])
          APPEND_CXXFLAG([-Wshadow])
          APPEND_CXXFLAG([-Wwrite-strings])])

# Debugging
AC_ARG_ENABLE([debug],
        AS_HELP_STRING([--enable-debug], [Build for debugging]))

AS_IF([test "x$enable_debug" = "xyes"],
      [
       AC_LANG([C])
       APPEND_CFLAG([-fstack-check])
       AC_LANG([C++])
       APPEND_CXXFLAG([-fstack-check])
      ],
      [
       enable_debug=no
       AC_LANG([C])
       APPEND_CFLAG([-DNDEBUG])
       AC_LANG([C++])
       APPEND_CXXFLAG([-DNDEBUG])
      ]
)

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

# Check for programs to make documentation
AC_CHECK_PROGS([HELP2MAN], [help2man])
AC_CHECK_PROGS([MAN2HTML], [man2html])
AC_CHECK_PROGS([DB2HTML], [db2html])
AM_CONDITIONAL([HAVE_HELP2MAN], [test x$HELP2MAN != x])
AM_CONDITIONAL([HAVE_MAN2HTML], [test x$MAN2HTML != x])
AM_CONDITIONAL([HAVE_DB2HTML],  [test x$DB2HTML != x])

# Check if we already have docs (from tarball)
dnl AC_CHECK_FILE is worthless, define our own macro
m4_define([ACH_CHECK_FILE],
          [
            AC_MSG_CHECKING([for $1])
            AS_IF([test -f "$1"],
                  [AC_MSG_RESULT([yes])
                   $2],
                  [AC_MSG_RESULT([no])
                   $3])
          ])

ACH_CHECK_FILE( [$srcdir/doc/man/ach.1],              [FOUND_MANPAGES=yes])
ACH_CHECK_FILE( [$srcdir/doc/man/ach.html],           [FOUND_MANHTML=yes])
ACH_CHECK_FILE( [$srcdir/doc/manual/ach-manual.html], [FOUND_MANUAL=yes])

# If we can make the docs or already have them, then we can install them
AM_CONDITIONAL([HAVE_MANPAGES], [test x$HELP2MAN != x -o x$FOUND_MANPAGES = xyes])
AM_CONDITIONAL([HAVE_MANHTML],  [test '(' x$HELP2MAN != x -a x$MAN2HTML != x ')' -o x$FOUND_MANHTML = xyes])
AM_CONDITIONAL([HAVE_MANUAL],   [test x$DB2HTML != x -o x$FOUND_MANUAL = xyes])

# Python
AC_ARG_WITH([python],
        AS_HELP_STRING([--with-python], [Enable python support]))

AS_IF([test "x$with_python" = "xyes"],[
    AM_PATH_PYTHON( [2.6],
                    [AX_PYTHON_DEVEL],
                    [PYTHON_VERSION=none] )
    ],
    [PYTHON_VERSION=none])

AM_CONDITIONAL([HAVE_PYTHON],   [test x: != x$PYTHON -a x != x$PYTHON])

# Java
AC_ARG_ENABLE([java],
              [AS_HELP_STRING([--disable-java],
                              [Do not build Java support])],
              [enable_java_force="$enable_java"],
              [enable_java="yes"])


AS_IF( [test "x$enable_java" = xyes], [

AS_IF( [test "x$JAVAPREFIX" = x],
       [
        AS_IF( [test "x$JAVA" = x],  [AC_CHECK_PROGS(JAVA, java kaffe)],  )
        AS_IF( [test "x$JAVAC" = x], [AC_CHECK_PROGS(JAVAC, javac "gcj -C" jikes guavac)] )
        AS_IF( [test "x$JAVAH" = x], [AC_CHECK_PROGS(JAVAH, javah)] )
        AS_IF( [test "x$JAR" = x],   [AC_CHECK_PROGS(JAR, jar)] )
        AS_IF( [test "x$JAVADOC" = x],  [AC_CHECK_PROGS([JAVADOC], [javadoc])])
       ],
       [
        AS_IF( [test "x$JAVA" = x],  [AC_CHECK_PROGS(JAVA, java kaffe)],
               [$JAVAPREFIX] )
        AS_IF( [test "x$JAVAC" = x], [AC_CHECK_PROGS(JAVAC, javac "gcj -C" jikes guavac)],
               [$JAVAPREFIX] )
        AS_IF( [test "x$JAVAH" = x], [AC_CHECK_PROGS(JAVAH, javah)],
               [$JAVAPREFIX] )
        AS_IF( [test "x$JAR" = x],   [AC_CHECK_PROGS(JAR, jar)],
               [$JAVAPREFIX] )
        AS_IF( [test "x$JAVADOC" = x],  [AC_CHECK_PROGS(JAVA, javadoc )],
               [$JAVAPREFIX] )
       ])

AS_IF([test "x$JAVAC" != "x"],
      [
       AX_JNI_INCLUDE_DIR
       for JNI_INCLUDE_DIR in $JNI_INCLUDE_DIRS
       do
           JNICPPFLAGS="$JNICPPFLAGS -I$JNI_INCLUDE_DIR"
       done
       TMPCPPFLAGS="$CPPFLAGS"
       CPPFLAGS="$CPPFLAGS $JNICPPFLAGS"
       AC_LANG(C)
       AC_CHECK_HEADER([jni.h], [HAVE_JNI=yes], [HAVE_JNI=no])
       CPPFLAGS="$TMPCPPFLAGS"
      ])

AS_IF([test "x$JAVAC" = "x" -a "x$enable_java_force" = xyes],
      [AC_MSG_ERROR([Could not find javac])])

AS_IF([test "x$HAVE_JNI" != "xyes" -a "x$enable_java_force" = xyes],
      [AC_MSG_ERROR([Could not find jni.h])])

AS_IF([test "x$JAVAC" = "x" -o "x$HAVE_JNI" != xyes],
      [enable_java=no])

])

AC_SUBST([JNICPPFLAGS])

AM_CONDITIONAL([BUILD_JAVA],   [test "x$enable_java" = xyes])
AM_CONDITIONAL([HAVE_JAVADOC],   [test "x$JAVADOC" != x])
AM_CONDITIONAL([INSTALL_JAVADOC],   [test "x$JAVADOC" != x -o -e "$srcdir/doc/javadoc"])

AC_CONFIG_FILES([Makefile doc/Makefile python/setup.py ach.pc])



AC_OUTPUT

AC_MSG_NOTICE([])
AC_MSG_NOTICE([CONFIGURATION SUMMARY])
AC_MSG_NOTICE([=====================])
AC_MSG_NOTICE([DEBUG:           $enable_debug])
AC_MSG_NOTICE([PREFIX:          $prefix])
AC_MSG_NOTICE([PYTHON VERSION:  $PYTHON_VERSION])
AC_MSG_NOTICE([BUILD JAVA LIB:  $enable_java])
