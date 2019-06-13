#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([ach], [2.0.0], [ndantam@mines.edu])
AM_INIT_AUTOMAKE([-Wall subdir-objects])
AC_CONFIG_SRCDIR([include/ach.h])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

AM_PROG_AR
AM_PROG_CC_C_O
AC_PROG_CXX
AC_PROG_LIBTOOL

AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST

##################
## MISC SUPPORT ##
##################

# Check 32/64 safe format specifiers
dnl AC_CHECK_DECLS([PRIuPTR,PRIu64,PRIx64])
AC_CHECK_DECL([PRIuPTR],
        [],
        [AC_DEFINE([PRIuPTR], ["ul"], [Assume this is unsigned long])])
AC_SEARCH_LIBS([clock_gettime],[rt])
AC_SEARCH_LIBS([shm_open],[rt])

#################
## DNS SUPPORT ##
#################
AC_SEARCH_LIBS([res_search],[resolv])
AC_SEARCH_LIBS([__res_search],[resolv])

######################
## PTHREADS SUPPORT ##
######################
AC_SEARCH_LIBS([pthread_mutex_lock],[pthread])
AC_SEARCH_LIBS([pthread_mutexattr_setpshared],[pthread])

# check process-shared mutex
AC_CHECK_DECL([_POSIX_THREAD_PROCESS_SHARED],
              [],
              [AC_MSG_ERROR([process-shared condition variables are required])])


# CHECK_SOURCE( checking-msg, prologue, body, symbol )
# Runs source if native compiling.  If cross-compiling, just checks that it compiles.
AC_DEFUN([CHECK_SOURCE],
         [AC_MSG_CHECKING([$1])
          AC_RUN_IFELSE([AC_LANG_PROGRAM([$2], [$3])],
                        [AC_MSG_RESULT([yes]); $4],
                        [AC_MSG_RESULT([no])],
                        [AC_TRY_COMPILE([$2],[$3],
                                        [AC_MSG_RESULT([yes]); $4],
                                        [AC_MSG_RESULT([no])])])])

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


#############
## TESTING ##
#############
AC_ARG_ENABLE([tests],
        AS_HELP_STRING([--disable-tests], [Disable test execution]))
AM_CONDITIONAL([ENABLE_TESTS], [test "x$enable_tests" != xno])

####################
## COMPILER FLAGS ##
####################

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

###################
## Documentation ##
###################

AC_ARG_ENABLE([documentation],
              [AS_HELP_STRING([--disable-documentation], [Do not build documentation])],
              [],
              [enable_documentation=yes])

AC_ARG_ENABLE([javadoc],
              [AS_HELP_STRING([--disable-javadoc], [Do not build Javadoc])],
              [],
              [with_javadoc="$enable_documentation"])

AC_ARG_ENABLE([man],
              [AS_HELP_STRING([--disable-man], [Do not build man pages])],
              [],
              [enable_man="$enable_documentation"])

AC_ARG_ENABLE([manhtml],
              [AS_HELP_STRING([--disable-manhtml], [Do not build HTML man pages])],
              [],
              [enable_manhtml="$enable_documentation"])

AC_ARG_ENABLE([docbook],
              [AS_HELP_STRING([--disable-docbook], [Do not build Docbook manual])],
              [],
              [enable_docbook="$enable_documentation"])


m4_ifdef([DX_INIT_DOXYGEN],
         [DX_HTML_FEATURE(ON)
          DX_PDF_FEATURE(OFF)
          DX_MAN_FEATURE(ON)
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
AS_IF([test "x$enable_man" = xyes -a "x$build" = "x$host"],
      [AC_CHECK_PROGS([HELP2MAN], [help2man])])

AS_IF([test "x$enable_manhtml" = xyes],
      [AC_CHECK_PROGS([MAN2HTML], [man2html])])

AS_IF([test "x$enable_docbook" = xyes],
      [AC_CHECK_PROGS([DB2HTML], [db2html])])

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

############
## Python ##
############
AC_ARG_WITH([python],
            [AS_HELP_STRING([--with-python],
                            [Build python module [default=no].  Module
                            can also be built and installed with
                            pip.])])

AS_IF([test "x$with_python" = "xyes"],[
    AM_PATH_PYTHON( [2.6],
                    [AX_PYTHON_DEVEL],
                    [PYTHON_VERSION=none] )
    ],
    [PYTHON_VERSION=none])

AM_CONDITIONAL([HAVE_PYTHON],   [test x: != x$PYTHON -a x != x$PYTHON])

##########
## Java ##
##########
AC_ARG_WITH([java],
            [AS_HELP_STRING([--with-java],
                            [Build Java support [default=no]])],
            [with_java_force="$with_java"],
            [with_java="no"])


AS_IF( [test "x$with_java" = xyes], [

AS_IF( [test "x$JAVAPREFIX" = x],
       [
        AS_IF( [test "x$JAVA" = x],  [AC_CHECK_PROGS(JAVA, java kaffe)],  )
        AS_IF( [test "x$JAVAC" = x], [AC_CHECK_PROGS(JAVAC, javac "gcj -C" jikes guavac)] )
        AS_IF( [test "x$JAVAH" = x], [AC_CHECK_PROGS(JAVAH, javah)] )
        AS_IF( [test "x$JAR" = x],   [AC_CHECK_PROGS(JAR, jar)] )
        AS_IF( [test "x$JAVADOC" = x -a "x$with_javadoc" = xyes],  [AC_CHECK_PROGS([JAVADOC], [javadoc])])
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
        AS_IF( [test "x$JAVADOC" = x -a "x$with_javadoc" = xyes],  [AC_CHECK_PROGS(JAVA, javadoc )],
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

AS_IF([test "x$JAVAC" = "x" -a "x$with_java_force" = xyes],
      [AC_MSG_ERROR([Could not find javac])])

AS_IF([test "x$HAVE_JNI" != "xyes" -a "x$with_java_force" = xyes],
      [AC_MSG_ERROR([Could not find jni.h])])

AS_IF([test "x$JAVAC" = "x" -o "x$HAVE_JNI" != xyes],
      [with_java=no])

])

AC_SUBST([JNICPPFLAGS])

AM_CONDITIONAL([BUILD_JAVA],   [test "x$with_java" = xyes])
AM_CONDITIONAL([HAVE_JAVADOC],   [test "x$JAVADOC" != x])
AM_CONDITIONAL([INSTALL_JAVADOC],   [test "x$JAVADOC" != x -o -e "$srcdir/doc/javadoc"])



###########
## Linux ##
###########
AC_MSG_CHECKING([whether host is Linux])
AS_CASE([$host],
        [*linux*],
        [
          AC_MSG_RESULT([yes])
          host_linux=yes
        ],
        [
          AC_MSG_RESULT([no])
          host_linux=no
        ])


AC_ARG_VAR([KERNELRELEASE],
           [Linux kernel release version. [default=`uname -r`]])
AC_ARG_VAR([KDIR],
           [Linux kernel headers directory.  [default searches common
            locations for headers of $KERNELRELEASE]])

AC_ARG_ENABLE([kbuild],
              [AS_HELP_STRING([--enable-kbuild],
                              [Build Linux kernel module with Kbuild])],
              [enable_kbuild_force="$enable_kbuild"],
              [enable_kbuild=no])

AC_CHECK_PROGS([DKMS], [dkms], [false], [$PATH:/usr/sbin:/sbin])
AS_IF([test "x$DKMS" != xfalse], [have_dkms=yes], [have_dkms=no])

AC_ARG_ENABLE([dkms],
              [AS_HELP_STRING([--disable-dkms],
                              [Do not install DKMS sources])],
              [enable_dkms_force="$enable_dkms"],
              [AS_IF([test "x$DKMS" != x -a "x$host_linux" = xyes -a "x$enable_kbuild" != xyes],
                     [enable_dkms=yes], [enable_dkms=no])])

AC_ARG_ENABLE([dkms-build],
              [AS_HELP_STRING([--enable-dkms-build],
                              [Build kernel module with DKMS (implies --enable-dkms)])],
              [enable_dkms_force="$enable_dkms_build"
               enable_dkms_build_force="$enable_dkms_build"
               AS_IF([test "x$enable_dkms_build" = xyes],
                     [enable_dkms=yes])
              ],
              [enable_dkms_build="$enable_dkms"])


AS_IF([test "x$enable_kbuild" = xyes],
[

AS_IF([test "x$KERNELRELEASE" = "x"],
      [AC_MSG_CHECKING([for running Linux kernel release])
       KERNELRELEASE=`uname -r`
       AC_MSG_RESULT([$KERNELRELEASE])
       ])

AC_DEFUN([CHECK_KDIR],
        [
          AS_IF([test "x$KDIR" = "x"],
                [
                  AC_MSG_CHECKING([for kernel headers in $TMPKDIR])
                  AS_IF([test -f "$TMPKDIR/Makefile"],
                        [AC_MSG_RESULT([yes])
                         KDIR=$TMPKDIR],
                        [AC_MSG_RESULT([no])])
                ])
        ])

AS_IF([test "x$KDIR" = "x"],
      [
        TMPKDIR=/usr/src/linux-$KERNELRELEASE
        CHECK_KDIR
        TMPKDIR=/usr/src/linux-headers-$KERNELRELEASE
        CHECK_KDIR
        TMPKDIR=/lib/modules/$KERNELRELEASE/build
        CHECK_KDIR
      ])

AS_IF([test "x$KDIR" = "x" ],
      [
       enable_kbuild=no
       AS_IF([test "x$enable_kbuild_force" = xyes],
             [AC_MSG_ERROR([Linux kernel headers not found])])])
])

AC_SUBST([KDIR])
AC_SUBST([KERNELRELEASE])

AM_CONDITIONAL([BUILD_KBUILD],
               [test "x$enable_kbuild" = xyes])

AM_CONDITIONAL([INSTALL_DKMS],
               [test "x$enable_dkms" = xyes])

AM_CONDITIONAL([BUILD_DKMS],
               [test "x$enable_dkms_build" = xyes])



#########
## ZSH ##
#########

AC_ARG_WITH([zsh],
            [AS_HELP_STRING([--without-zsh],
                            [Setup zsh support [default=yes].  You may need to set function directory with the zshfdir variable.])],
            [with_zsh_force=$with_zsh],
            [with_zsh=yes])

AC_ARG_VAR([zshfdir],
           [Directory to install zsh functions. [default=$datarootdir/zsh/site-functions]])

AS_IF([test "x$with_zsh" = "xyes"],
      [
      AS_IF([test "x$zshfdir" = "x"],
            [zshfdir="$datarootdir/zsh/site-functions"])
      ],
      [zshfdir=""])

AC_SUBST([zshfdir])

AM_CONDITIONAL([INSTALL_ZSH],
               [test "x$zshfdir" != x])


###############
## BENCHMARK ##
###############
AC_ARG_WITH([lcm], AS_HELP_STRING([--without-lcm], [Disable LCM support]))
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

############
## OUTPUT ##
############

AC_CONFIG_FILES([Makefile doc/Makefile src/klinux/Makefile src/klinux/Kbuild src/klinux/dkms/dkms.conf src/klinux/dkms-pre/Makefile src/klinux/dkms-post/Makefile python/setup.py ach.pc])


AC_OUTPUT

AC_MSG_NOTICE([])
AC_MSG_NOTICE([CONFIGURATION SUMMARY])
AC_MSG_NOTICE([=====================])
AC_MSG_NOTICE([BUILD:               $build])
AC_MSG_NOTICE([HOST:                $host])
AC_MSG_NOTICE([DEBUG:               $enable_debug])
AC_MSG_NOTICE([PREFIX:              $prefix])
AC_MSG_NOTICE([PYTHON VERSION:      $PYTHON_VERSION])
AC_MSG_NOTICE([BUILD JAVA LIB:      $with_java])
AC_MSG_NOTICE([INSTALL DKMS SOURCE: $enable_dkms])
AC_MSG_NOTICE([BUILD DKMS MODULE:   $enable_dkms_build])
AC_MSG_NOTICE([KBUILD LINUX MODULE: $enable_kbuild])
AS_IF([test "x$enable_kbuild" = xyes],
      [
AC_MSG_NOTICE([KERNEL RELEASE:      $KERNELRELEASE])
AC_MSG_NOTICE([KDIR:                $KDIR])
      ])
