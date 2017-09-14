# ============================================================================ #
# Copyright (c) 2017, Barcelona Supercomputing Center and International Business Machines.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ============================================================================ #

AC_PREREQ([2.63])
AC_INIT(libPRISM, 0.1, cristo@us.ibm.com)

AC_MSG_NOTICE([ *********************************************************** ])
AC_MSG_NOTICE([ * NOTICE. * ])
AC_MSG_NOTICE([ *********************************************************** ])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_MACRO_DIR([m4])

# Add option DEBUG
AC_ARG_ENABLE(debug,
[  --enable-debug         Enable DEBUG mode (disabled by default)],
 [ debug_support="yes" ], [ debug_support="no" ])

if test "$debug_support" == "yes"; then
    AC_DEFINE(DEBUG,1,[Enable DEBUG support])
    AC_MSG_NOTICE([Activated])
fi

# Add option TIMING
AC_ARG_ENABLE(timing,
[  --enable-timing         Enable TIMING mode (disabled by default)],
 [ timing_support=yes ], [ timing_support=no ])

if test "$enable_timing" == "yes"; then
    AC_DEFINE(TIMING,1, [Enable TIMING support])
fi


#PFM_HOME=/home/Computational/cortega/apps/libpfm-4.6.0
PFM_INCLUDE=${PFM_HOME}/include
PFM_LIB=${PFM_HOME}/lib

#INCLUDES="-I $PFM_INCLUDE"
#LIBS="-L$PFM_LIB"

OMPFLAG=fopenmp

CPPFLAGS="$INCLUDES $CPPFLAGS -pthread -g3"
CXXFLAGS="$INCLUDES $CXXFLAGS -Wall -std=c++0x -$OMPFLAG -pthread"
CFLAGS="$CFLAGS"

if test "$debug_support" == "yes"; then
	CXXFLAGS="$CXXFLAGS -g3 -Werror"
	CFLAGS="$CFLAGS -g3"
	CPPFLAGS="$INCLUDES $CPPFLAGS"
fi

#CPPFLAGS="$INCLUDES $CPPFLAGS -pthread"
#CXXFLAGS="$INCLUDES $CXXFLAGS -Wall -std=c++0x -fopenmp -pthread"
#CFLAGS="$INCLUDES $CFLAGS"

AM_CXXFLAGS=$CXXFLAGS
AM_CPPFLAGS=$CPPFLAGS
AM_CFLAGS=$CFLAGS
AC_SUBST([AM_CXXFLAGS])
AC_SUBST([AM_CPPFLAGS])
AC_SUBST([AM_CFLAGS])

AC_SEARCH_LIBS(pfm_initialize, pfm, [],
	[ AC_MSG_ERROR([Could not find libpfm library. Provide adequate LDFLAGS ]) ],
	[])

AC_CHECK_HEADER(perfmon/pfmlib_perf_event.h,
	[],
  [AC_MSG_ERROR([Couldn't find libpfm headers (perfmon/pfmlib_perf_event.h). Provide adequate CFLAGS/CXXFLAGS: $CFLAGS / $CXXFLAGS. If needed, download from: http://perfmon2.sourceforge.net/]) ],
	[ #ifdef HAVE_PFMLIB.H
		#include <perfmon/pfmlib.h>
		#include <perfmon/pfmlib_perf_event.h>
		#endif
	]
	)
AC_CHECK_HEADER(perfmon/pfmlib.h,
	[],
  [AC_MSG_ERROR([Couldn't find libpfm headers (perfmon/pfmlib.h). Provide adequate CFLAGS/CXXFLAGS: $CFLAGS / $CXXFLAGS. If needed, download from: http://perfmon2.sourceforge.net/]) ],
	[ #ifdef HAVE_PFMLIB.H
		#include <perfmon/pfmlib.h>
		#include <perfmon/pfmlib_perf_event.h>
		#endif
	]
	)

AM_INIT_AUTOMAKE(subdir-objects)

# Checks for programs.
AC_PROG_CXX
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_LIBTOOL
LT_INIT

AC_CONFIG_FILES(Makefile src/Makefile src/sensors/Makefile src/policies/Makefile src/utils/Makefile src/wrappers/Makefile tests/Makefile libPRISM-0.1.pc)

AC_OUTPUT

AC_MSG_NOTICE([ **********************CONFIGURATION********************* ])
if test "$debug_support" == "yes"; then
AC_MSG_NOTICE([ * DEBUG mode:                                  ENABLED * ])
AC_MSG_NOTICE([ *                                   (huge overhead!!!) * ])
else
AC_MSG_NOTICE([ * DEBUG mode:                                 DISABLED * ])
fi

if test "$timing_support" == "yes"; then
AC_MSG_NOTICE([ * TIMING mode:                                  ENABLED * ])
AC_MSG_NOTICE([ *                                   (huge overhead!!!) * ])
else
AC_MSG_NOTICE([ * TIMING mode:                                 DISABLED * ])
fi

AC_MSG_NOTICE([ ******************************************************** ])
AC_MSG_NOTICE([        EXPERIMENTAL LIBRARY - NO SUPPORT PROVIDED        ])
AC_MSG_NOTICE([ ******************************************************** ])

