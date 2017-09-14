SUBDIRS = sensors utils policies . wrappers

lib_LTLIBRARIES = libPRISM-0.1.la
libPRISM_0_1_la_SOURCES = Driver.cpp

libPRISM_0_1_la_LIBADD = policies/libPRISM_policies.la utils/libPRISM_utils.la
#$(PFM_LIB)/libpfm.a

libPRISM_0_1_la_CXXFLAGS = -Wall -fPIC -shared \
								 -I$(top_srcdir) \
								 -L$(top_srcdir) \
								 -I$(top_srcdir)/src/utils/ \
								 -I$(top_srcdir)/src/policies/ \
								 -I$(top_srcdir)/src/policies/dummy/ \
								 -I$(top_srcdir)/src/policies/oracle/ \
								 -I$(top_srcdir)/src/policies/exploration/ \
								 -I$(top_srcdir)/src/sensors/ \
								 -I$(top_srcdir)/src/sensors/pmc/ \
								 -L$(top_srcdir)/src/sensors/ \
								 $(AM_CXXFLAGS)
