lib_LTLIBRARIES = libPRISM-omp-wrapper.la
libPRISM_omp_wrapper_la_SOURCES = gnu-libgomp.cpp #../libPRISM-0.1.la

libPRISM_omp_wrapper_la_CXXFLAGS = -I$(top_srcdir)/ \
								   -I$(top_srcdir)/src/policies/ \
								   -I$(top_srcdir)/src/policies/dummy/ \
								   -I$(top_srcdir)/src/policies/oracle/ \
								   -I$(top_srcdir)/src/policies/exploration/ \
								   -I$(top_srcdir)/src/sensors/ \
								   -I$(top_srcdir)/src/sensors/pmc/ \
								   -I$(top_srcdir)/src/utils/ \
									 -L$(top_srcdir)/src/

libPRISM_omp_wrapper_la_LDFLAGS = -ldl -fopenmp
libPRISM_omp_wrapper_la_LIBADD = ../libPRISM-0.1.la
