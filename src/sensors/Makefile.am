noinst_LTLIBRARIES = libPRISM_sensors.la

libPRISM_sensors_la_SOURCES = Sensor.cpp \
							  pmc/PMCDriver.cpp \
							  pmc/perf_util.c

libPRISM_sensors_la_CXXFLAGS = -pthread \
				-D_GNU_SOURCE \
				-I$(top_srcdir) \
				-I$(top_srcdir)/src/sensors/pmc/ \
				-L$(top_srcdir)/libpfm/
