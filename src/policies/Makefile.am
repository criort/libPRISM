SUBDIRS = ../sensors

noinst_LTLIBRARIES = libPRISM_policies.la

libPRISM_policies_la_SOURCES = Policy.cpp Policy.h \
							   dummy/DummyPolicy.cpp dummy/Policy.h \
							   oracle/OraclePolicy.cpp oracle/OraclePolicy.h \
							   exploration/ExplorationPolicy.cpp exploration/ExplorationPolicy.h

libPRISM_policies_la_LIBADD = ../sensors/libPRISM_sensors.la

libPRISM_policies_la_CXXFLAGS = -I./dummy/ \
								-I./oracle/ \
								-I./exploration/ \
				-I$(top_srcdir)/
