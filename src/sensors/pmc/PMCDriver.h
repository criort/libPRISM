/*
 * Copyright (c) 2017, Barcelona Supercomputing Center and International Business Machines.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:	Cristobal Ortega <cortega@bsc.es>, <cristo@us.ibm.com>
 *
 */

#ifndef SRC_SENSORS_PMC_PMCDRIVER_H_
#define SRC_SENSORS_PMC_PMCDRIVER_H_

#include "PMCDriver.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <locale.h>
#include <sys/ioctl.h>
#include <err.h>
#include <fcntl.h>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <string>
#include <utility>
#include <cstdint>
#include "src/sensors/Sensor.h"
#include "src/utils/utils.h"

#define __STDC_FORMAT_MACROS

#include "perfmon/pfmlib_perf_event.h"
extern "C" {
#include "src/sensors/pmc/perf_util.h"
}

/** Sensor module for reading Performance Monitor Counters
 * Driver that takes care of monitoring different logical CPUs
 */
class PMCDriver: public Sensor {
 public:
    PMCDriver();
    explicit PMCDriver(char * events);
    ~PMCDriver();

    int startMonitor(pid_t pid);
    std::vector<std::pair<std::string, uint64_t>> readMonitor(pid_t pid);
    void endMonitor(pid_t pid);

    int getNum();
    std::string getName(int i);
    int num_fds;
    std::string stringEvents;

 private:
    struct infoPID {
      perf_event_desc_t *fds;
      int num_fds;
      int fd_leader;
    };

    char * events;
    std::unordered_map<pid_t, infoPID> _tracking;

    // TODO(cortega) all this should be removed
    std::vector<int> eventSets;
    std::vector<int> eventsToMonitor;
    std::vector<int64_t> valuesRead;

    /**
     * Function that controls errors caused by the library PAPI, usually prints
     * information on the standard error output and exits
     * @param error Variable containing the error produced by PAPI
     */
    void handle_error(int error);
};

#endif  // SRC_SENSORS_PMC_PMCDRIVER_H_
