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

#include <vector>
#include <string>
#include <utility>
#include <unordered_map>
#include "src/sensors/pmc/PMCDriver.h"

static void read_groups(perf_event_desc_t *fds, int num) {
  uint64_t *values = NULL;
  size_t new_sz, sz = 0;
  int i, evt;
  ssize_t ret;

  /*
   * 	{ u64		nr;
   * 	  { u64		time_enabled; } && PERF_FORMAT_ENABLED
   * 	  { u64		time_running; } && PERF_FORMAT_RUNNING
   * 	  { u64		value;
   * 	    { u64	id;           } && PERF_FORMAT_ID
   * 	  }		cntr[nr];
   * 	} && PERF_FORMAT_GROUP
   *
   * we do not use FORMAT_ID in this program
   */

  for (evt = 0; evt < num; ) {
    int num_evts_to_read;

    num_evts_to_read = perf_get_group_nevents(fds, num, evt);
    new_sz = sizeof(uint64_t) * (3 + num_evts_to_read);

    if (new_sz > sz) {
      sz = new_sz;
      values = reinterpret_cast<uint64_t *>(realloc(values, sz));
    }

    if (!values)
      err(1, "cannot allocate memory for values\n");

    ret = read(fds[evt].fd, values, new_sz);
    if (ret != (ssize_t)new_sz) { /* unsigned */
      if (ret == -1)
        err(1, "cannot read values event %s", fds[evt].name);

      /* likely pinned and could not be loaded */
      warnx("could not read event %d, tried to read %zu bytes, but got %zd",
          evt, new_sz, ret);
    }

    /*
     * propagate to save area
     */
    for (i = evt; i < (evt + num_evts_to_read); i++) {
      values[0] = values[3 + (i - evt)];
      /*
       * scaling because we may be sharing the PMU and
       * thus may be multiplexed
       */
      fds[i].values[0] = values[0];
      fds[i].values[1] = values[1];
      fds[i].values[2] = values[2];
    }
    evt += num_evts_to_read;
  }
  if (values)
    free(values);
}

/*
static void print_counts(perf_event_desc_t *fds, int num) {
  double ratio;
  uint64_t val;
  int i;

  read_groups(fds, num);

  for (i = 0; i < num; i++) {
    val   = perf_scale(fds[i].values);
    ratio = perf_scale_ratio(fds[i].values);

    // separate groups
    if (perf_is_group_leader(fds, i))
      printf("I am group leader\n");

    printf("%'20" PRIu64 " %s (%.2f%% scaling, ena=%'" PRIu64 ", run=%'" PRIu64 ")\n",
        val,
        fds[i].name,
        (1.0-ratio)*100.0,
        fds[i].values[1],
        fds[i].values[2]);

    fds[i].prev_values[0] = fds[i].values[0];
    fds[i].prev_values[1] = fds[i].values[1];
    fds[i].prev_values[2] = fds[i].values[2];
  }
}
*/


PMCDriver::PMCDriver() {
  int ret;

  stringEvents = getVarEnv("PRISM_PMCS");

  if (stringEvents == "")
    stringEvents = "PM_CYC,PM_INST_CMPL";
  const char * events = stringEvents.c_str();

  debug("Initialing PMCDriver with counters: %s", events);

  ret = pfm_initialize();
  if (ret != PFM_SUCCESS)
    debug("init of libPFM failed");

  debug("%s", events);

  this->events = new char[strlen(events)];
  strcpy(this->events, events);
  //snprintf(this->events, strlen(events-1), "%s", events);

  debug("%s", this->events);
  this->num_fds = this->getNum();
}

PMCDriver::PMCDriver(char * events) {
  int ret;

  debug("Initialing PMCDriver with counters: %s", events);

  ret = pfm_initialize();
  if (ret != PFM_SUCCESS)
    debug("init of libPFM failed");

  this->events = new char[strlen(events)];
  snprintf(this->events, strlen(events), events);
}

PMCDriver::~PMCDriver() {
  pfm_terminate();
}


int PMCDriver::startMonitor(pid_t pid) {
  debug("--Start PMCs for pid: %i", pid);

  int ret;
  std::unordered_map<pid_t, infoPID>::iterator exists = _tracking.find(pid);

  if (1) {
    debug("--TRACKING New PID");

    struct infoPID aux;
    aux.fds = NULL;
    aux.num_fds = 0;

    ret = perf_setup_list_events(this->events, &aux.fds, &aux.num_fds);
    if (ret || !aux.num_fds)
      debug("Not able to setup the list of events: %s", this->events);


    int group_fd;
    for (int i = 0; i < aux.num_fds; ++i) {
      int is_group_leader = perf_is_group_leader(aux.fds, i);

      aux.fds[i].hw.read_format = PERF_FORMAT_SCALE;
      if (is_group_leader) {
        group_fd = -1;
        aux.fds[i].hw.disabled = 1;
        aux.fds[i].hw.read_format |= PERF_FORMAT_GROUP;
      } else {
        group_fd = aux.fds[ aux.fds[i].group_leader].fd;
        aux.fds[i].hw.disabled = 0;
      }
      aux.fds[i].fd = perf_event_open(&aux.fds[i].hw, pid, -1, group_fd, 0);
      if (aux.fds[i].fd == -1)
        debug("Cannot attacht event %i with name %s, because errno = %s", \
            i, aux.fds[i].name, strerror(errno));

      if (is_group_leader) {
        aux.fd_leader = aux.fds[i].fd;
      }
    }

    _tracking.insert(std::pair<pid_t, infoPID> (pid, aux));
    ret = ioctl(aux.fd_leader, PERF_EVENT_IOC_ENABLE, 0);
    if (ret)
      debug("ioctl(enable) failed, ret = %i, errno = %s", \
          ret, strerror(errno));
  } else {
    debug("--TRACKING Known PID");
    ret = ioctl(exists->second.fd_leader, PERF_EVENT_IOC_ENABLE, 0);
    if (ret)
      debug("ioctl(enable) failed, ret = %i", ret);
  }

  return ret;
}

std::vector<std::pair<std::string, uint64_t>> \
PMCDriver::readMonitor(pid_t pid) {
  debug("--Read PMCs for pid: %i", pid);

  std::unordered_map<pid_t, infoPID>::iterator exists = _tracking.find(pid);
  if (exists == _tracking.end()) {
    debug("Reading a non started pid");
  }

  std::vector<std::pair<std::string, uint64_t>> results;

  read_groups(exists->second.fds, exists->second.num_fds);
  // print_counts(exists->second.fds, exists->second.num_fds);
  for (int i = 0; i < exists->second.num_fds; ++i) {
    std::string name = exists->second.fds[i].name;
    uint64_t value = (uint64_t) perf_scale(exists->second.fds[i].values);
    std::pair<std::string, uint64_t> aux(name, value);
    results.push_back(aux);
  }

  return results;
}

void PMCDriver::endMonitor(pid_t pid) {
  debug("--Stop PMCs for pid: %i", pid);

  std::unordered_map<pid_t, infoPID>::iterator exists = _tracking.find(pid);

  if (exists == _tracking.end()) {
    debug("Stoping a non started pid");
  }
  int ret = ioctl(exists->second.fd_leader, PERF_EVENT_IOC_DISABLE, 0);
  if (ret)
    debug("endMonitor failed with fd_leader: %i and pid: %i", \
        exists->second.fd_leader, pid);
  for (int i = 0; i < exists->second.num_fds; ++i) {
    ret = close(exists->second.fds[i].fd);
    if (ret)
      debug("Couldn't close fd: %i with name: %s", \
          exists->second.fds[i].fd, exists->second.fds[i].name);
  }
  _tracking.erase(exists);
}


int PMCDriver::getNum() {
  int count = 0;
  std::vector<char*> v;
  unsigned int length = strlen(this->events);
  char *eventsCopy = reinterpret_cast<char *>(malloc(length * sizeof(char)));
  strcpy(eventsCopy, this->events);
  //debug("eventsCopy: %s", eventsCopy);
  //snprintf(this->events, length, events);
  // char* chars_array = strtok(eventsCopy, ",");
  char *saveptr;
  char* chars_array = strtok_r(eventsCopy, ",", &saveptr);
  while (chars_array) {
    //debug("%s", chars_array);
    // chars_array = strtok(NULL, ",");
    chars_array = strtok_r(NULL, ",", &saveptr);
    ++count;
  }
  return count;
}

std::string PMCDriver::getName(int i) {
  int count = 0;
  unsigned int length = strlen(this->events);
  char *eventsCopy = reinterpret_cast<char *>(malloc(length * sizeof(char)));
  strcpy(eventsCopy, this->events);
  //snprintf(this->events, length, events);
  std::vector<char*> v;
  // char* chars_array = strtok(eventsCopy, ",");
  char *saveptr;
  char* chars_array = strtok_r(eventsCopy, ",", &saveptr);
  while (chars_array) {
    //debug("%s", chars_array);
    if (count == i)
      return chars_array;

    // chars_array = strtok(NULL, ",");
    chars_array = strtok_r(NULL, ",", &saveptr);
    ++count;
  }
  return NULL;
}
