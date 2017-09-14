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

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <cstdint>
#include <utility>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <memory>
#include <vector>
#include <fstream>
#include <stdexcept>
#include "src/policies/Policy.h"

Policy::Policy() {
  // TODO(cortega): Auto-generated constructor stub
  // Start and keep always the default system behavior
  debug("Policy object starting...");

  std::string stats = getVarEnv("OMP_NUM_THREADS");
  this->_smtLevel = atoi(stats.c_str());
  this->_DSCR = 0;
  this->_masterPID = getpid();
  pmcDriver.startMonitor(this->_masterPID);
  _threads.push_back(this->_masterPID);  // to track master

  this->level = 0;

  this->initPrefetcher();
  this->_prefetcher = 0;
  // this->pmcsStart = pmcDriver.readMonitor(this->_masterPID);
  this->timeStart = _GETTIME_;
}

Policy::~Policy() {
  this->timeEnd = _GETTIME_;
  // this->pmcsEnd = pmcDriver.readMonitor(this->_masterPID);
  pmcDriver.endMonitor(this->_masterPID);
  changeSMT(48);
  debug("Policy object dying");

  std::string stats = getVarEnv("PRISM_STATS");
  std::string generateReport = getVarEnv("PRISM_GENERATE_REPORT");
  int toFile;

  if (generateReport.empty() || generateReport == "0" )
    toFile = 0;
  else
    toFile = 1;

  if (stats == "resumeSimple")
    this->resumeSimpleStats(toFile);
}

void Policy::resumeSimpleStats(int toFile) {
  debug("allStats starting...");
  FILE *pFile;
  if (toFile) {
    std::string where = getVarEnv("PRISM_REPORT_NAME");
    if (where.empty()) {
      debug("PRISM_REPORT_NAME is empty");
      pFile = stderr;
    } else {
      debug("Writing to file: %s", where.c_str());
      pFile = fopen(where.c_str(), "w");
      if (pFile == NULL)
        debug("Couldn't open file because: %s", strerror(errno));
    }
  } else {
    pFile = stderr;
  }
  debug("Starting to write trace to file...");
  fprintf(pFile, "PC:THREADS:ITERS:TIME:IPC:PREF");
  for (int i = 0; i < pmcDriver.getNum(); ++i) {
    fprintf(pFile, ":%s", pmcDriver.getName(i).c_str());
  }
  fprintf(pFile, ":SMT[exploration]:PREF[exploration]");
  fprintf(pFile, "\n");

  auto it = _trace.begin();  // UNORDERED MAP!

  while (it != _trace.end()) {
    struct PcTrace aux = it->second;
    double duration = 0;
    int iter = 0, nthreads = 0, pref = 0;
    double instr = 0,
           cycles = 0;
    std::vector<std::string> pmcNames(pmcDriver.getNum());
    std::vector<double> pmcValues(pmcDriver.getNum());
    for (int i = 0; i < pmcDriver.getNum(); ++i) {
      pmcNames[i] = pmcDriver.getName(i);
      pmcValues[i] = 0;
    }
    for (iter = 0; iter <= aux.iter; ++iter) {
      struct Profile p = aux._profile[iter];
      nthreads = p.nthreads;
      pref = p.prefetcher;
      duration += (p.timeEnd/_TICKSPERSECOND_) - (p.timeStart/_TICKSPERSECOND_);
      ProfileThread info = p._threads[this->_masterPID];

      for (unsigned int k = 0; k < info.pmcs_end.size(); ++k) {
        pmcValues[k] += info.pmcs_end[k].second - info.pmcs_init[k].second;
        if (info.pmcs_init[k].first == "PM_CYC") {
          cycles += info.pmcs_end[k].second - info.pmcs_init[k].second;
        } else if (info.pmcs_init[k].first == "PM_RUN_INST_CMPL") {
          instr += info.pmcs_end[k].second - info.pmcs_init[k].second;
        }
      }
    }
    for (int k = 0; k < pmcDriver.getNum(); ++k)
      pmcValues[k] = pmcValues[k] / iter;

    double mean = duration / iter;
    double mean_ipc = (instr/cycles);
    fprintf(pFile, "%#x:%i:%i:%.4f:%.4f:%i", \
        it->first, nthreads, iter, mean, mean_ipc, pref);

    fprintf(pFile, ":SMT::%i", _stats[it->first].bestThreads);
    for (unsigned int i = 0; i < 4; i++) {
      double time = _stats[it->first].meanSMT[i];
      int conf = _path[i];
      fprintf(pFile, ":%i:%.6f", conf, time);
    }
    fprintf(pFile, ":PREF::%i", _stats[it->first].bestPrefetcher);
    for (unsigned int i = 0; i < 4; i++) {
      double time = _stats[it->first].meanPref[i];
      int conf = _pathPref[i];
      fprintf(pFile, ":%i:%.4f", conf, time);
    }
    fprintf(pFile, "\n");

    ++it;
  }
  fprintf(pFile, "0:0:%.4f:0.00", \
      (this->timeEnd/_TICKSPERSECOND_) - (this->timeStart/_TICKSPERSECOND_));
  fprintf(pFile, "\n");

  printCounters(pFile);
  trace(pFile);

  fclose(pFile);
}


double Policy::getCounter(std::string name, ProfileThread v) {
  for (unsigned int i = 0; i < v.pmcs_end.size(); ++i )
    if (v.pmcs_init[i].first == name )
      return v.pmcs_end[i].second - v.pmcs_init[i].second;
  return 0.0;
}

void Policy::printCounters(FILE *pFile) {
  debug("Starting to print counters to file...");
  fprintf(pFile, "==== PMCs Readings ====\n");
  fprintf(pFile, "PC:THREADS:ITERS:TIME:TIME_CYCLES:IPC");
  for (int i = 0; i < pmcDriver.getNum(); ++i) {
    fprintf(pFile, ":%s", pmcDriver.getName(i).c_str());
  }
  fprintf(pFile, "\n");

  auto it = _trace.begin();  // UNORDERED MAP!

  while (it != _trace.end()) {
    struct PcTrace aux = it->second;
    double duration = 0;
    int iter = 0;
    int nthreads = 0;
    uint64_t instr = 0,
           cycles = 0;
    std::vector<std::string> pmcNames(pmcDriver.getNum());
    std::vector<double> pmcValues(pmcDriver.getNum());
    for (int i = 0; i < pmcDriver.getNum(); ++i) {
      pmcNames[i] = pmcDriver.getName(i);
      pmcValues[i] = 0;
    }
    for (; iter <= aux.iter; ++iter) {
      struct Profile p = aux._profile[iter];
      nthreads = p.nthreads;
      duration += (p.timeEnd/_TICKSPERSECOND_) - (p.timeStart/_TICKSPERSECOND_);
      ProfileThread info = p._threads[this->_masterPID];


      for (unsigned int k = 0; k < info.pmcs_end.size(); ++k) {
        pmcValues[k] += info.pmcs_end[k].second - info.pmcs_init[k].second;
        if (info.pmcs_init[k].first == "PM_CYC") {
          cycles += info.pmcs_end[k].second - info.pmcs_init[k].second;
        } else if (info.pmcs_init[k].first == "PM_RUN_INST_CMPL") {
          instr += info.pmcs_end[k].second - info.pmcs_init[k].second;
        }
      }
    }
    double mean = duration / iter;
    double mean_cycles = (cycles/_FREQUENCY_) / iter;
    fprintf(pFile, "%#x:%i:%i:%.4f:%.4f:%.4f", \
        it->first, nthreads, iter, mean, mean_cycles, \
        static_cast<double>(instr)/static_cast<double>(cycles));
    for (int i = 0; i < pmcDriver.getNum(); ++i) {
      // fprintf(pFile, "%s",pmcNames[i].c_str());
      fprintf(pFile, ":%f", pmcValues[i]);
    }
    fprintf(pFile, "\n");
    ++it;
  }
}

void Policy::trace(FILE *pFile) {
  debug("printing order");
  fprintf(pFile, "====ORDER TRACE ====\n");
  fprintf(pFile, "PC:ITERATION\n");
  unsigned int moment = 0;
  for (auto it = this->order.begin(); it != this->order.end(); ++it) {
    unsigned int PC = (*it).first;
    uint64_t iter = (*it).second;
    unsigned int num_threads = _trace[PC]._profile[iter].nthreads;
    unsigned int prefetcher = _trace[PC]._profile[iter].prefetcher;
    int level = _trace[PC]._profile[iter].level;

    double time = (_trace[PC]._profile[iter].timeEnd/_TICKSPERSECOND_) \
                  - (_trace[PC]._profile[iter].timeStart/_TICKSPERSECOND_);

    ProfileThread info = _trace[PC]._profile[iter]._threads[this->_masterPID];

    fprintf(pFile, "%u:%#x:%" PRIu64 ":%u:%u:%i:%.4f", \
        moment, PC, iter, num_threads, prefetcher, level, time);

    double ipc = getCounter("PM_RUN_INST_CMPL", info) \
                 / getCounter("PM_CYC", info);

    fprintf(pFile, ":%.4f",  ipc);
    fprintf(pFile, "\n");
    ++moment;
  }
}

void Policy::initNumThreads() {
  debug("reading OMP_NUM_THREADS");
  std::string omp_num_threads = getVarEnv("OMP_NUM_THREADS");

  if (!omp_num_threads.empty())
    this->nthreads = std::stoi(omp_num_threads, 0, 10);
  else
    this->nthreads = 1;
}

void Policy::initPrefetcher() {
  std::string var = getVarEnv("PRISM_PREFETCHER");
  debug("Choosing if use sudo to set the prefetcher: %s", var.c_str());

  if (var.empty() || var == "0")
    this->prefetcherSudo = 0;
  else
    this->prefetcherSudo = 1;
}

void Policy::changeSMT(int num_threads) {
  unsigned int smt = 8;
  switch (num_threads) {
    case 6:
      smt = 1;
      break;
    case 12:
      smt = 2;
      break;
    case 24:
      smt = 4;
      break;
    case 48:
      smt = 8;
      break;
  }

  std::string folder = "/sys/devices/system/cpu/cpu";
  int on = 1;
  int off = 0;
  for (unsigned int i = 0; i < 48; i+=8) {
    unsigned int j = 0;
    for (; j < smt; ++j) {
      if (i+j == 0)
        continue;
      uint64_t tmp = i+j;
      std::string file = folder + std::to_string(tmp) + "/online";
      debug("Changing ONLINE of %s with value: %i", file.c_str(), on);
      std::ofstream of(file, std::ios::binary);
      of.write(reinterpret_cast<char *>(&on), sizeof(on));
      of.close();
    }
    uint64_t tmp;
    for (; j < 8; ++j) {
      tmp = i+j;
      std::string file = folder + std::to_string(tmp) + "/online";
      debug("Changing OFFLINE of %s with value: %i", file.c_str(), off);
      std::ofstream of(file, std::ios::binary);
      of.write(reinterpret_cast<char *>(&off), sizeof(off));
      of.close();
    }
  }
}

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}


void Policy::fixAffinity() {
  int inc = 1;
  if (this->nthreads == 48) {
    inc = 1;
  } else if (this->nthreads == 24) {
    inc = 2;
  } else if (this->nthreads == 12) {
    inc = 4;
  } else {
    inc = 8;
  }

  cpu_set_t cpuset;

  CPU_ZERO(&cpuset);
  for (int i = 0; i < 96; i+=inc) {
    CPU_SET(i, &cpuset);
  }

  checkActiveThreads();

  for (unsigned int i = 0; i < _threads.size(); ++i) {
    sched_setaffinity(_threads[i], sizeof(cpuset), &cpuset);
  }
}


void Policy::setNumThreadsPhysical(int num_threads) {
  debug("Policy changing num threads to %i", num_threads);
  if (this->nthreads != num_threads) {
    // TODO(cortega): do not rely on ppc64_cpu
    uint64_t tmp = num_threads/6;
    std::string value_str = std::to_string(tmp);
    std::string result = exec(("sudo ppc64_cpu --smt="+value_str).c_str());
    debug("SMT changed is %s", result.c_str());
    result = exec(("sudo ppc64_cpu --smt"));
    debug("SMT is %s", result.c_str());
    this->nthreads = num_threads;
    fixAffinity();
  }
}

void Policy::setNumThreads(int num_threads) {
  debug("Policy changing num threads to %i", num_threads);
  if (this->nthreads != num_threads) {
    debug("Before set threads");
    omp_set_num_threads(num_threads);
    debug("After set threads");
    this->nthreads = num_threads;
  }
}

void setPrefetcherInternal(int value) {
  std::string folder = "/sys/devices/system/cpu/cpu";
  int fd, rc, len;
  char* str;
  uint64_t tmp;
  for (int i = 0; i < 48; ++i) {
    tmp = i;
    std::string file = folder + std::to_string(tmp) + "/dscr";
    debug("Changing DSCR of %s with value: %i", file.c_str(), value);

    fd = open(file.c_str(), O_WRONLY);
    len = asprintf(&str, "%x", value);
    rc = write(fd, str, len);
    free(str);
    if (rc == -1)
      debug("Error writting to DSCR register");
    close(fd);
  }
}

void Policy::setPrefetcher(int value) {
  debug("Policy changing prefetcher to: %i", value);
  if (this->_DSCR != value) {
    setPrefetcherInternal(value);
    this->_DSCR = value;
  }
}

int Policy::getNumThreads() {
  return this->nthreads;
}

void Policy::parallelStart(uint64_t PC, int nthreads, int level) {
  // TODO(cortega): end monitoring serial region


  // non nested
  // if (!this->level) {
  std::unordered_map<int, PcTrace>::iterator exists = _trace.find(PC);

  struct PcTrace *aux;

  if (exists == _trace.end()) {
    aux = new struct PcTrace();
    aux->iter = 0;
  } else {
    aux = &exists->second;
    aux->iter++;
  }

  this->_PCs.push(PC);
  this->_Iters.push(aux->iter);
  int actualPC = PC;
  int actualIter = aux->iter;

  // TODO(cortega):  fix nthreads reporting always 1

  this->_trace[PC] = *aux;
  this->bestConfig(actualPC, actualIter);

  debug("===== PARALLEL STARTED");
  debug("PC: %" PRIu64 "level %i and iteration: %i", PC, level, actualIter);
  debug("with %i threads", this->nthreads);

  struct Profile *p = new struct Profile();
  p->parallelRegion = 1;
  p->level = this->level;
  p->nthreads = this->nthreads;
  p->prefetcher = this->_DSCR;


  ProfileThread *pt = new ProfileThread();
  pt->pid = this->_masterPID;
  pt->prefetcher = 0;
  pt->smt = 0;
  // pmcDriver.startMonitor(this->_masterPID);
  pt->pmcs_init = pmcDriver.readMonitor(this->_masterPID);
  p->_threads[this->_masterPID] = *pt;
  this->_trace[PC]._profile[actualIter] = *p;

  this->order.push_back(std::pair<uint64_t, int64_t> (actualPC, actualIter));
  this->_trace[actualPC]._profile[actualIter].timeStart = _GETTIME_;
  debug("--Executing...");
}

void Policy::parallelEnd(int level) {
    debug("===== PARALLEL ENDED, Level: %i =====", level);

    if (this->_PCs.size() == 0 )
      return;

    int actualPC = this->_PCs.top();
    int actualIter = this->_Iters.top();

    this->_PCs.pop();
    this->_Iters.pop();

    Profile *aux = &_trace[actualPC]._profile[actualIter];
    aux->timeEnd = _GETTIME_;
    aux->_threads[_masterPID].pmcs_end = pmcDriver.readMonitor(_masterPID);
}

void Policy::pthreadCreate(pid_t pid, int level) {
}

void Policy::checkActiveThreads() {
    // debug("checking active threads");
    char     dirname[64];
    snprintf(dirname, sizeof dirname, "/proc/%d/task", getpid());
    // debug("dirname: %s", dirname);
    DIR *dir = opendir(dirname);
    if (dir == NULL) {
      _threads.erase(_threads.begin()+1, _threads.end());  // just master...
      closedir(dir);
      return;
    }
    // int nthreads = this->getNumThreads();
    int nthreads = 48;
    int i = 0;
    std::vector<pid_t> newThreads;
    while (i < nthreads) {
      struct dirent *ent;
      int            value;
      char           dummy;
      errno = 0;
      ent = readdir(dir);
      if (!ent)
        break;

      /* Parse TIDs. Ignore non-numeric entries. */
      if (sscanf(ent->d_name, "%d%c", &value, &dummy) != 1)
        continue;

      /* Ignore obviously invalid entries. */
      if (value < 1)
        continue;
      debug("value: %i", value);
      newThreads.push_back((pid_t) value);  // to track master
      ++i;
    }
    // debug("Erasing %i positions", i);
    // _threads.erase(_threads.begin()+i, _threads.end());
    _threads.erase(_threads.begin(), _threads.end());
    _threads = newThreads;
    closedir(dir);
    debug("Threads available: %lu", _threads.size());
}

void Policy::reconfigureThreads() {
    // We get only the n first threads from the vector
    debug("Reconfiguring threads");
    debug("--size: %lu nthreads: %i", _threads.size(), this->getNumThreads());
    unsigned int i = 0;
    // while (i < _threads.size() && i < this->getNumThreads()) {
    // debug("Reconfigure threads: %i", _threads[i]);
    // ++i;
    // }
    // for (; i < _threads.size(); ++i)
    if (this->getNumThreads() == 0) {
      i = 1;
    } else {
      int size = static_cast<int>(_threads.size());
      unsigned int min = std::min(size, this->getNumThreads());
      i = min;
    }
    _threads.erase(_threads.begin()+i, _threads.end());
    for (i = 0; i < _threads.size(); ++i)
      debug("Reconfigure threads: %i", _threads[i]);
}
