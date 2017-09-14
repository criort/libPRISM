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

#include <limits>
#include <string>
#include "src/policies/exploration/ExplorationPolicy.h"

ExplorationPolicy::ExplorationPolicy() {
  debug("EXPLORATION policy starting...");

  std::string aux = getVarEnv("PRISM_EXPLORATION_REPETIONS");
  if (aux.empty())
    REPETIONS = 5;
  else
    REPETIONS = atoi(aux.c_str());

  aux = getVarEnv("PRISM_EXPLORATION_AVOID");
  if (aux.empty())
    AVOID = 0;
  else
    AVOID = atoi(aux.c_str());

  aux = getVarEnv("PRISM_EXPLORATION_WAIT_CHANGE_THREAD");
  if (aux.empty())
    WAIT_CHANGE_THREADS = 1;
  else
    WAIT_CHANGE_THREADS = atoi(aux.c_str());

  aux = getVarEnv("PRISM_EXPLORATION_THRESHOLD");
  if (aux.empty())
    THRESHOLD_TIME = 0.01;
  else
    THRESHOLD_TIME = atof(aux.c_str());

  _path[0] = 48;
  _path[1] = 24;
  _path[2] = 12;
  _path[3] = 6;
  sizePath = 4;
  _pathPref[0] = 16;
  _pathPref[1] = 7;
  _pathPref[2] = 0;
  _pathPref[3] = 1;
  sizePathPref = 4;
  debug("Exploration configuration");
  debug("repetions: %i", REPETIONS);
  debug("avoid: %i", AVOID);
  debug("change_thread: %i", WAIT_CHANGE_THREADS);
  debug("threshold: %f", THRESHOLD_TIME);
}

ExplorationPolicy::~ExplorationPolicy() {
}


void ExplorationPolicy::moduleSMT(int PC, double elapsed) {
  if (_stats[PC].characterized) {
    this->setNumThreads(_stats[PC].bestThreads);
    return;
  }

  int iterations = ++_stats[PC].iterations;
  unsigned int *smtOpt = &_stats[PC].currentOPT;
  if (iterations < REPETIONS) {
    debug("Characterizing... with %i threads", _path[*smtOpt]);
    _stats[PC].times[*smtOpt] += elapsed;
    this->setNumThreads(_path[*smtOpt]);
  } else {
    _stats[PC].iterations = 0;
    _stats[PC].times[*smtOpt] += elapsed;
    _stats[PC].mean[*smtOpt] = _stats[PC].times[*smtOpt] / _stats[PC].REPETIONS;
    debug("Characterization with %i threads FINISHED", _path[*smtOpt]);
    debug("--time accumulated: %.4f", _stats[PC].times[*smtOpt]);
    debug("--mean time is %.4f", _stats[PC].times[*smtOpt]);
    debug("--repetions: %i", _stats[PC].REPETIONS);

    // We did at least SMT8 and SMT4
    if (*smtOpt > 0) {
      // If time(SMT4) > time(SMT8), keep SMT8
      // If overhead(SMT8->SMT4) > gain, keep SMT8
      if (_stats[PC].mean[*smtOpt-1] < _stats[PC].mean[*smtOpt]) {
        debug("%i threads are betther than %i threads", \
            _path[*smtOpt-1], _path[*smtOpt]);
        debug("times were: %.4f and %.4f", \
            _stats[PC].mean[*smtOpt-1], _stats[PC].mean[*smtOpt]);
        _stats[PC].characterized = 1;
        _stats[PC].bestThreads = _path[*smtOpt-1];
        _stats[PC].bestTime = _stats[PC].mean[*smtOpt-1];
        this->setNumThreads(_stats[PC].bestThreads);
        _stats[PC].iterations = 0;
        return;
      }
    }
    // We were in ST
    if (++*smtOpt >= sizePath) {
      debug("no more exploration options, keep it");
      _stats[PC].characterized = 1;
      _stats[PC].bestThreads = _path[*smtOpt-1];
      _stats[PC].bestTime = _stats[PC].mean[*smtOpt-1];
      this->setNumThreads(_stats[PC].bestThreads);
      _stats[PC].iterations = 0;
      return;
    }

    this->setNumThreads(_path[*smtOpt]);
    _stats[PC].changed = WAIT_CHANGE_THREADS;
  }
}

void ExplorationPolicy::modulePrefetcher(int PC, double elapsed) {
  if (_stats[PC].characterizedPref) {
    debug("Prefetcher characterized");
    this->setPrefetcher(_stats[PC].bestPrefetcher);
    return;
  }

  int iterations = ++_stats[PC].iterations;
  unsigned int *prefOpt = &_stats[PC].currentCONF;

  if (iterations < REPETIONS) {
    debug("Characterizing... with %i prefetcher", _pathPref[*prefOpt]);
    _stats[PC].times[*prefOpt] += elapsed;
    this->setPrefetcher(_pathPref[*prefOpt]);
  } else {
    _stats[PC].iterations = 0;
    _stats[PC].times[*prefOpt] += elapsed;
    _stats[PC].mean[*prefOpt] = _stats[PC].times[*prefOpt] / REPETIONS;
    debug("Characterization with %i threads FINISHED", _pathPref[*prefOpt]);
    debug("--time accumulated: %.4f", _stats[PC].times[*prefOpt]);
    debug("--mean time is %.4f", _stats[PC].times[*prefOpt]);
    debug("--repetions: %i", _stats[PC].REPETIONS);

    // We were in the last configuration, get statistics
    if (++(*prefOpt) >= sizePathPref) {
      debug("no more exploration options, keep it");
      unsigned int var = 0;
      double best = _pathPref[0];
      for (unsigned int i = 0; i < sizePathPref; ++i) {
        if (_pathPref[i] < best*1.05) {
          best = _pathPref[i];
          var = i;
        }
      }
      _stats[PC].characterizedPref = 1;
      _stats[PC].bestPrefetcher = _pathPref[var];
      _stats[PC].bestTime = best;
      this->setPrefetcher(_stats[PC].bestPrefetcher);
      return;
    }
    this->setPrefetcher(_pathPref[*prefOpt]);
  }
}

void ExplorationPolicy::init(int PC) {
  debug("First time seeing this, run with 48 threads");
  this->setNumThreads(_path[0]);
  _stats[PC].REPETIONS = REPETIONS;
  _stats[PC].iterations = 0;
  _stats[PC].iterationsPref = 0;
  _stats[PC].numChars = 0;
  _stats[PC].resets = 0;
  _stats[PC].stopCharacterizing = 0;
  _stats[PC].characterized = 0;
  _stats[PC].characterizedPref = 0;
  _stats[PC].currentOPT = 0;
  _stats[PC].currentCONF = 0;
  _stats[PC].lastElapsed = 0;
  _stats[PC].avoid = AVOID;
  for (unsigned int i = 0; i < sizePath; ++i) {
    _stats[PC].times[i] = 0;
  }
  return;
}

void ExplorationPolicy::bestConfig(int PC, int iteration) {
  debug("Choosing best configuration for PC: %#x", PC);

  // Short run, check later
  if (_stats[PC].shortRun > 0) {
    --_stats[PC].shortRun;
    debug("Short Run, avoid");
    return;
  }

  // We changed number of threads, wait for strange behaviors
  if (_stats[PC].changed > 0) {
    debug("Number of threads have changed, this iteration we should wait");
    --_stats[PC].changed;
    this->setNumThreads(_path[_stats[PC].currentOPT]);
    return;
  }

  // First time, initialize everything
  if (iteration == 0) {
    init(PC);
    return;
  }

  int it = this->_Iters.top() - 1;
  double elapsed = (this->_trace[PC]._profile[it].timeEnd/_TICKSPERSECOND_) \
                - (this->_trace[PC]._profile[it].timeStart /_TICKSPERSECOND_);

  debug("Elapsed time from last iteration is: %.4f", elapsed);

  if (elapsed < THRESHOLD_TIME && !_stats[PC].noshort) {
    debug("Short parallel!");
    _stats[PC].shortRun = 100;
    return;
  }
  _stats[PC].noshort = 1;

  // If full characterized check time for different phases
  if ( (_stats[PC].characterized || _stats[PC].characterizedPref) \
      && (elapsed > _stats[PC].bestTime*1.05 \
          || elapsed < _stats[PC].bestTime*0.95) \
      && !_stats[PC].stopCharacterizing ) {
    debug("Starting to characterize again");
    debug("Elapsed time: %.4f", elapsed);
    debug("Best Time is: %.4f", _stats[PC].bestTime);
    debug("REPETIONS: %i", _stats[PC].REPETIONS);
    _stats[PC].numChars++;

    if (_stats[PC].numChars > 5) {
      debug("Characterized %i times, increase REPETIONS", _stats[PC].numChars);
      _stats[PC].REPETIONS += 3;
      _stats[PC].numChars = 0;
      ++_stats[PC].resets;
      if ( _stats[PC].REPETIONS > 15 ) {
        debug("RREPETIONS greater than 5");
        _stats[PC].characterized = 1;
        _stats[PC].stopCharacterizing = 1;
        return;
      }
    }

    _stats[PC].characterized = 0;
    _stats[PC].characterizedPref = 0;
    this->setNumThreads(_path[0]);
    _stats[PC].iterations = 0;
    _stats[PC].iterationsPref = 0;
    _stats[PC].currentOPT = 0;
    _stats[PC].currentCONF = 0;
    _stats[PC].bestTime = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < sizePath; ++i) {
      _stats[PC].times[i] = 0;
    }
    return;
  }

  // Characterize SMT
  if (_stats[PC].characterized && !_stats[PC].stopCharacterizing)
    this->setNumThreads(_stats[PC].bestThreads);
  else
    moduleSMT(PC, elapsed);

  // Characterize Pref if SMT is characterized
  if (_stats[PC].characterizedPref)
    this->setPrefetcher(_stats[PC].bestPrefetcher);
  else if (_stats[PC].characterized)
    modulePrefetcher(PC, elapsed);
}

void ExplorationPolicy::selectBestSMT() {
  int smt = _path[sizePath - 1];
  // This should match sizePath
  double performance[4];
  for (unsigned int i = 0; i < sizePath; ++i) {
    float ipc = 0.0;
    performance[i] = 0.0;
    setNumThreadsPhysical(_path[i]);
    // System-wide performance measurement
    // There should be a cleaner way to do this
    FILE *fp = popen("perf stat -e instructions,cycles sleep 1 2>&1"
      " | grep \"insns per cycle\" | awk \'{print $4}\' | sed s/,/./g", "r");
    fscanf(fp, "%f", &ipc);
    pclose(fp);

    performance[i] = ipc;
    if (i > 0) {
      if ( performance[i-1] > performance[i] ) {
        smt = _path[i-1];
        break;
      }
    }

    smt = _path[i];
  }
  setNumThreadsPhysical(smt);
}

void ExplorationPolicy::selectBestPrefetcher() {
  int pref = _pathPref[sizePath - 1];
  // This should match sizePathPref
  double performance[4];
  for (unsigned int i = 0; i < sizePath; ++i) {
    float ipc = 0.0;
    performance[i] = 0.0;
    setPrefetcher(_pathPref[i]);

    // System-wide performance measurement
    // There should be a cleaner way to do this
    FILE *fp = popen("perf stat -e instructions,cycles sleep 1 2>&1"
      "| grep \"insns per cycle\" | awk \'{print $4}\' | sed s/,/./g", "r");
    fscanf(fp, "%f", &ipc);
    pclose(fp);

    performance[i] = ipc;
    if (i > 0) {
      if ( performance[i-1] > performance[i] ) {
        pref = _pathPref[i-1];
        break;
      }
    }

    pref = _pathPref[i];
  }
  setPrefetcher(pref);
}

void ExplorationPolicy::parallelTask(int PC) {
  selectBestSMT();
  selectBestPrefetcher();
}
