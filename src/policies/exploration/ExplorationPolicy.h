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

#ifndef SRC_POLICIES_EXPLORATION_EXPLORATIONPOLICY_H_
#define SRC_POLICIES_EXPLORATION_EXPLORATIONPOLICY_H_

#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/ptrace.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <sys/user.h>
#include <sys/time.h>
#include <dirent.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <omp.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <limits>
#include "src/policies/Policy.h"
#include "src/sensors/pmc/PMCDriver.h"
#include "src/utils/utils.h"


/** Base class for policies.
 * Abstract class for building specific policies
 */
class ExplorationPolicy: public Policy {
 public:
    ExplorationPolicy();

    virtual ~ExplorationPolicy();

    void bestConfig(int PC, int iteration);

    void parallelTask(int PC);

 private:
    void selectBestSMT();
    void selectBestPrefetcher();
    void moduleSMT(int PC, double time);
    void modulePrefetcher(int PC, double time);

    void dirtySMT(int PC, double time);

    void init(int PC);

    struct Stats {
      int shortRun;
      int avoid;
      int changed;
      int REPETIONS;

      unsigned int numChars;
      unsigned int resets;
      int stopCharacterizing;
      int characterized;
      int characterizedPref;

      int iter;
      int iterPref;

      double times[4];
      double mean[4];

      int bestThreads;
      int bestPrefetcher;

      double bestTime;

      double lastElapsed;

      int iterations;
      int iterationsPref;

      unsigned int currentOPT;
      unsigned int currentCONF;

      int noshort;
    };

    std::unordered_map<int, Stats> _stats;
    int _path[4];
    int _pathPref[4];
    double overheadSMT[4];
    unsigned int sizePath;
    unsigned int sizePathPref;
    int REPETIONS;
    int AVOID;
    int WAIT_CHANGE_THREADS;
    double THRESHOLD_TIME;
};

#endif  // SRC_POLICIES_EXPLORATION_EXPLORATIONPOLICY_H_
