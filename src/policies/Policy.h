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

#ifndef SRC_POLICIES_POLICY_H_
#define SRC_POLICIES_POLICY_H_

#include <stdlib.h>
#include <omp.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ptrace.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <sys/user.h>
#include <dirent.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <vector>
#include <algorithm>
#include <list>
#include <stack>
#include <unordered_map>
#include <utility>
#include "src/sensors/Sensor.h"
#include "src/sensors/pmc/PMCDriver.h"
#include "src/utils/utils.h"

extern int updating_threads;
extern int working_threads;

/** Base class for policies.
 * Abstract class for building specific policies
 */
class Policy {
 public:
    // Constructors

    Policy();
    virtual ~Policy();

    // Interface

    /**
     * Start monitoring a parallel region and stop the previous active
     * serial monitoring
     * @param PC Program Counter used as region identifier
     * @param nthreads Original number of threads when starting the parallel region
     * @param level Level of parallel region, 1 is outter parallel region, >1 means there are nested parallel regions
     */
    virtual void parallelStart(uint64_t PC, int nthreads, int level);

    /**
     * Stop monitoring a parallel region and start to recording
     * a serial region
     * @param level Level of parallel region, 1 is outter parallel region, >1 means there are nested parallel regions
     */
    virtual void parallelEnd(int level);

    /**
     * Remember that PID for the thread for future access to the thread
     * @param pid PID of the new created thread that joins to the work
     * @param level Level of parallel region, 1 is outter parallel region, >1 means there are nested parallel regions
     */
    virtual void pthreadCreate(pid_t pid, int level);

    /**
     * Init original number of threads by reading OMP_NUM_THREADS environment variable
     */
    virtual void initNumThreads();

    /**
     * Track original number of threads
     */
    virtual void setNumThreads(int num_threads);

    /**
     * Track original number of threads in physical cores.
     * Useful for nested parallel regions.
     */
    virtual void setNumThreadsPhysical(int num_threads);

    /**
     * Decide if use sudo ppc64_dscr to change the prefetcher or not
     */
    virtual void initPrefetcher();

    /**
     * Set the prefetcher to value for all the existing threads and possibly future threads
     * @param value Prefetcher configuration that will be used in the next parallel region
     */
    virtual void setPrefetcher(int value);

    /**
     * Return current original number of threads
     */
    virtual int getNumThreads();


    /**
     *	Process the current data obtained from the execution of the same parallel region identified with PC,
     *	to get the best configuration for the number of threads and prefetcher
     *	@param PC Parallel region identifier
     *  @iteration Number of iterations that the parallel region has been executed
     */
    virtual void bestConfig(int PC, int iteration) = 0;

    /**
     *  Obtaint the best performance in a parallel region identified with PC, which is composed by tasks
     *  pending to be completed
     *	@param PC Parallel region identifier
     */
    virtual void parallelTask(int PC) = 0;

    void changeSMT(int num_threads);


    void resumeSimpleStats(int toFile);
    void printCounters(FILE *pFile);
    void trace(FILE *pFile);


    struct ProfileThread {
      pid_t pid;
      int prefetcher;
      int smt;
      std::vector< std::pair<std::string, uint64_t > > pmcs;
      std::vector< std::pair<std::string, uint64_t > > pmcs_init;
      std::vector< std::pair<std::string, uint64_t > > pmcs_end;
    };

    struct Stats {
      int shortRun;
      int avoid;
      int changed;
      int REPETIONS;
      int noise;

      unsigned int numChars;
      unsigned int resets;
      int stopCharacterizing;
      int characterized;
      int count;
      int characterizedPref;
      int noPref;

      int iter;
      int iterPref;

      double times[4];
      double SMT[4];
      double Pref[4];
      double meanSMT[4];
      double meanPref[4];

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

    struct Profile {
      int parallelRegion;
      int level;
      double timeStart;
      double timeEnd;
      int nthreads;
      int prefetcher;
      std::unordered_map<pid_t, ProfileThread> _threads;
    };

    struct PcTrace {
      int iter;
      std::unordered_map<int, Profile> _profile;
    };

    void checkActiveThreads();
    void reconfigureThreads();
    void fixAffinity();

    int nthreads;
    int _prefetcher;
    int prefetcherSudo;

    std::vector<pid_t> _threads;
    std::vector<int64_t> _info;
    std::unordered_map<int, PcTrace> _trace;

    std::list< std::pair<uint64_t, uint64_t> > order;

    PMCDriver pmcDriver;

    int _smtLevel;
    int _DSCR;
    std::stack<int> _PCs;
    std::stack<int> _Iters;
    // int actualPC;
    // int actualIter;
    int level;

    pid_t _masterPID;

    double timeStart;
    double timeEnd;

    std::vector< std::pair<std::string, uint64_t > > pmcsStart;
    std::vector< std::pair<std::string, uint64_t > > pmcsEnd;
    double getCounter(std::string, ProfileThread v);


 private:
    // PRIVATE FUNCTIONS
    // void setPrefetcherInternal(int value);
};

#endif  // SRC_POLICIES_POLICY_H_
