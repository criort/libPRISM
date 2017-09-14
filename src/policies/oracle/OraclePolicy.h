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

#ifndef SRC_POLICIES_ORACLE_ORACLEPOLICY_H_
#define SRC_POLICIES_ORACLE_ORACLEPOLICY_H_

#include <sys/time.h>
#include <omp.h>
#include <unistd.h>
#include <stdlib.h>
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
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <utility>
#include "src/policies/Policy.h"
#include "src/sensors/pmc/PMCDriver.h"
#include "src/utils/utils.h"

/** Base class for policies.
 * Abstract class for building specific policies
 */
class OraclePolicy: public Policy {
 public:
    OraclePolicy();

    virtual ~OraclePolicy();

    void bestConfig(int PC, int iteration);

    void parallelTask(int PC);

 private:
    /**	If environment variable PRISM_STATS is set to dump
     *	a raw data will generated with the next information:
     *	PC:iteration:nthreads:prefetcher:smt:IPC
     */
    void RawStats();

    /**	If environment variable PRISM_STATS is set to parallel
     *	a raw data will generated with the next information:
     *	PC:NITER:NTHREADS:PMC1:###:PMC2:###... PMCs: means
     */
    void parallelStats(int toFile);

    /**	If environment variable PRISM_STATS is set to all
     *	a raw data will generated with the next information:
     *	PC:NITER:NTHREADS:timeStampStart:timeStampEnd:IDThread:PMC1:###:PMC2:###...
     */
    void allStats(int toFile);

    /**	If environment variable PRISM_STATS is set to resume
     *	a raw data will generated with the next information:
     *	PC:NITER:NTHREADS:timeStampStart:timeStampEnd
     */
    void resumeStats(int toFile);

    /**	If environment variable PRISM_STATS is set to resumeSimple
     *	a raw data will generated with the next information:
     *	PC:ITERS:totalTime
     */
    void resumeSimpleStats(int toFile);

    /** Read the environment variable PRISM_ORACLE_CONFIG to fullfill
     * the static configuration map that will be used later
     * PRISM_ORACLE_CONFIG syntax: PC:THREADS:PREFETCHER,PC2:THREADS:PREFETCHER
     */
    std::unordered_map<int, std::pair<int, int>> configStatic(std::string var);
    std::unordered_map<int, std::pair<int, int>> _config;
};

#endif  // SRC_POLICIES_ORACLE_ORACLEPOLICY_H_
