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

#ifndef SRC_DRIVER_H_
#define SRC_DRIVER_H_

#include <stdlib.h>
#include <string>
#include <vector>
#include <cstdio>
#include "Sensor.h"
#include "Policy.h"
#include "DummyPolicy.h"
#include "OraclePolicy.h"
#include "ExplorationPolicy.h"
#include "utils/utils.h"
#include "sys/unistd.h"
#include "sys/types.h"


// Object Policy
Policy * _policy;

// Track parallelism level
int max_level;
int level;

void __attribute__((constructor)) PRISM_init(void);

void __attribute__((destructor)) PRISM_fini(void);


/** Library API
 * API to libPRISM
 */

/**
 * API to communicate to PRISM that a parallel region has started
 * @param PC Value of Program Counter where the parallel region starts, used as Identifier for that region
 * @param nthreads Number of threads that originially the parallel region was using
 */
void PRISM_parallelStart(int PC, int nthreads);

/**
 * API to communicate to PRISM that a parallel region ended
 * @param PC Value of Program Counter where the parallel region end, used as Identifier for that region, just for information.
 */
void PRISM_parallelEnd(int PC);

/**
 * API to communicate to PRISM that a pthread was created
 * @param pid PID of the created pthread
 */
void PRISM_pthread_create(pid_t pid);

/**
 * API to communicate to PRISM that the programmer changed the number of threads through the call omp_set_num_threads
 * @param nthreads Number of threads that the programmer wants to use
 */
void PRISM_setNumThreads(int nthreads);

/**
 * API to communicate to PRISM that the program is about to enter in a parallel region identified with PC, PRISM should
 * make the changes in the current configuration to get the best configuration for the active policy
 * @param PC Identifier of the parallel region the program will enter
 */
void PRISM_bestConfig(int PC);

/**
 * API to communicate to PRISM that the program is about to enter in a parallel region identified with PC, which has tasks to be completed
 * PRISM should
 * make the changes in the current configuration to get the best configuration for the active policy
 * @param PC Identifier of the parallel region the program will enter
 */
void PRISM_taskBasedParallel(int PC);


#endif  // SRC_DRIVER_H_
