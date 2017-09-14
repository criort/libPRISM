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

#include <string>
#include "src/Driver.h"

void __attribute__((constructor)) PRISM_init(void) {
  timingStart();
  debug("Initializing PRISM...");

  std::string selected_policy = getVarEnv("PRISM_POLICY");
  debug("Policy selected: %s.", selected_policy.c_str() );

  if ( selected_policy == "dummy" || selected_policy == "DUMMY" )
    _policy = new DummyPolicy ();
  else if ( selected_policy == "oracle" || selected_policy == "ORACLE" )
    _policy = new OraclePolicy();
  else if ( selected_policy == "exploration" \
      || selected_policy == "EXPLORATION" )
    _policy = new ExplorationPolicy();
  else
    _policy = new DummyPolicy ();

  _policy->initNumThreads();

  std::string selected_level = getVarEnv("PRISM_LEVEL");
  debug("Tracking level: %s", selected_level.c_str() );

  if (selected_level.empty() )
    max_level = -1;
  else
    max_level = atoi(selected_level.c_str());
  level = 0;
  timingEnd();
}

void __attribute__((destructor)) PRISM_fini(void) {
  debug("Exiting PRISM...");
  delete _policy;
}

void PRISM_parallelStart(int PC, int nthreads) {
  ++level;
  debug("Parallel start captured with level: %i", level);
  if ( max_level == -1 || level <= max_level ) {
    timingStart();

    // No one interfiering with us or we with them
    /* TODO(cortega): Should not be here, but since we are ensuring
       thread placement and memory allocation with numctl has to be
       here at the moment
    */
    setenv("LD_PRELOAD", "", 1);

    int num_threads = _policy->getNumThreads();
    _policy->parallelStart(PC, num_threads, level);

    timingEnd();
  }
}

void PRISM_parallelEnd(int PC) {
  debug("Parallel end captured with level: %i", level);
  if (max_level == -1 || level <= max_level) {
    timingStart();
    _policy->parallelEnd(level);
    timingEnd();
  }
  --level;
}

void PRISM_setNumThreads(int nthreads) {
  timingStart();
  _policy->setNumThreads(nthreads);
  timingEnd();
}

void PRISM_pthread_create(pid_t pid) {
  _policy->pthreadCreate(pid, level);
}

void PRISM_bestConfig(int PC) {
  timingStart();
  // _policy->bestConfig(PC);
  timingEnd();
}

void PRISM_taskBasedParallel(int PC) {
  _policy->parallelTask(PC);
}
