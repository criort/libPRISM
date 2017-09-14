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

#ifndef SRC_UTILS_UTILS_H_
#define SRC_UTILS_UTILS_H_

#include <stdlib.h>
#include <errno.h>
#include <string>
#include <sstream>
#include <vector>
#include <cstdint>
#include "./config.h"

#define _FREQUENCY_ 3000000000.0

// POWER Machines
#if defined(__powerpc__) || defined(__ppc__) || defined(__PPC__)
#define stringify(s) tostring(s)
#define tostring(s) #s
#define mfspr(rn) ({uint64_t rval; \
    asm volatile("mfspr %0," stringify(rn) \
        : "=r" (rval)); rval;})

#if defined (__powerpc64__) || defined(__ppc64__) || defined(__PPC64__)
// 64 bits
#define _GETTIME_ (mfspr(268))
// TODO(cortega): this is power7, check power8
#define _TICKSPERSECOND_ (512000000)
#else
// 32 bits
#define _GETTIME_ 0

#endif
#else
#define _GETTIME_ 0
#endif

#ifdef DEBUG
#define debug(M, ...) fprintf(stderr, "[DEBUG] %s:%d: " M "\n", \
    __FILE__, __LINE__, ##__VA_ARGS__); fflush(stderr)
#else
#define debug(M, ...) do {} while (0)
#endif

#ifdef TIMING
#define timingStart() double __time = _GETTIME_
#define timingEnd() __time = (_GETTIME_ - __time) / _TICKSPERSECOND_; \
                             fprintf(stderr, "[TIMING] %s: %.4f\n", \
                                 __func__, __time); \
fflush(stderr)
#else
#define timingStart() do {} while (0)
#define timingEnd() do {} while (0)
#endif

/**
 * Function to read environment variables on Linux
 * @param var Environment variable to be read
 * @return It returns the content of the environment variable
 * */
std::string getVarEnv(std::string var);
std::vector<std::string> split(const std::string &s, char delim);



#endif  // SRC_UTILS_UTILS_H_
