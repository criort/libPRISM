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

#ifndef SRC_WRAPPERS_GNU_LIBGOMP_H_
#define SRC_WRAPPERS_GNU_LIBGOMP_H_

#define __USE_GNU
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <dlfcn.h>
#include <string.h>
#include <signal.h>
//#include <omp.h>
#include "src/Driver.h"
#include "src/utils/utils.h"

/** \brief Specifies the pid of the master thread, which is the first in execute the library */
extern pid_t master;

int gnu_libgomp_4_2_hook_points (int rank);

void __attribute__((constructor)) wrapper_OMP_init(void);
void __attribute__((destructor)) wrapper_OMP_fini(void);
static int getOpenMPHookPoints();




static void (*GOMP_parallel_real)(void*, void*, unsigned, unsigned) = NULL;
static void (*GOMP_parallel_start_real)(void*, void*, unsigned) = NULL;
static void (*GOMP_parallel_end_real)(void) = NULL;
static void (*GOMP_barrier_real)(void) = NULL;
static void (*GOMP_critical_name_start_real)(void**) = NULL;
static void (*GOMP_critical_name_end_real)(void**) = NULL;
static void (*GOMP_critical_start_real)(void) = NULL;
static void (*GOMP_critical_end_real)(void) = NULL;
static void (*GOMP_atomic_start_real)(void) = NULL;
static void (*GOMP_atomic_end_real)(void) = NULL;
static void (*GOMP_parallel_loop_static_start_real)(void*, void*, unsigned, long, long, long, long) = NULL;
static void (*GOMP_parallel_loop_runtime_start_real)(void*, void*, unsigned, long, long, long, long) = NULL;
static void (*GOMP_parallel_loop_dynamic_start_real)(void*, void*, unsigned, long, long, long, long) = NULL;
static void (*GOMP_parallel_loop_guided_start_real)(void*, void*, unsigned, long, long, long, long) = NULL;
static int (*GOMP_loop_static_next_real)(long*, long*) = NULL;
static int (*GOMP_loop_runtime_next_real)(long*, long*) = NULL;
static int (*GOMP_loop_dynamic_next_real)(long*, long*) = NULL;
static int (*GOMP_loop_guided_next_real)(long*, long*) = NULL;
static int (*GOMP_loop_static_start_real)(long, long, long, long, long*, long*) = NULL;
static int (*GOMP_loop_runtime_start_real)(long, long, long, long, long*, long*) = NULL;
static int (*GOMP_loop_guided_start_real)(long, long, long, long, long*, long*) = NULL;
static int (*GOMP_loop_dynamic_start_real)(long, long, long, long, long*, long*) = NULL;

static int (*GOMP_parallel_loop_dynamic_real)(long, long, long, long, long*, long*) = NULL;


static void (*GOMP_loop_end_real)(void) = NULL;
static void (*GOMP_loop_end_nowait_real)(void) = NULL;
static unsigned (*GOMP_sections_start_real)(unsigned) = NULL;
static unsigned (*GOMP_sections_next_real)(void) = NULL;
static void (*GOMP_sections_end_real)(void) = NULL;
static void (*GOMP_sections_end_nowait_real)(void) = NULL;
static void (*GOMP_parallel_sections_start_real)(void*, void*, unsigned,unsigned) = NULL;
static void (*GOMP_task_real)(void*, void*, void*, long, long, int, unsigned) = NULL;
static void (*GOMP_taskwait_real)(void) = NULL;
static int (*GOMP_loop_ordered_static_start_real)(long, long, long, long, long *, long *) = NULL;
static int (*GOMP_loop_ordered_runtime_start_real)(long, long, long, long, long *, long *) = NULL;
static int (*GOMP_loop_ordered_dynamic_start_real)(long, long, long, long, long *, long *) = NULL;
static int (*GOMP_loop_ordered_guided_start_real)(long, long, long, long, long *, long *) = NULL;

static void (*GOMP_ordered_start_real)(void) = NULL;
static void (*GOMP_ordered_end_real)(void) = NULL;
static bool (*GOMP_single_start_real)(void) = NULL;

static void (*GOMP_free_thread_real)(void *) = NULL;

static int (*omp_get_thread_num_real)(void) = NULL;
static void (*omp_set_lock_real)(void *) = NULL;
static void (*omp_unset_lock_real)(void *) = NULL;
static void (*omp_set_num_threads_real)(int) = NULL;
static int (*omp_get_num_threads_real)(void) = NULL;

static int (*pthread_create_real)(pthread_t*, const pthread_attr_t*, void *(*) (void *), void*) = NULL;
static int (*pthread_cancel_real)(pthread_t) = NULL;
static int (*pthread_join_real)(pthread_t, void**) = NULL;
static int (*pthread_detach_real)(pthread_t) = NULL;
static int (*pthread_barrier_wait_real)(pthread_barrier_t *barrier) = NULL;

#endif  // SRC_WRAPPERS_GNU_LIBGOMP_H_
