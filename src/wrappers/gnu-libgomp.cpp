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


#include "gnu-libgomp.h"

pid_t master;
pid_t single_master;
int onBarrier;
int r_PC_task;

struct openmp_task_st {
  void *p1;
  void *p2;
  void *p3;
  long long task_ctr; /* assigned task counter */
};

struct arg_struct {
  void *(*real_func) (void *);
  void * real_args;
  pid_t pid;
  pthread_cond_t wait;
};

void __attribute__((constructor)) wrapper_OMP_init(void) {
  int res = getOpenMPHookPoints();
  master = syscall(SYS_gettid);
	single_master = 0; //unlikely to be 0
	onBarrier = 0;
	r_PC_task = 0;
  debug("Master thread is: %i", (int) master);

  if ( res == 0 ){
    debug("No OpenMPHooks");
  }else{
    debug("OpenMPHooks got: %i",res);
  }

  //Some spacing
  debug("\n\n");
}

void __attribute__ ((destructor)) wrapper_OMP_fini(void) {
}


extern "C" int pthread_cancel(pthread_t thread) {
  debug("Canceling: %i", (int)thread);
  return pthread_cancel_real(thread);
}

pthread_mutex_t count_mutex;

void *trap_threads(void *arguments) {
  struct arg_struct *args = (struct arg_struct *) arguments;
  void *(*routine) (void *) = args->real_func;
  void * real_args = args->real_args;

  pthread_mutex_lock(&count_mutex);


  //TODO this is not portable
  args->pid = syscall(SYS_gettid);
  //debug("Thread reporting its TID: %i", args->pid);

  pthread_cond_signal(&args->wait);
  pthread_mutex_unlock(&count_mutex);

  routine(real_args);

  return NULL;
}



extern "C" int pthread_create (pthread_t* p1, const pthread_attr_t* p2, void *(*p3) (void *), void* p4) {
  return pthread_create_real(p1,p2,p3,p4);
  //if (syscall(SYS_gettid) != master){
  //return pthread_create_real(p1,p2,p3,p4);
  //}

  ////debug("Creating new pthread_create");

  ////First parameter will be our pid
  //struct arg_struct args;
  //args.real_func = p3;
  //args.real_args = p4;
  //args.pid = -1;

  //pthread_cond_init(&args.wait,NULL);
  //pthread_mutex_init(&count_mutex,NULL);

  //pthread_mutex_lock(&count_mutex);

  //int ret = pthread_create_real(p1, p2, &trap_threads, (void *)&args);

  //if (ret == 0){
  //pthread_cond_wait(&args.wait, &count_mutex);
  //}
  ////At this point we know the thread was created


  //pthread_mutex_unlock( &count_mutex);
  //pthread_cond_destroy(&args.wait);
  //pthread_mutex_destroy(&count_mutex);

  //PRISM_pthread_create(args.pid);

  //return ret;
}

extern "C" int pthread_join (pthread_t p1, void **p2) {
  debug("pthread join");
  return pthread_join_real (p1, p2);
}

extern "C" void omp_set_num_threads (int p1) {
  //if (syscall(SYS_gettid) != master)
  //omp_set_num_threads_real(p1);

  ////debug("hook set_num_threads");
  //PRISM_setNumThreads(p1);
  omp_set_num_threads_real(p1);
}

extern "C" int omp_get_thread_num() {
  return omp_get_thread_num_real();
}

extern "C" int omp_get_num_threads() {
  return omp_get_num_threads_real();
}

void handler(int sign) {
	// we still doing work
	// not the best approach with threads
	if (onBarrier == 1 && sign == SIGALRM) {
		PRISM_taskBasedParallel(r_PC_task);
	}
}

extern "C" void GOMP_barrier(void) {

	pid_t me = syscall(SYS_gettid);

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC_task = mfspr(8);
#endif
	if (single_master == me) {
		onBarrier = 1;
		signal(SIGALRM, handler);
		// wait 2 seconds and see if onBarrier = 0;
    alarm(2);
	}

  GOMP_barrier_real();
	onBarrier = 0;
  //exit (0);
}


extern "C" void GOMP_free_thread(void *p1) {
  debug("free_thread");
  GOMP_free_thread_real (p1);
  //exit (0);
}

extern "C" void GOMP_parallel (void *p1, void *p2, unsigned p3, unsigned p4) {
  int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif

  if (syscall(SYS_gettid) != master) {
    GOMP_parallel_real(p1,p2,p3,p4);
    return;
  }

  //debug("Wrapper parallel");

  //std::string omp_num_threads = getVarEnv( "OMP_NUM_THREADS" );
  //int num_threads = -1;
  //if (!omp_num_threads.empty())
  //num_threads = std::stoi(omp_num_threads, 0, 10);

  //PRISM_bestConfig(r_PC);

  //TODO fix
  PRISM_parallelStart(r_PC, 1);
  GOMP_parallel_real (p1, p2, p3, p4);
  PRISM_parallelEnd(r_PC);
}

extern "C" void GOMP_parallel_start(void *p1, void *p2, unsigned p3) {
  int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif

  if (syscall(SYS_gettid) != master) {
    GOMP_parallel_start_real(p1,p2,p3);
    return;
  }
  //debug("Wrapper parallel start %li master is: %i", syscall(SYS_gettid), (int) master);

  //std::string omp_num_threads = getVarEnv( "OMP_NUM_THREADS" );
  //int num_threads = -1;
  //if (!omp_num_threads.empty())
  //num_threads = std::stoi(omp_num_threads, 0, 10);

  //TODO fix
  PRISM_parallelStart(r_PC, 1);
  GOMP_parallel_start_real(p1, p2, p3);
}

extern "C" int GOMP_loop_static_start(long p1, long p2, long p3, long p4, long *p5, long *p6) {
  if (syscall(SYS_gettid) != master) {
    return GOMP_loop_static_start_real(p1,p2,p3,p4,p5,p6);
  }

  int r_PC = -1;
#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif
  //fprintf(stderr,"Parallel loop end in PC: %#x\n", r_PC);
  PRISM_parallelStart(r_PC, 1);
  return GOMP_loop_static_start_real( p1,p2,p3,p4,p5,p6 );
}

extern "C" void GOMP_loop_end(void) {
  debug("loop_end");
  if (syscall(SYS_gettid) != master) {
    GOMP_loop_end_real();
    return;
  }

  int r_PC = -1;
#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif
  //fprintf(stderr,"Parallel loop end in PC: %#x\n", r_PC);
  PRISM_parallelEnd(r_PC);
  GOMP_loop_end_real();
}

extern "C" void GOMP_parallel_end (void) {
  if (syscall(SYS_gettid) != master) {
    GOMP_parallel_end_real();
    return;
  }
  //debug("Wrapper parallel end %li, master is: %i", syscall(SYS_gettid), (int) master);


  int r_PC = -1;
#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif
  PRISM_parallelEnd(r_PC);
  GOMP_parallel_end_real();
}


extern "C" int GOMP_loop_static_next(long *p1, long *p2) {
  //debug("loop static next");
  return GOMP_loop_static_next_real(p1,p2);
}

extern "C" void GOMP_parallel_sections_start(void *p1, void *p2, unsigned p3, unsigned p4) {
  //debug("parallel sections start");
  return GOMP_parallel_sections_start_real(p1,p2,p3,p4);
}

extern "C" int GOMP_loop_runtime_next(long *p1, long *p2) {
  //debug("loop runtime next");
  return GOMP_loop_runtime_next_real(p1,p2);
}

extern "C" int GOMP_loop_runtime_start(long p1, long p2, long p3, long p4, long *p5, long *p6) {
  int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif

  if (syscall(SYS_gettid) != master) {
    return GOMP_loop_runtime_start_real(p1,p2,p3,p4,p5,p6);
  }
  debug("Wrapper parallel runtime start");

  //PRISM_bestConfig(r_PC);

  PRISM_parallelStart(r_PC, 1);
  return GOMP_loop_runtime_start_real(p1,p2,p3,p4,p5,p6);
}

extern "C" int GOMP_loop_guided_start(long p1, long p2, long p3, long p4, long *p5, long *p6) {
  int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif

  if (syscall(SYS_gettid) != master) {
    return GOMP_loop_guided_start_real(p1,p2,p3,p4,p5,p6);
  }
  //debug("Wrapper parallel runtime start");

  //PRISM_bestConfig(r_PC);

  PRISM_parallelStart(r_PC, 1);
  return GOMP_loop_guided_start_real(p1,p2,p3,p4,p5,p6);
}

extern "C" int GOMP_loop_guided_next(long *p1, long *p2) {
  //debug("loop runtime next");
  return GOMP_loop_guided_next_real(p1,p2);
}

extern "C" void GOMP_loop_end_nowait() {
  debug("loop_end_nowait");
  int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif

  if (syscall(SYS_gettid) != master) {
    GOMP_loop_end_nowait_real();
    return;
  }
  debug("Wrapper parallel loop_end_nowait");

  PRISM_parallelEnd(r_PC);
  GOMP_loop_end_nowait_real();
}

extern "C" void GOMP_ordered_start(void) {
  debug("ordered_start");
  int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif

  if (syscall(SYS_gettid) != master){
    GOMP_ordered_start_real();
    return;
  }
  //debug("Wrapper parallel dynamic start");

  //PRISM_bestConfig(r_PC);

  PRISM_parallelStart(r_PC, 1);
  GOMP_ordered_start_real();
  return;
}
extern "C" int GOMP_loop_dynamic_start (long p1, long p2, long p3, long p4, long *p5, long *p6) {
  debug("loop_dynamic_start");
  int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif

  if (syscall(SYS_gettid) != master) {
    return GOMP_loop_dynamic_start_real(p1,p2,p3,p4,p5,p6);
  }
  //debug("Wrapper parallel dynamic start");

  //PRISM_bestConfig(r_PC);

  PRISM_parallelStart(r_PC, 1);
  return GOMP_loop_dynamic_start_real(p1,p2,p3,p4,p5,p6);

}

extern "C" int GOMP_parallel_loop_dynamic (long p1, long p2, long p3, long p4, long *p5, long *p6) {
  debug("parallel_loop_dynamic");

  int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif

  if (syscall(SYS_gettid) != master) {
    return GOMP_parallel_loop_dynamic_real(p1,p2,p3,p4,p5,p6);
  }
  //debug("Wrapper parallel dynamic start");

  //PRISM_bestConfig(r_PC);

  PRISM_parallelStart(r_PC, 1);
  return GOMP_parallel_loop_dynamic_real(p1,p2,p3,p4,p5,p6);

}

extern "C" void GOMP_task(void *p1,void *p2,void *p3,long p4,long p5,int p6, unsigned p7) {
  if (syscall(SYS_gettid) != master) {
    return GOMP_task_real(p1,p2,p3,p4,p5,p6,p7);
    return;
  }

  debug("GOMP_task captured for: %i", (int) syscall(SYS_gettid));
  long long unsigned int r_PC = -1;

#if defined (__powerpc64__) || defined(__powerpc64__)
  r_PC = mfspr(8);
#endif


  //debug("Wrapper parallel");

  //std::string omp_num_threads = getVarEnv( "OMP_NUM_THREADS" );
  //int num_threads = -1;
  //if (!omp_num_threads.empty())
  //num_threads = std::stoi(omp_num_threads, 0, 10);

  PRISM_bestConfig(r_PC);

  //TODO fix
  PRISM_parallelStart(r_PC, 1);
  GOMP_task_real(p1, p2, p3, p4, p5,p6,p7);
  PRISM_parallelEnd(r_PC);
  debug("GOMP task ended");
  return;
}

extern "C" bool GOMP_single_start(void) {
  debug("GOMP_single_start");
	bool ret = GOMP_single_start_real();
	if (ret == true) {
		single_master = syscall(SYS_gettid);
	}
	return ret;
}

#define INC_IF_NOT_NULL(ptr,cnt) (cnt = (ptr == NULL)?cnt:cnt+1)

static int getOpenMPHookPoints() {
  int count = 0;

  /* Obtain @ for GOMP_parallel */
  GOMP_parallel_real =
    (void(*)(void*,void*,unsigned,unsigned)) dlsym (RTLD_NEXT, "GOMP_parallel");
  INC_IF_NOT_NULL(GOMP_parallel_real,count);

  /* Obtain @ for GOMP_parallel_start */
  GOMP_parallel_start_real =
    (void(*)(void*,void*,unsigned)) dlsym (RTLD_NEXT, "GOMP_parallel_start");
  INC_IF_NOT_NULL(GOMP_parallel_start_real,count);

  /* Obtain @ for GOMP_parallel_end */
  GOMP_parallel_end_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_parallel_end");
  INC_IF_NOT_NULL(GOMP_parallel_end_real,count);

  /* Obtain @ for GOMP_barrier */
  GOMP_barrier_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_barrier");
  INC_IF_NOT_NULL(GOMP_barrier_real,count);

  /* Obtain @ for GOMP_atomic_start */
  GOMP_atomic_start_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_atomic_start");
  INC_IF_NOT_NULL(GOMP_atomic_start_real,count);

  /* Obtain @ for GOMP_atomic_end */
  GOMP_atomic_end_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_atomic_end");
  INC_IF_NOT_NULL(GOMP_atomic_end_real,count);

  /* Obtain @ for GOMP_critical_enter */
  GOMP_critical_start_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_critical_start");
  INC_IF_NOT_NULL(GOMP_critical_start_real,count);

  /* Obtain @ for GOMP_critical_end */
  GOMP_critical_end_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_critical_end");
  INC_IF_NOT_NULL(GOMP_critical_end_real,count);

  /* Obtain @ for GOMP_critical_name_start */
  GOMP_critical_name_start_real =
    (void(*)(void**)) dlsym (RTLD_NEXT, "GOMP_critical_name_start");
  INC_IF_NOT_NULL(GOMP_critical_name_start_real,count);

  /* Obtain @ for GOMP_critical_name_end */
  GOMP_critical_name_end_real =
    (void(*)(void**)) dlsym (RTLD_NEXT, "GOMP_critical_name_end");
  INC_IF_NOT_NULL(GOMP_critical_name_end_real,count);

  /* Obtain @ for GOMP_parallel_loop_static_start */
  GOMP_parallel_loop_static_start_real =
    (void(*)(void*,void*,unsigned, long, long, long, long)) dlsym (RTLD_NEXT, "GOMP_parallel_loop_static_start");
  INC_IF_NOT_NULL(GOMP_parallel_loop_static_start_real,count);

  /* Obtain @ for GOMP_parallel_loop_runtime_start */
  GOMP_parallel_loop_runtime_start_real =
    (void(*)(void*,void*,unsigned, long, long, long, long)) dlsym (RTLD_NEXT, "GOMP_parallel_loop_runtime_start");
  INC_IF_NOT_NULL(GOMP_parallel_loop_runtime_start_real,count);

  /* Obtain @ for GOMP_parallel_loop_guided_start */
  GOMP_parallel_loop_guided_start_real =
    (void(*)(void*,void*,unsigned, long, long, long, long)) dlsym (RTLD_NEXT, "GOMP_parallel_loop_guided_start");
  INC_IF_NOT_NULL(GOMP_parallel_loop_guided_start_real,count);

  /* Obtain @ for GOMP_parallel_loop_dynamic_start */
  GOMP_parallel_loop_dynamic_start_real =
    (void(*)(void*,void*,unsigned, long, long, long, long)) dlsym (RTLD_NEXT, "GOMP_parallel_loop_dynamic_start");
  INC_IF_NOT_NULL(GOMP_parallel_loop_dynamic_start_real,count);

  GOMP_parallel_loop_dynamic_real =
    (int(*)(long,long, long, long, long*, long*)) dlsym (RTLD_NEXT, "GOMP_parallel_loop_dynamic");
  INC_IF_NOT_NULL(GOMP_parallel_loop_dynamic_real,count);

  /* Obtain @ for GOMP_loop_static_next */
  GOMP_loop_static_next_real =
    (int(*)(long*,long*)) dlsym (RTLD_NEXT, "GOMP_loop_static_next");
  INC_IF_NOT_NULL(GOMP_loop_static_next_real,count);

  /* Obtain @ for GOMP_loop_runtime_next */
  GOMP_loop_runtime_next_real =
    (int(*)(long*,long*)) dlsym (RTLD_NEXT, "GOMP_loop_runtime_next");
  INC_IF_NOT_NULL(GOMP_loop_runtime_next_real,count);

  /* Obtain @ for GOMP_loop_guided_next */
  GOMP_loop_guided_next_real =
    (int(*)(long*,long*)) dlsym (RTLD_NEXT, "GOMP_loop_guided_next");
  INC_IF_NOT_NULL(GOMP_loop_guided_next_real,count);

  /* Obtain @ for GOMP_loop_dynamic_next */
  GOMP_loop_dynamic_next_real =
    (int(*)(long*,long*)) dlsym (RTLD_NEXT, "GOMP_loop_dynamic_next");
  INC_IF_NOT_NULL(GOMP_loop_dynamic_next_real,count);

  /* Obtain @ for GOMP_loop_static_start */
  GOMP_loop_static_start_real =
    (int(*)(long,long,long,long,long*,long*)) dlsym (RTLD_NEXT, "GOMP_loop_static_start");
  INC_IF_NOT_NULL(GOMP_loop_static_start_real,count);

  /* Obtain @ for GOMP_loop_runtime_start */
  GOMP_loop_runtime_start_real =
    (int(*)(long,long,long,long,long*,long*)) dlsym (RTLD_NEXT, "GOMP_loop_runtime_start");
  INC_IF_NOT_NULL(GOMP_loop_runtime_start_real,count);

  /* Obtain @ for GOMP_loop_guided_start */
  GOMP_loop_guided_start_real =
    (int(*)(long,long,long,long,long*,long*)) dlsym (RTLD_NEXT, "GOMP_loop_guided_start");
  INC_IF_NOT_NULL(GOMP_loop_guided_start_real,count);

  /* Obtain @ for GOMP_loop_dynamic_start */
  GOMP_loop_dynamic_start_real =
    (int(*)(long,long,long,long,long*,long*)) dlsym (RTLD_NEXT, "GOMP_loop_dynamic_start");
  INC_IF_NOT_NULL(GOMP_loop_dynamic_start_real,count);

  /* Obtain @ for GOMP_loop_end */
  GOMP_loop_end_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_loop_end");
  INC_IF_NOT_NULL(GOMP_loop_end_real,count);

  /* Obtain @ for GOMP_loop_end_nowait */
  GOMP_loop_end_nowait_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_loop_end_nowait");
  INC_IF_NOT_NULL(GOMP_loop_end_nowait_real,count);

  /* Obtain @ for GOMP_sections_end */
  GOMP_sections_end_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_sections_end");
  INC_IF_NOT_NULL(GOMP_sections_end_real,count);

  /* Obtain @ for GOMP_sections_end_nowait */
  GOMP_sections_end_nowait_real =
    (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_sections_end_nowait");
  INC_IF_NOT_NULL(GOMP_sections_end_nowait_real,count);

  /* Obtain @ for GOMP_sections_start */
  GOMP_sections_start_real =
    (unsigned(*)(unsigned)) dlsym (RTLD_NEXT, "GOMP_sections_start");
  INC_IF_NOT_NULL(GOMP_sections_start_real,count);

  /* Obtain @ for GOMP_sections_next */
  GOMP_sections_next_real =
    (unsigned(*)(void)) dlsym (RTLD_NEXT, "GOMP_sections_next");
  INC_IF_NOT_NULL(GOMP_sections_next_real,count);

  /* Obtain @ for GOMP_parallel_sections_start */
  GOMP_parallel_sections_start_real =
    (void(*)(void*,void*,unsigned,unsigned)) dlsym (RTLD_NEXT, "GOMP_parallel_sections_start");
  INC_IF_NOT_NULL(GOMP_parallel_sections_start_real,count);

  /* Obtain @ for GOMP_task */
  GOMP_task_real =
    (void(*)(void*,void*,void*,long,long,int,unsigned)) dlsym (RTLD_NEXT, "GOMP_task");
  INC_IF_NOT_NULL(GOMP_task_real,count);

  /* Obtain @ for GOMP_taskwait */
  GOMP_taskwait_real = (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_taskwait");
  INC_IF_NOT_NULL(GOMP_taskwait_real,count);

  /* Obtain @ for GOMP_loop_ordered_static_start */
  GOMP_loop_ordered_static_start_real =
    (int(*)(long, long, long, long, long *, long *)) dlsym (RTLD_NEXT, "GOMP_loop_ordered_static_start");
  INC_IF_NOT_NULL(GOMP_loop_ordered_static_start_real, count);

  /* Obtain @ for GOMP_loop_ordered_runtime_start */
  GOMP_loop_ordered_runtime_start_real =
    (int(*)(long, long, long, long, long *, long *)) dlsym (RTLD_NEXT, "GOMP_loop_ordered_runtime_start");
  INC_IF_NOT_NULL(GOMP_loop_ordered_runtime_start_real, count);

  /* Obtain @ for GOMP_loop_ordered_dynamic_start */
  GOMP_loop_ordered_dynamic_start_real =
    (int(*)(long, long, long, long, long *, long *)) dlsym (RTLD_NEXT, "GOMP_loop_ordered_dynamic_start");
  INC_IF_NOT_NULL(GOMP_loop_ordered_dynamic_start_real, count);

  /* Obtain @ for GOMP_loop_ordered_guided_start */
  GOMP_loop_ordered_guided_start_real =
    (int(*)(long, long, long, long, long *, long *)) dlsym (RTLD_NEXT, "GOMP_loop_ordered_guided_start");
  INC_IF_NOT_NULL(GOMP_loop_ordered_guided_start_real, count);



  GOMP_free_thread_real =
    (void(*)(void *)) dlsym (RTLD_NEXT, "GOMP_free_thread");
  INC_IF_NOT_NULL(GOMP_free_thread_real, count);




  /* Obtain @ for omp_set_lock */
  omp_get_thread_num_real =
    (int(*)(void)) dlsym (RTLD_NEXT, "omp_get_thread_num");
  INC_IF_NOT_NULL(omp_get_thread_num_real, count);

  /* Obtain @ for omp_set_lock */
  omp_set_lock_real =
    (void(*)(void*)) dlsym (RTLD_NEXT, "omp_set_lock");
  INC_IF_NOT_NULL(omp_set_lock_real, count);

  /* Obtain @ for omp_unset_lock */
  omp_unset_lock_real =
    (void(*)(void*)) dlsym (RTLD_NEXT, "omp_unset_lock");
  INC_IF_NOT_NULL(omp_unset_lock_real, count);

  /* Obtain @ for omp_set_num_threads */
  omp_set_num_threads_real =
    (void(*)(int)) dlsym (RTLD_NEXT, "omp_set_num_threads");
  INC_IF_NOT_NULL(omp_set_num_threads_real, count);

  /* Obtain @ for omp_get_num_threads */
  omp_get_num_threads_real =
    (int(*)(void)) dlsym (RTLD_NEXT, "omp_get_num_threads");
  INC_IF_NOT_NULL(omp_set_num_threads_real, count);

  /* Obtain @ for pthread_create */
  pthread_create_real =
    (int(*)(pthread_t*,const pthread_attr_t*,void *(*) (void *),void*))
    dlsym (RTLD_NEXT, "pthread_create");
  INC_IF_NOT_NULL(pthread_create_real, count);

  /* Obtain @ for pthread_join */
  pthread_join_real =
    (int(*)(pthread_t,void**)) dlsym (RTLD_NEXT, "pthread_join");
  INC_IF_NOT_NULL(pthread_join_real, count);

  /* Obtain @ for pthread_cancel */
  pthread_cancel_real =
    (int(*)(pthread_t))dlsym (RTLD_NEXT, "pthread_cancel");
  INC_IF_NOT_NULL(pthread_cancel_real, count);

  /* Obtain @ for pthread_barrier_wait */
  pthread_barrier_wait_real =
    (int(*)(pthread_barrier_t *)) dlsym (RTLD_NEXT, "pthread_barrier_wait");
  INC_IF_NOT_NULL(pthread_barrier_wait_real, count);

  /* Obtain @ for pthread_detach */
  pthread_detach_real = (int(*)(pthread_t)) dlsym (RTLD_NEXT, "pthread_detach");
  INC_IF_NOT_NULL(pthread_detach_real, count);

  /* Obtain @ for GOMP_ordered_start */
  GOMP_ordered_start_real = (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_ordered_start");
  INC_IF_NOT_NULL(GOMP_ordered_start_real,count);

  /* Obtain @ for GOMP_ordered_end */
  GOMP_ordered_end_real = (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_ordered_end");
  INC_IF_NOT_NULL(GOMP_ordered_end_real,count);

  /* Obtain @ for GOMP_single_start */
  GOMP_single_start_real = (bool(*)(void)) dlsym (RTLD_NEXT, "GOMP_single_start");
  INC_IF_NOT_NULL(GOMP_single_start_real,count);

#if 0
  /* Obtain @ for GOMP_ordered_start */
  GOMP_ordered_start_real = (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_ordered_start");
  INC_IF_NOT_NULL(GOMP_ordered_start_real,count);

  /* Obtain @ for GOMP_ordered_end */
  GOMP_ordered_end_real = (void(*)(void)) dlsym (RTLD_NEXT, "GOMP_ordered_end");
  INC_IF_NOT_NULL(GOMP_ordered_end_real,count);
#endif

  /* Any hook point? */
  return count;
}
