# README
# libPRISM: An Intelligent Adaption of Prefetch and SMT Levels  
## Description: 
 
libPRISM is designed to tune several hardware knobs of a processor.  In its
current form, libPRISM optimize the SMT level and the data prefetcher hardware
knobs on a POWER8 processor for OpenMP applications.  

It works on top of OpenMP, but it can be used for more programming models. See
/src/wrappers folders to see how implement a new wrapper for a different
programming model.  
There is no need to recompile any runtime or the operating system, libPRISM
does library interposition in Linux with LD_PRELOAD variable (see Usage section
for more information).  
libPRISM is designed to be architectural independent. But, it is coded to set
specific hardware knobs on the processor where it was tested. In case you need
to change how a hardware knob is modified or add more hardware knobs, please
see the file src/policies/Policy.cpp  

For more information, please read our published paper on ICS'17 
(libPRISM: an intelligent adaptation of prefetch and SMT levels).  
libPRISM is under a BSD 3-Clause License. Please refer to the file LICENSE for
more information.  

---
## Usage:  

LD_PRELOAD="/path/to/libPRISM/wrapper/" ./application

In OpenMP codes, it is needed to set several environment variables for a better
performance:
export OMP_PLACES="threads"
export OMP_PROC_BIND="spread"
This variables might vary when running in a different processor than POWER8.

There are several environment variables to modify the behavior of libPRISM as
writing a output file with the different regions libPRISM found, modify
thresholds for the algorithm, etc.  
For more information please build the documentation (this is done in the
bootstrap script).  

---

## Contributing: 
Feel free to contribute to this project.  
Google C++ style guide is used for the whole project
(<https://google.github.io/styleguide/cppguide.html>).  
Script *check_style.py* will check all files against the guidelines for you.  
Please, respect these guidelines if possible.  

--- 

## Compilation:  

- mkdir build_DIR
- run bootstrap script: $ ./bootstrap build_DIR
build_DIR will contain the different available versions of the library in
build_DIR/install/*/lib and the documentation in build_DIR/doc/.

One can use the linux standard autotools sequence to customize the installation:
- run configure: $ ./configure
It is possible to check the available options with the -h flag.
Flag --prefix=<INSTALL_DIR> can be useful.
- make
- make install
- make check

Dependencies:
  - libpfm. Point to the path with LDFLAGS/CFLAGS  
  - gcc. libPRISM relies on gcc. In the case you want to use another compiler,
  please, correct the OpenMP flags  in the configure.ac.
  There are hooks for OpenMP from gcc.

Requirements:
  - Permissions to modify the data prefetcher file. In a POWER8 system these
  files are located in /sys/devices/system/cpu/cpuX/dscr, where X goes from 0 to 
  the number of hardware threads available in the system.
  
### Cleanup:

run `cleanup` script to remove everything that can be regenerated.

---  

## Contact
Feel free to contact me at: cortega at bsc.es  

---
