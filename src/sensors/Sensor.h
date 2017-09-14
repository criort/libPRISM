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

#ifndef SRC_SENSORS_SENSOR_H_
#define SRC_SENSORS_SENSOR_H_

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include <cstdint>

/** Base class for sensors.
 * Abstract class for building specific monitoring sensors
 */
class Sensor {
 public:
    Sensor();
    virtual ~Sensor();

    /**
     * Start to monitor with the underlying system the different specified CPUs
     * @param cpus Bitmask with the lowest order bit corresponding to the
     * 			first logical CPU and the highest order bit corresponding to the last logical CPU.
     * 			High bit is to monitor that logical CPU, low bit means not to monitor it
     * @return 0 if no error, -1 otherwise
     */
    virtual int startMonitor(int cpus) = 0;


    /**
     * Read to monitor with the underlying system
     * @param pid Corresponding PID that has to be stopped
     * @return Vector of pairs with the following information: Name and value
     */
    virtual std::vector<std::pair<std::string, uint64_t>> \
    readMonitor(pid_t pid) = 0;

    /**
     * End to monitor with the underlying system
     * @param pid Corresponding PID that has to be stopped
     */
    virtual void endMonitor(pid_t pid) = 0;

 private:
};

#endif  // SRC_SENSORS_SENSOR_H_
