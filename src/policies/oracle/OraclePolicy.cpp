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
#include <vector>
#include <utility>
#include <fstream>
#include <unordered_map>
#include "src/policies/oracle/OraclePolicy.h"

OraclePolicy::OraclePolicy() {
  debug("ORACLE policy starting...");

  // fill the map with the recommended static configuration
  std::string var = getVarEnv("PRISM_ORACLE_CONFIG");
  if (!var.empty()) {
    std::ifstream file(var.c_str());
    std::getline(file, var);
    file.close();
    this->_config = this->configStatic(var);
  }

  debug("Config map size: %lu", this->_config.size());
}

OraclePolicy::~OraclePolicy() {
}

std::unordered_map<int, std::pair<int, int>> \
OraclePolicy::configStatic(std::string var) {
  std::unordered_map<int, std::pair<int, int>> result;
  debug("getting info of oracle policy");
  debug("Var: %s", var.c_str());
  std::string delimiter = ",";

  int pc, smt, pref;

  std::size_t last = 0;
  std::size_t next = 0;
  next = var.find(delimiter, last);

  while (next != std::string::npos) {
    debug("while");
    std::string line = var.substr(last, next-last);
    std::vector<std::string> elements = split(line, ':');

    pc = strtoul(elements[0].c_str(), NULL, 16);
    smt = atoi(elements[1].c_str());
    pref = atoi(elements[2].c_str());
    debug("pc: %i, smt: %i, pref: %i", pc, smt, pref);
    result[pc] = std::pair<int, int>(smt, pref);
    last = next + 1;
    next = var.find(delimiter, last);
  }

  result[pc] = std::pair<int, int>(smt, pref);

  return result;
}

void OraclePolicy::bestConfig(int PC, int iteration) {
  debug("Choosing best configuration for PC: %#x", PC);
  auto it = this->_config.find(PC);
  if (it != this->_config.end()) {
    this->setNumThreads(it->second.first);
    this->setPrefetcher(it->second.second);
    debug("Best configuration for PC: %#x: Prefetcher: %i, Threads: %i\n", \
          PC, it->second.second, it->second.first);
  }
}

void OraclePolicy::parallelTask(int PC) {
}

