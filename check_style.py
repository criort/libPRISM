#!/usr/bin/env python
# ============================================================================ #
# Copyright (c) 2017, Barcelona Supercomputing Center and International Business Machines.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ============================================================================ #

import os
import sys
from subprocess import Popen, PIPE

CPPLINT="./styleguide/cpplint/cpplint.py"
if not os.path.isfile(CPPLINT):
    print "CPPLINT not present"

for root, dirs, files in os.walk("./src/"):
    for name in files:
	if not name.endswith(tuple([".cpp",".h"])) or name == "perf_util.h" \
		or name == "perf_util.c" \
		or name == "gnu-libgomp.h" \
		or name == "gnu-libgomp.cpp":
		continue
	route = os.path.join(root, name)
        print "Inspecting " + name + "..."
	process = Popen([CPPLINT, route], stdout=PIPE)
	(output, err) = process.communicate()
	exit_code = process.wait()
	if exit_code == 1:
		sys.exit(0)

print "Compliant with the style"
