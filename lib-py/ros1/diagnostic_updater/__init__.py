#! /usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-

"""
`diagnostic_updater` contains assorted Python classes to assist in diagnostic
publication. These libraries are commonly used by device drivers as part of the diagnostics
toolchain. The main parts of `diagnostic_updater` are:

 - :class:`DiagnosticStatusWrapper`, a wrapper providing
   convenience functions for working with :class:`diagnostics_msgs.msg.DiagnosticStatus`.

 - :class:`Updater`, a class for managing periodic publishing of the
   :class:`DiagnosticStatusWrapper` output by a set of :class:`DiagnosticTask`.

 - :class:`TopicDiagnostic` and :class:`HeaderlessTopicDiagnostic` for calculating and
   publishing statistics on timestamps and publication frequencies, and
   their corresponding :class:`DiagnosedPublisher` and :class:`HeaderlessDiagnosedPublisher`
   to update the statistics automatically when publications are made to a topic.

Example uses of these classes can be found in `src/example.py`.
"""

from ._diagnostic_status_wrapper import *
from ._diagnostic_updater import *
from ._update_functions import *
from ._publisher import *

__all__ = [
  'DiagnosticStatusWrapper',
  'DiagnosticTask',
  'FunctionDiagnosticTask',
  'CompositeDiagnosticTask',
  'DiagnosticTaskVector',
  'Updater',
  'HeaderlessTopicDiagnostic',
  'TopicDiagnostic',
  'DiagnosedPublisher',
  'FrequencyStatusParam',
  'FrequencyStatus',
  'TimeStampStatusParam',
  'TimeStampStatus',
  'Heartbeat',
]
