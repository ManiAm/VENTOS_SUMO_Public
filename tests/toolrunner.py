#!/usr/bin/env python
"""
@file    toolrunner.py
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2008-03-29
@version $Id: toolrunner.py 22608 2017-01-17 06:28:54Z behrisch $

Wrapper script for running tool tests with TextTest.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2017 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
import os
import subprocess
import sys
if len(sys.argv) < 2:
    sys.exit('required argument <tool> missing')
tool = [os.path.join(os.path.dirname(sys.argv[0]), "..", sys.argv[-1])]
if tool[0].endswith(".jar"):
    tool = ["java", "-jar"] + tool

if tool[0].endswith(".py"):
    tool = [os.environ.get('PYTHON', 'python')] + tool

subprocess.call(tool + sys.argv[1:-1], env=os.environ,
                stdout=sys.stdout, stderr=sys.stderr)
