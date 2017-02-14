#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file    runner.py
@author  Jakob Erdmann
@date    2017-01-23
@version $Id: runner.py 22929 2017-02-13 14:38:39Z behrisch $


SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2017 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

from __future__ import print_function
from __future__ import absolute_import
import os
import subprocess
import sys
import random
sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import sumolib

sumoBinary = os.environ["SUMO_BINARY"]
PORT = sumolib.miscutils.getFreeSocketPort()
sumoProcess = subprocess.Popen([sumoBinary,
                                '-c', 'sumo.sumocfg',
                                '-S', '-Q',
                                '--remote-port', str(PORT)], stdout=sys.stdout)


def checkMiddleEdges():
    now = traci.simulation.getCurrentTime() / 1000
    for edge in ["middle", "middle2", "middle3"]:
        print("edge=%s current=%s adapted=%s" % (edge,
                                                 traci.edge.getTraveltime(
                                                     edge),
                                                 traci.edge.getAdaptedTraveltime(edge, now)))

vehID = "ego"
traci.init(PORT)
traci.simulationStep()
checkMiddleEdges()
traci.edge.adaptTraveltime("middle", 20)
traci.edge.adaptTraveltime("middle3", 14)
checkMiddleEdges()
traci.vehicle.rerouteTraveltime(vehID, False)
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
traci.close()
sumoProcess.wait()
