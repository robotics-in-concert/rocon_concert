#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
###############################################################################
from concert_utilities.conductor_graph import ConductorStateDotcodeGenerator
from concert_conductor import StateTransitionTable
from qt_dotgraph.pydotfactory import PydotFactory

if __name__ == '__main__':

    dotcode_factory = PydotFactory()

    csdg = ConductorStateDotcodeGenerator(dotcode_factory)
    dotgraph = csdg.generate_dotgraph(StateTransitionTable)

    dotgraph.write_png('state_graph.png') 
