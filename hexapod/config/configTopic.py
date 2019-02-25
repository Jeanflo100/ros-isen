#!/usr/bin/env python3

topicTXmin = -10
topicTXmax = 10
topicTYmin = -10
topicTYmax = 10
topicTZmin = -10
topicTZmax = 10

topicRXmin = -10
topicRXmax = 10
topicRYmin = -10
topicRYmax = 10
topicRZmin = -10
topicRZmax = 10

newLZmin = 0
newLZmax = 100
newRZmin = 0
newRZmax = 100

topicMaximas = [[[topicTXmin, topicTXmax], [topicTYmin, topicTYmax], [topicTZmin, topicTZmax]], [[topicRXmin, topicRXmax], [topicRYmin, topicRYmax], [topicRZmin, topicRZmax]], [[newLZmin, newLZmax], [newRZmin, newRZmax]]]

#topicName = 'holobot1/cmd_vel'
topicName = 'hexapod/serial_command'
