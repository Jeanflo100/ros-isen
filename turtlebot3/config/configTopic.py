#!/usr/bin/env python3

topicTXmin = -0.22
topicTXmax = 0.22
topicTYmin = -10
topicTYmax = 10
topicTZmin = -10
topicTZmax = 10

topicRXmin = -10
topicRXmax = 10
topicRYmin = -10
topicRYmax = 10
topicRZmin = 2.84
topicRZmax = -2.84

newLZmin = 0
newLZmax = 100
newRZmin = 0
newRZmax = 100

topicMaximas = [[[topicTXmin, topicTXmax], [topicTYmin, topicTYmax], [topicTZmin, topicTZmax]], [[topicRXmin, topicRXmax], [topicRYmin, topicRYmax], [topicRZmin, topicRZmax]], [[newLZmin, newLZmax], [newRZmin, newRZmax]]]

topicName = '/cmd_vel'
