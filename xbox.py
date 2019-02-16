#!/usr/bin/env python
#pip install inputs
from inputs import get_gamepad

## Definition of the values we want to read as human understanding
newJoystickLeftLeft = -100
newJoystickLeftRight = 100
newJoystickLeftDown = -100
newJoystickLeftUp = 100

newJoystickRightLeft = -100
newJoystickRightRight = 100
newJoystickRightDown = -100
newJoystickRightUp = 100

newL2Released = 0
newL2Pushed = 100
newR2Released = 0
newR2Pushed = 100

newMaximasJoystickLeft = {
	"L": newJoystickLeftLeft,
	"R": newJoystickLeftRight
	"D": newJoystickLeftDown,
	"U": newJoystickLeftUp,
}

newMaximasJoystickRight = {
	"L": newJoystickRihtLeft,
	"R": newJoystickRightRight
	"D": newJoystickRightDown,
	"U": newJoystickRightUp,
}

newMaximasL2 = {
	"R": newL2Released,
	"P": newL2Pushed
}

newMaximasR2 = {
	"R": newR2Released,
	"P": newR2Pushed
}

newMaximas = {
	"JL": newMaximasJoystickLeft,
	"JR": newMaximasJoystickRight,
	"L2": newMaximasL2,
	"R2": newMaximasR2
}

virtualController = {
	"A": False,
	"B": False,
	"X": False,
	"Y": False,

	"U": False,
	"D": False,
	"L": False,
	"R": False,

	"L1": False,
	"L2": newMaximas["L2"]["R"],
	"R1": False,
	"R2": newMaximas["R2"]["R"],

	"JL": ((newMaximas["JL"]["L"] + newMaximas["JL"]["R"])/2, (newMaximas["JL"]["D"] + newMaximas["JL"]["U"])/2),
	"JR": ((newMaximas["JR"]["L"] + newMaximas["JR"]["R"])/2, (newMaximas["JR"]["D"] + newMaximas["JR"]["U"])/2),,
	"L3": False,
	"R3": False,

	"Start": False,
	"Select": False
	#"Central": False
}


## Definition of the real controller's values
lastJoystickLeftLeft = -32768
lastJoystickLeftRight = 32767
lastJoystickLeftDown = 32767
lastJoystickLeftUp = -32768

lastJoystickRightLeft = -32768
lastJoystickRightRight = 32767
lastJoystickRightDown = 32767
lastJoystickRightUp = -32768

lastL2Released = 0
lastL2Pushed = 1023
lastR2Released = 0
lastR2Pushed = 1023

lastMaximasJoystickLeft = {
	"L": lastJoystickLeftLeft,
	"R": lastJoystickLeftRight
	"D": lastJoystickLeftDown,
	"U": lastJoystickLeftUp,
}

lastMaximasJoystickRight = {
	"L": lastJoystickRihtLeft,
	"R": lastJoystickRightRight
	"D": lastJoystickRightDown,
	"U": lastJoystickRightUp,
}

lastMaximasL2 = {
	"R": lastL2Released,
	"P": lastL2Pushed
}

lastMaximasR2 = {
	"R": lastR2Released,
	"P": lastR2Pushed
}

lastMaximas = {
	"JL": lastMaximasJoystickLeft,
	"JR": lastMaximasJoystickRight,
	"L2": lastMaximasL2,
	"R2": lastMaximasR2
}


## Definition of the map function which change the scale
def map(x0, xMin0, xMax0, xMin1, xMax1):
	return xMin1 + (((x0 - xMin0) * (xMax1 - xMin1)) / (xMax0 - xMin0))


if __name__ == '__main__':
	## Those make the ling between the string received by the controller and the string we have in the virtualController
	simpleButtons = {
		"BTN_SOUTH": ("A", True),
		"BTN_EAST": ("B", True),
		"BTN_NORTH": ("X", True),
		"BTN_WEST": ("Y", True),

		"BTN_TL": ("L1", True),
		"BTN_TR": ("R1", True),

		"BTN_THUMBL": ("L3", False),
		"BTN_THUMBR": ("R3", False),

		"BTN_START": ("Start", True),
		"BTN_SELECT": ("Select", True)
	}
	directionalButtons = {
		"ABS_HAT0X": ("L", "R")
		"ABS_HAT0Y": ("U", "D"),
	}
	analogValues = {
		"ABS_X": ("JL", 0),
		"ABS_Y": ("JL", 1),
		"ABS_RX": ("JR", 0),
		"ABS_RY": ("JR", 1),
		"ABS_Z": ("L2", 2),
		"ABS_RZ": ("R2", 2)
	}

	while 1:
		events = get_gamepad()
		for event in events:
			for key, value in simpleButtons.items():
				if(event.code == key):
					virtualController[value[0]] = not (value[1] ^ event.state == 1)
					break
			for key, value in directionalButtons.items():
				if(event.code == key):
					for i in range(len(value)):
						virtualController[value[i]] = (False, False, True)[i + event.state]
			for key, value in analogValues.items():
				if(event.code == key):
					if(value[1] != 2):
						virtualController[value[0]][value[1]] = map(event.state, lastMaximas[value[0]][("L", "D")[value[1]]], lastMaximas[value[0]][("R", "U")[value[1]]], newMaximas[value[0]][("L", "D")[value[1]]], newMaximas[value[0]][("R", "U")[value[1]]])
					else:
						virtualController[value[0]] = map(event.state, lastMaximas[value[0]]["R"], lastMaximas[value[0]]["P"], newMaximas[value[0]]["R"], newMaximas[value[0]]["P"])
					break

			for key, value in virtualController.items():
				print key, ":", value

	#while(1):
		#events = get_gamepad()
		#for event in events:
			#print event.code, event.state, event.ev_type
