from cFunctions import *
from dsFunctions import *

motion = openMotionProxy()
speech = openSpeechProxy()

DSpaths = ["./demoData/DS1/", "./demoData/DS2/"]
names = ["A", "B", "D", "E", "F"]

# targets = setRandomPosition(motion, names)
# goToAngles(motion, names, targets)

performDSSimulation(speech, motion, names, DSpaths[0], 1)
