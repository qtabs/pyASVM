from cFunctions import *
from asvmFunctions import *

motion = openMotionProxy()
speech = openSpeechProxy()

DSpaths = ["./demoData/DS1/", "./demoData/DS2/"]
ASVMpaths = ["./demoData/ASVM1/", "./demoData/ASVM2/"]
names = ["A", "B", "D", "E", "F"]

# targets = setRandomPosition(motion, names)
# goToAngles(motion, names, targets)

performDSSimulation(speech, motion, names, DSpaths[0], 1)
performASVMSimulation(speech, motion, names, ASVMpaths, 1)

