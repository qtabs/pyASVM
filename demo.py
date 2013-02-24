from pyASVM import *

motion = openMotionProxy()
speech = openSpeechProxy()

DSpaths = ["./demoData/DS1/", "./demoData/DS2/"]
ASVMpaths = ["./demoData/ASVM1/", "./demoData/ASVM2/"]
names = ["A", "B", "D", "E", "F"]

# DS = DynamicalSystem(DSpaths[0])
ASVM = MultiClassASVM(ASVMpaths)

# DS.simulate(speech, motion, names, 1)
ASVM.simulate(speech, motion, names, 1)

