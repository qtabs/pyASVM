from pyASVM import *

DSpaths = ["./demoData/DS1/", "./demoData/DS2/"]
ASVMpaths = ["./demoData/ASVM1/", "./demoData/ASVM2/"]
names = ["A", "B", "D", "E", "F"]

platform = ConsolePlatform(names)

DS = DynamicalSystem(DSpaths[0])
# ASVM = MultiClassASVM(ASVMpaths)

DS.simulate(platform, 1)
# ASVM.simulate(platform, 1)

