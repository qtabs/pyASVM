from time import sleep
from random import gauss


class MotionProxySimulator():
    """ A testing purposed console-based motion object. The object is 
        designed to have 10 joints {A, B, C, ... J} that can be moved in 
        intervals {-1..1} with a top speed of 0.25.
    """

    def __init__(self, initialPosition=None):

        self.LIMITS = {"A":[-1,1], "B":[-1,1], "C":[-1,1], "D":[-1,1], 
                       "E":[-1,1], "F":[-1,1], "G":[-1,1], "H":[-1,1], 
                       "I":[-1,1], "J":[-1,1]}
        self.TOPSPEED = {"A":0.25, "B":0.25, "C":0.25, "D":0.25, "E":0.25, 
                         "F":0.25, "G":0.25, "H":0.25, "I":0.25, "J":0.25}
        
        if initialPosition is None:
            self.angles = {"A":0, "B":0, "C":0, "D":0, "E":0, 
                           "F":0, "G":0, "H":0, "I":0, "J":0}
        else:
            self.angles = initialPosition  


    def readSensors(self, names):
        """ Simulates a sensor reading by adding a small gaussian term to the 
            current known position.
            Input:   
                names: list of strings with the names of the desired joints
            Output:
                read: a dictionary with pairs name:readedValue
        """
        read = {name:(self.angles[name] + gauss(0,0.001)) for name in names}
        return read


    def changeAngles(self, names, deltas, verbose = True):
        """ change the specified joints by a delta amount, preserving
            some realism by moving them whithout exceeding a predetermined
            speed.
                  names: list of strings with the names of the desired joints
                 deltas: list with the desired change for each joint
                verbose: boolean specifying if the motion should report the
                         movements at each step
        """  
        for i in range(len(names)):
            if deltas[i] > (self.LIMITS[names[i]][1] - self.angles[names[i]]):
                print "WARNING! Trying to move", names[i], "over top limit"
                deltas[i] = self.LIMITS[names[i]][1] - self.angles[names[i]]
            elif deltas[i] < (self.LIMITS[names[i]][0] - self.angles[names[i]]):
                print "WARNING! Trying to move", names[i], "under bottom limi"
                deltas[i] = self.LIMITS[names[i]][0] - self.angles[names[i]]      
        
        done = False
        while not done:
            for i in range(len(names)):
                if deltas[i] > 0:
                    change = min(deltas[i], self.TOPSPEED[names[i]])
                    deltas[i] -= change
                else:
                    change = max(deltas[i], -self.TOPSPEED[names[i]])
                    deltas[i] -= change
                self.angles[names[i]] += change
                if verbose:
                    print "Motion:", names[i], " =", self.angles[names[i]]
                sleep(0.25)    
            done = deltas == [0]*len(deltas)

        if verbose: 
            print "Motion: Done!"


    def setAngles(self, names, angles, verbose=True):
        """ Sets the specified joints to a determined angles, preserving
            some realism by moving them whithout exceeding a predetermined
            speed.
            Inputs:
                  names: list of strings with the names of the desired joints
                 angles: list with the final positions for each joint
                verbose: boolean specifying if the motion should report the
                         movements at each step
        """
        for i in range(len(names)):
            if angles[i] > self.LIMITS[names[i]][1]:
                print "WARNING! Target", names[i], "exceeds joint top lim."
                angles[i] = self.LIMITS[names[i]][1]
            elif angles[i] < self.LIMITS[names[i]][0]:
                print "WARNING! Target", names[i], "exceeds joint bottom lim."
                angles[i] = self.LIMITS[names[i]][0]

        done = False 

        while not done:
            for i in range(len(names)):
                dif = angles[i] - self.angles[names[i]]
                if abs(dif) > self.TOPSPEED[names[i]]:
                    delta = (dif / abs(dif)) * self.TOPSPEED[names[i]]
                else:
                    delta = dif
                self.angles[names[i]] += delta
                if verbose:
                    print "Motion:", names[i], " =", self.angles[names[i]]
                sleep(0.1)    
            done = [self.angles[name] for name in names] == angles

        if verbose: 
            print "Motion: Done!"


    def getAngles(self, names, fromSensors=True):
        """ Gets current position of the specified joints
            Inputs:
                   names: list of strings with the names of the desired joints
             fromSensors: boolean value specifying the source of the reading, 
                          being True for sensors and false for direct joints
                          lecture
        """  

        if fromSensors:
            state = self.readSensors(names)
        else:
            state = self.angles

        x = [state[name] for name in names]

        return x


    def getLimits(self, names):
        """ Retunrs the top and bottom limits of the specified joints
            Input:
                names: list of strings with the names of the desired joints
            Output: 
                limits: list with pairs [bottomLimit, topLimit] preserving 
                        the ordering of names          
        """

        limits = [self.LIMITS[name] for name in names]
        return limits



class SpeechProxySimulator():
    """ A testing purposed console-based motion object """


    def __init__(self, speaker=False):
        self.speaker = speaker


    def say(self, text):
        """ Says a given text. The text is simply printed in the console if 
            speaker is false. The input text should be a string
        """
        if speaker:
            print text
        else:
            print text


