from random import random
import cProxies as pseudoProxies
#from naoqi import ALProxy as naoProxies
from config import *


class ConsolePlatform():

    def __init__(self, names, speech=True, initialPosition=None, speaker=False):
        
        self.TOL = CONSOLE_TOLERANCES
        self.openMotionProxy(initialPosition)
        
        if speech:
            self.openSpeechProxy(speaker)
        else:
            self.speech = None

        self.names = names
        self.updateAngles()


    def openMotionProxy(self, initalPosition=None):
        """ Creates a console motion proxy simulator. Returns the proxy object """
        self.motion = pseudoProxies.MotionProxySimulator(initalPosition)


    def openSpeechProxy(self, speaker=False):
        """ Creates a console speech proxy simulator Returns the proxy object """
        self.speech = pseudoProxies.SpeechProxySimulator(speaker)


    def say(self, message):
        if self.speech is None:
            print "Speech:", message
        else:
            self.speech.say(message)


    def updateAngles(self, fromSensors=True):
        """ Reads the angles of specified joints from sensors.
            Two readings are stored in the object:
                angles: dictionary with pairs {name:value}
                x: list with values of the readings preserving
                   the ordering in names list.
        """

        values = self.motion.getAngles(self.names, fromSensors)

        angles = {}
        for i in range(len(self.names)):
            angles[self.names[i]] = values[i]

        self.angles = angles
        self.x = values


    def moveAngles(self, deltas, fractionMaxSpeed=None):
        """ Moves specified joints by a delta.
            Inputs:
                motion: already opened motion proxy
                 names: string list with the desired joint
                deltas: list with the increment for each joint
            Optionally, fractionMaxSpeed determines the maximum speed Nao
            can use to perform the change as a fraction of the whole movement.
            Can be fine tuned to achieve softer and more natural movements.
        """
        self.motion.changeAngles(self.names, deltas)


    def goToAngles(self, positions, fractionMaxSpeed = None):
        """ Makes Nao to change its joints to specific positions.
            Inputs:
                motion: already opened motion proxy
                 names: string list with the desired joints
             positions: list with the final positions for each joint
            Optionally, fractionMaxSpeed determines the maximum speed Nao
            can use to perform the change as a fraction of the whole movement.
        """
        self.motion.setAngles(self.names, positions)
        self.updateAngles()


    def getRandomPosition(self, minI=None, maxI=None):
        """ Choose a random position for a set of joints.
            Inputs: 
              min/maxI: optionally, user defined intervals can be selected as 
                        dictionaries with pairs (jointName: max/min limit)
                        If not specified, mechanical limits of the robot will be
                        used instead
            Output:
                x: a list of integers with the positions of all the joints in 
                   the names list preserving ordering
        """
        if minI==None or maxI==None:
            maximum = [self.motion.getLimits(name)[0][1] for name in self.names]
            minimum = [self.motion.getLimits(name)[0][0] for name in self.names]
        else:
            maximum = [maxI[name] for name in self.names]
            minimum = [minI[name] for name in self.names] 

        x = [(maximum[i]-minimum[i])*random() + minimum[i] for i in range(len(self.names))]

        return x


    def getSubset(self, namesSubSet=None):
        """ Transforms an angles dictionary in a possibly reduced ordered
            list of joint position values x
            Input:
                namesSubSet: A reduced list of strings with the desired
                             joints. If nothing specified, original 
                             names variable of the object will be used.
        """
        if namesSubSet is None:
            names = self.names
        else:
            names = namesSubSet

        x = [self.angles[name] for name in names]
        
        return x


    def equilibriumReached(self, speeds, tol=1):
        """ Checks if the motion is in an equilibrium position. For that, the
            function checks if all speeds are similar to zero using the tolerances
            provided in the config file
            Inputs:
              platform: platform object from pyASVMplatforms library 
                speeds: list of speed values for each of the joints
        """

        localEq = True
        
        for i in range(len(self.names)):
            if speeds[i] > (tol * self.TOL[self.names[i]]): 
                localEq = False
        
        return localEq



class NaoPlatform():

    def __init__(self):
        self.TOL = NAO_TOLERANCES
        self.openMotionProxy(initialPosition)
        
        if speech:
            self.openSpeechProxy(speaker)
        else:
            self.speech = None

        self.names = names
        self.updateAngles()


    def openMotionProxy(self, nao=NAO_HOST, port=NAO_PORT):
        """ Opens a motion proxy on a Nao host """
        
        try:
            self.motion = naoProxies.ALProxy("ALMotion", nao, port)
        except Exception, e:
            print "Could not create proxy to ALMotion"
            print "Error was: ", e
            sys.exit(1)


    def openSpeechProxy(self, nao=NAO_HOST, port=NAO_PORT):
        """ Opens a speech proxy on a Nao host """
       
        try:
            self.speech = ALProxy("ALTextToSpeech", nao, port)
        except Exception,e:
            print "Could not create proxy to ALTextToSpeech"
            print "Error was: ",e
            sys.exit(1)


    def say(self, message):
        if self.speech is None:
            print "Speech:", message
        else:
            self.speech.say(message)


    def updateAngles(self, fromSensors=True):
        """ Reads the angles of specified joints from sensors.
            Two readings are stored in the object:
                angles: dictionary with pairs {name:value}
                x: list with values of the readings preserving
                   the ordering in names list.
        """

        values = self.motion.getAngles(self.names, fromSensors)

        angles = {}
        for i in range(len(self.names)):
            angles[self.names[i]] = values[i]

        self.angles = angles
        self.x = values


    def moveAngles(self, deltas, fractionMaxSpeed=None):
        """ Moves specified joints by a delta.
            Inputs:
                motion: already opened motion proxy
                 names: string list with the desired joint
                deltas: list with the increment for each joint
            Optionally, fractionMaxSpeed determines the maximum speed Nao
            can use to perform the change as a fraction of the whole movement.
            Can be fine tuned to achieve softer and more natural movements.
        """
        self.motion.changeAngles(self.names, deltas)


    def goToAngles(self, positions, fractionMaxSpeed = None):
        """ Makes Nao to change its joints to specific positions.
            Inputs:
                motion: already opened motion proxy
                 names: string list with the desired joints
             positions: list with the final positions for each joint
            Optionally, fractionMaxSpeed determines the maximum speed Nao
            can use to perform the change as a fraction of the whole movement.
        """
        self.motion.setAngles(self.names, positions)
        self.updateAngles()


    def getRandomPosition(self, minI=None, maxI=None):
        """ Choose a random position for a set of joints.
            Inputs: 
              min/maxI: optionally, user defined intervals can be selected as 
                        dictionaries with pairs (jointName: max/min limit)
                        If not specified, mechanical limits of the robot will be
                        used instead
            Output:
                x: a list of integers with the positions of all the joints in 
                   the names list preserving ordering
        """
        if minI==None or maxI==None:
            maximum = [self.motion.getLimits(name)[0][1] for name in self.names]
            minimum = [self.motion.getLimits(name)[0][0] for name in self.names]
        else:
            maximum = [maxI[name] for name in self.names]
            minimum = [minI[name] for name in self.names] 

        x = [(maximum[i]-minimum[i])*random() + minimum[i] for i in range(len(self.names))]

        return x


    def getSubset(self, namesSubSet=None):
        """ Transforms an angles dictionary in a possibly reduced ordered
            list of joint position values x
            Input:
                namesSubSet: A reduced list of strings with the desired
                             joints. If nothing specified, original 
                             names variable of the object will be used.
        """
        if namesSubSet is None:
            names = self.names
        else:
            names = namesSubSet

        x = [self.angles[name] for name in names]
        
        return x


    def equilibriumReached(self, speeds, tol=1):
        """ Checks if the motion is in an equilibrium position. For that, the
            function checks if all speeds are similar to zero using the tolerances
            provided in the config file
            Inputs:
              platform: platform object from pyASVMplatforms library 
                speeds: list of speed values for each of the joints
        """

        localEq = True
        
        for i in range(len(self.names)):
            if speeds[i] > (tol * self.TOL[self.names[i]]): 
                localEq = False
        
        return localEq

