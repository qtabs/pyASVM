import sys

from random import random
from naoqi import ALProxy
from loadData import *
from config import *


def openMotionProxy(nao=NAO, port=PORT):
    """ Opens a motion proxy on a Nao host. Returns the proxy object """
    
    try:
        motion = ALProxy("ALMotion", nao, port)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e
        sys.exit(1)

    return motion



def openSpeechProxy(nao=NAO, port=PORT):
    """ Opens a speech proxy on a Nao host. Returns the proxy object """
   
    try:
        tts = ALProxy("ALTextToSpeech", nao, port)
    except Exception,e:
        print "Could not create proxy to ALTextToSpeech"
        print "Error was: ",e
        sys.exit(1)

    return tts



def readAngles(motion, names, fromSensors=True):
    """ Reads the angles of specified joints from sensors.
        Inputs:
            motion: already opened motion proxy
             names: string list with the desired joints
        Output:
            values: a dictionary with pairs (Name: Read value)
    """

    angles = motion.getAngles(names, fromSensors)

    values = {}
    for i in range(len(names)):
        values[names[i]] = angles[i]

    return values



def moveAngles(motion, names, deltas, fractionMaxSpeed = 0.07):
    """ Moves specified joints by a delta.
        Inputs:
            motion: already opened motion proxy
             names: string list with the desired joint
            deltas: list with the increment for each joint
        Optionally, fractionMaxSpeed determines the maximum speed Nao
        can use to perform the change as a fraction of the whole movement.
        Can be fine tuned to achieve softer and more natural movements.
    """

    motion.changeAngles(names, deltas, fractionMaxSpeed)

    return None



def goToAngles(motion, names, positions, fractionMaxSpeed = 0.07):
    """ Makes Nao to change its joints to specific positions.
        Inputs:
            motion: already opened motion proxy
             names: string list with the desired joints
         positions: list with the final positions for each joint
        Optionally, fractionMaxSpeed determines the maximum speed Nao
        can use to perform the change as a fraction of the whole movement.
    """

    motion.setAngles(names, positions, fractionMaxSpeed)

    for name in nonPower:
        motion.setStiffnesses(name, stn[NAMESNUM[name]])

    return None



def setRandomPosition(motion, names, minI=None, maxI=None):
    """ Choose a random position for a set of joints.
        Inputs: 
            motion: motion proxy already opened in Nao
             names: list of strings with the names of the joints
          min/maxI: optionally, user defined intervals can be selected as 
                    dictionaries with pairs (jointName: max/min limit)
                    If not specified, mechanical limits of the robot will be
                    used instead
        Output:
            x: a list of integers with the positions of all the joints in 
               the names list preserving ordering
    """
    if minI==None or maxI==None:
        maximum = [motion.getLimits(name)[0][1] for name in names]
        minimum = [motion.getLimits(name)[0][0] for name in names]
    else:
        maximum = [maxI[name] for name in names]
        minimum = [minI[name] for name in names] 

    x = [(maximum[i]-minimum[i])*random() for i in range(len(names))]

    return x



def getX(angles, names):
    """ Transforms an angles dictionary provided by Nao sensors in an ordered
        list of joint position values x
        Inputs:
            angles: angle dictionary provided by Nao's sensor reading
             names: list of strings with the considered joint names in the 
                    desired order   
    """

    x = [angles[name] for name in names]

    return x



