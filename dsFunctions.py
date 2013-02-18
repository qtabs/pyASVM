from sys import float_info
from math import e, pi, sqrt
from random import random
from time import sleep
from matrix import *
from loadData import *
#from naoFunctions import *
from cFunctions import *

REALMIN = float_info.min


def performDSSimulation(speech, motion, names, dataPath, n, maxLimits=None, minLimits=None):
    """ Performs a simulation of a given Dynamical System. The robot goes 
        to a random position and then execute the Dynamical System until 
        equilibrium is reached. The procedure is repeated n times.
        Inputs: 
                speech: speech proxy already opened in Nao
                motion: motion proxy already opened in Nao
                 names: list of strings with the names of the joints involved 
                        in the motion. Ordering of this list and the DS vars
                        should, obviously, match.
              dataPath: String specifying location of the DS data files
                     n: Number of tries
         max/minLimits: optionally, the upper/bottom limits of the positions
                        can be set to test DS defined only in a bounded region
                        as lists of numbers. Both limits should be provided.
                        Otherwise, the system will use the mechanical limits
                        of Nao for both, max and min bounds. 
    """

    if maxLimits is None or minLimits is None:
        bounded = False
    else:
        bounded = True

    for i in range(n):
        speech.say("Going to initial position")
        x = getRandomPosition(motion, names, minLimits, maxLimits)
        goToAngles(motion, names, x)
        sleep(4.0)
        angles = readAngles(motion, names)
        speech.say("Starting Dynamical System")
        sleep(2.0)
        runDynamicalSystem(motion, names, dataPath)
        speech.say("equilibrium reached")

    speech.say("Simulation finished")



def runDynamicalSystem(motion, names, dataPath, dt=0.12, waitTime=0.01):
    """ Runs a Dynamical System over Nao until equilibrium is reached.
        Inputs:
              motion: motion proxy already opened in Nao
               names: ordered list of strings with the considered joint names
            dataPath: path to the data files describing the Dynamical System
                  dt: optionally, the considered time increment for each step
                      (default: 0.12)
            waitTime: optionally, the sleep time for each iteration (i.e. the 
                      time the system is waiting between iterations). The 
                      waitTime should be experimentally adjusted to obtain a 
                      credible motion. In theory:
                      waitTime = dt - time expended in computation/iteration)
                      waitTime can be used also to accelerate the motion.
                      (default: 0.01)
    """
    # Reading Dynamical System
    Mu, Sigma, SigmaInv, SigmaDet, Priors, xTrans = loadDSParameters(dataPath)
    
    equilibrium = False
    while not equilibrium:
        equilibrium = runDynamcalSystemStep(motion, names, Mu, Sigma, 
                                            SigmaInv, SigmaDet, 
                                            Priors, xTrans, dt)
        sleep(waitTime)



def equilibriumReached(names, speeds, tol=1):
    """ Checks if the motion is in an equilibrium position. For that, the
        function checks if all speeds are similar to zero using the tolerances
        provided in the config file
        Inputs:
            names: list of strings with the joints to be considered in the 
                   motion
            speeds: list of speed values for each of the joints
    """

    localEq = True
    
    for i in range(len(names)):
        if speeds[i] > (tol * TOL[names[i]]): 
            localEq = False
    
    return localEq



def runDynamcalSystemStep(motion, names, Mu, Sigma, SigmaInv, SigmaDet, Priors, xTrans, dt=0.1, tol=1):
    """ Runs a step in the Dynamical System
        Inputs: 
                motion: motion proxy already opened in Nao
                 names: list of strings with the names of the joints involved 
                        in the motion. Ordering of this list and the DS vars
                        should, obviously, match.
          Priors, Mu, Sigma, SigmaInv, SigmaDet, xTrans: parameters of the DS                 
                    dt: time interval of the step, used to compute dx = v dt
                        Default: 0.1
                   tol: equilibirum tolerance factor, used to amplify or 
                        decrease the predefined tolerances of the joints.
                        Default: 1
    """
    angles = readAngles(motion, names)
    x = getX(angles, names)
    velocity = GMRpy(Priors, Mu, Sigma, x, SigmaInv, SigmaDet, xTrans)
    dx = [dt*v for v in velocity]
    moveAngles(motion, names, dx)

    equilibrium = equilibriumReached(names, dx, tol)
    # if equilibrium: speech.say("equilibrium reached")

    return equilibrium



def GMRpy(Priors, Mu, Sigma, x, SigmaInv, SigmaDet, xTrans=None):
    """ Returns the velocity assigned by a Gaussian Mixture represented
        Dynamical System to a point x.
        Inputs:
               x: Position vector as a list of numbers.
          Priors: List of the prior probability of each gaussian of the GMR
              Mu: A list of the means of each gaussians, as Matrix objects
           Sigma: List of covariance matrices (as Matrix) of the gaussians 
        SigmaInv: List of the inverse of the spatial part of the covariance 
                  matrices, also as Matrix objects
        SigmaDet: List with the determinants of the spatial part of the 
                  convariance matrices
          xTrans: Position of the attractor in the input SDR (as a Matrix
                  object). Take into account that the attractor in a GMR 
                  stable DS lies always in the origin, so probably a 
                  translation must be perform when dealing with several DSs
                  of this kind in complex scenarios. If no xTrans is 
                  provided, GMRpy assumes that no translation is needed.
        Output:
               y: A list with the numerical velocities in each direction
    """

    x = Matrix.fromList([x]).getTranspose()  

    # Adjusting intercept if necessearly
    if xTrans is not None:
        x = x - xTrans

    d = Mu[0].getRank()[0]/2    # Dimensions
    K = len(Sigma)              # Number of gaussians

    h = [Priors[j] * gaussPDF(x, Mu[j].getSlice(range(d), [0]), 
                                 Sigma[j].getSlice(range(d), range(d)), 
                                 SigmaInv[j], SigmaDet[j]) 
                                                            for j in range(K)]

    # Normalising
    totalh = max(sum(h), REALMIN)
    h = [hi / totalh for hi in h]

    A = [Sigma[i].getSlice(range(d, 2*d),range(d)) * SigmaInv[i] 
                                                            for i in range(K)]

    b = [(Mu[j].getSlice(range(d, 2*d), [0]) + 
          A[j] * (x - Mu[j].getSlice(range(d), [0]))) 
                                                            for j in range(K)]

    y = Matrix(d,1)
    for j in range(K):
        y += sprod(h[j], b[j])

    return y.getTranspose().rows[0]



def gaussPDF(x, Mu, Sigma, SigmaInv, SigmaDet):
    """ Returns the Probability Density Function of a multivariate Gaussian 
        represented by means and covariance matrices for a given point x.
        Inputs:
                  x: position vector (as a Matrix object)
                 Mu: the mean vector of the gaussian (Matrix object)
              Sigma: covariance matrix of the gaussian (Matrix object)
           SigmaInv: the inverse of the covariance matrix (Matrix object)
           SigmaDet: determinant of the covariance matrix (number)
        Output:
                  p: a single scalar representing the Probability of the point
    """
    gamma = 0.6              # RBF factor
    d = x.getRank()[0]       # Dimension

    # Argument of the exponential
    arg = (((x - Mu).getTranspose() * SigmaInv) * (x - Mu))[0][0]

    # Coefficient of the probability
    coef = 1 / (sqrt((2 * pi) ** d * (abs(SigmaDet) + REALMIN)))

    p = coef * (e ** (- gamma * arg)) 

    return p
