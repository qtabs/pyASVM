import sys

from matrix import *
from math import *
from GMR import *

REALMIN = sys.float_info.min


def performASVMSimulation(speech, motion, names, dataPath, n=10, minI=None, maxI=None):
    """ Performs a simulation of a given A-SVM system. The robot goes first
        to a random position and then execute the ASVM until equilibrium
        is reached. The procedure is repeated n times to test different 
        initial conditions.
        Inputs: 
                speech: speech proxy already opened in Nao
                motion: motion proxy already opened in Nao
                 names: list of strings with the names of the joints involved 
                        in the motion. Ordering of this list and the DS vars
                        should, obviously, match.
              dataPath: String specifying location of the ASVM parameter files
                     n: Number of tries. Default: 10
              max/minI: optionally, the upper/bottom limits of the positions
                        can be set to test DS defined only in a bounded region
                        as lists of numbers. Both limits should be provided.
                        Otherwise, the system will use the mechanical limits
                        of Nao for both, max and min bounds. 
    """
    for i in range(n):
        speech.say("Going to initial position")
        x = setRandomPosition(motion, names, minI, maxI)
        goToAngles(motion, names, x)
        time.sleep(2.0)
        angles = readAngles(motion, names)
        speech.say("Starting A.S.V.M")
        time.sleep(1.0)
        gK = runASVMUntilEq(motion, names, dataPath, 0.25, 0.02)
        speech.say("Equilibrium reached at attractor: " + str(gK))

    speech.say("Simulation finished")  
    
    return None



def runASVMUntilEq(motion, names, dataPath, dt=0.1, waitTime=0.01):
    """ Runs a SVM motion until the equilibrium has been reached.
        Inputs:
           motion: motion proxy already opened in Nao
            names: list of strings with the names of the joints involved in 
                   the motion
         dataPath: path to the ASVM parameter data (a string)
               dt: time interval used to compute dx = v·dt. Default: 0.1
         waitTime: optionally, the sleep time for each iteration (i.e. the 
                   time the system is waiting between iterations). The 
                   waitTime should be experimentally adjusted to obtain a 
                   credible motion. In theory:
                   waitTime = dt - time expended in computation/iteration)
                   waitTime can be used also to accelerate the motion.
                   Default: 0.01
        Outputs:
            gK: the index of the reached attractor
    """

    [Mu, Sigma, SigmaInv, SigmaDet, Priors, target, 
              a, b, g, bias, xa, xb, vb] = loadASVMParameters(dataPath)

    angles = readAngles(motion, names)
    equilibrium = False

    while not equilibrium:
        gK, eq = runGreetingStepASVM(motion, names, a, b, g, bias, xa, xb, vb,  
                                     target, Mu, Sigma, SigmaInv, SigmaDet, 
                                     Priors, dt)
        equilibrium = eq
        time.sleep(waitTime)

    return gK



def runGreetingStepASVM(motion, names, a, b, g, bias, xa, xb, vb, target, Mu, Sigma, SigmaInv, SigmaDet, Priors, dt=0.07, tol=1):
    """ Runs an ASVM step given its parameters and a motion proxy.
        Inputs:
           motion: motion proxy already opened in Nao
            names: list of strings with the names of the joints involved 
                   in the motion.
          a, b, g: list of alpha/beta/gamma numerical lists characterising the 
                   A-SVMs of each class
       xa, xb, vb: list of matrices of support vectors of alpha, beta and the
                   velocities of the beta ones for the A-SVMs of each class
           target: list of vector positions of the attractors of each class 
           Mu, Sigma, SigmaInv, SigmaDet, Priors: rest of DS parameters as 
                                lists of the original objects, characterising
                                the DSs of each class
               dt: time interval used to compute dx = v·dt. Default: 0.1
              tol: multiplicative factor to amplify/decrease the tolerances of 
                   the joints. Default: 1
        Outputs:
               gK: the index of the used attractor in the step
      equilibrium: boolean variable indicating if the step finished in an 
                   equilibrium situation (i.e. velocities under the threshold)
    """
    angles = readAngles(motion, names)
    x = getX(angles, names)
    gk = getLargerHIndex(x, a, b, g, bias, xa, xb, vb, target)
    velocity = modVelocity(x, a[gk], b[gk], g[gk], bias[gk], xa[gk], xb[gk], vb[gk], target[gk], Priors[gk], Mu[gk], Sigma[gk], SigmaInv[gk], SigmaDet[gk])
    dx = [dt*v for v in velocity]
    moveAngles(motion, names, dx)
    equilibrium = equilibriumReached(names, dx, tol)

    return [gk, equilibrium]



def getLargerHIndex(x, a, b, g, bias, xa, xb, vb, target):
    """ Returns the class index with the larger h value. 
        Inputs:
                x: list of numbers representing the current point vector
          a, b, g: list of alpha/beta/gamma numerical lists characterising the 
                   A-SVMs of each class
       xa, xb, vb: list of matrices of support vectors of alpha, beta and the
                   velocities of the beta ones for the A-SVMs of each class
             bias: list of bias numbers for the A-SVM of each class
           target: list of vector positions of the attractors of each class 
        Output: the index of the class with greater h
    """

    h = [getH(x, a[i], b[i], g[i], bias[i], xa[i], xb[i], vb[i], target[i])
                                                       for i in range(len(a))]

    return h.index(max(h))



def modVelocity(x, a, b, g, bias, xa, xb, vb, target, Priors, Mu, Sigma, SigmaInv, SigmaDet, steps=None):
    """ Returns the a-svm modulated velocity given a vector point x as a list,
        the parameters of the chosen DS [target, Priors, Mu, Sigma(Inv, Det)]
        Parameters:
            a = alpha values of each point characterising the 
                A-SVM times their label (list)
            b = beta values of each point belonging to positive class
                characterising the A-SVM (list)
            g = gamma values of each dimension characterising the A-SVM (list)
         bias = real vector characterising the data translation in the A-SVM
           xa = collection of support vectors (matrix)
           xb = collection of beta-support vectors (matrix)
           vb = velocities at beta-support vector points (matrix)
         step = numerical vector (dx1, dx2, ..., dxn) for the gradient (list)
    """

    # Modulation Function h
    h = getH(x, a, b, g, bias, xa, xb, vb, target)

    # Gradient of the modulation function Dh
    Dh = gradH(x, a, b, g, bias, xa, xb, vb, target, steps, h)
    Dh = Matrix.fromList([Dh])

    # Unitary vector in Dh direction Dh1
    Dh1 = sprod(1 / max(Dh.getNorm(), REALMIN), Dh)  # avoids dividing by zero

    # Nominal velocity according to DS + stabiliser 
    velocity = GMRpy(Priors, Mu, Sigma, x, SigmaInv, SigmaDet)
    velocity = Matrix.fromList([velocity]).getTranspose() 
    velocity = stabiliseVelocity(velocity, target, x)

    orthVel = sprod(computeLambda(Dh1, h, velocity), Dh1).getTranspose()
    parVel = velocity - sprod((Dh1 * velocity)[0][0], Dh1).getTranspose()

    modVel = parVel + orthVel

    return modVel.getTranspose().rows[0]



def stabiliseVelocity(velocity, target, x):
    """ Returns stabilised velocity given a velocity vector, attractor 
        position target and current position x 
    """  

    position = Matrix.fromList([x]).getTranspose()
    stabilisationConstant = 0.3 * velocity.getNorm()

    badDir = target - position
    badDir = sprod(1 / max(badDir.getNorm(), REALMIN), badDir)

    badComponent = velocity.getNorm()
    coefficient =  max(badComponent, stabilisationConstant)

    velocity = velocity + sprod(coefficient - badComponent, badDir)

    return velocity



def computeLambda(Dh1, h, velocity):
    """ Returns lambda value for a velocity vector, the real value
        of the modulation function h and its gradient direction Dh1 
    """
    
    speed = velocity.getNorm()
    epsMax = 0.5 * speed
    epsMin = 0.1 * speed
    proj = (Dh1 * velocity)[0][0]

    if h > 0:
        l = max(epsMin, proj);
    else:
        l = max(epsMax, proj);

    return l



def getH(x, a, b, g, bias, xa, xb, vb, target):
    """ Returns the modulation function value for a vector point x (as a list)
        Parameters:       
            a = alpha values of each point characterising the A-SVM
            b = beta values of each point belonging to positive class
                characterising the A-SVM
            g = gamma values of each dimension characterising the A-SVM
         bias = real vector characterising the data translation in the A-SVM
           xa = collection of support vectors (matrix)
           xb = collection of beta-support vectors (matrix)
           vb = velocities at beta-support vector points (matrix)
       target = vector location of the attractor of the chosen DS (vector)

    """

    h = alphaSum(x, a, xa) + betaSum(x, b, xb, vb) - gammaSum(x, g, target) + bias

    return h
    


def gradH(x, a, b, g, bias, xa, xb, vb, target, step=1E-5, steps=None, h=None):
    """ Returns the numerically computed gradient of the modulation function h
        given a vector point x (as a list), a step vector defining the step
        taken in every direction (as lists)
        Parameters:
            a = list of alpha values of each point characterising the A-SVM
            b = list of beta values of each point belonging to positive class
                characterising the A-SVM
            g = list of gamma values of each dimension characterising the A-SVM
         bias = real vector characterising the data translation in the A-SVM
           xa = list of training point vectors (as lists of lists)
       target = vector location of the attractor of the current DS
            y = list with the labels of the training points
           dx = value of the steps for the numerical gradient computation
                (only works if no steps parameter is introduced)
         step = list of the steps (dx1, dx2, ..., dxn) for the gradient. You 
                can use step instead if all dimensions have the same step
            h = the current h value (can be provided to improve performance)
    """

    dim = len(x)

    if h is None:
        h = getH(x, a, b, g, bias, xa, xb, vb, target)

    if steps is None:
        steps = [step for i in range(dim)]

    # As we want to measure the change in each dimension, we prepare a 
    # different x_final for each dimension
    xf = [x for i in range(dim)]

    for i in range(dim):
      xf[i][i] +=  steps[i]    

    gradH = [(getH(xf[i], a, b, g, bias, xa, xb, vd, target) - h) / steps[i] 
                                                          for i in range(dim)]

    return gradH



def alphaSum(x, a, xa):
    """ Returns the first term of the h sum computation """

    alphaS = sum([a[i] * ker(x, xa.getTranspose()[i]) for i in range(xa.n)])

    return alphaS



def betaSum(x, b, xb, vb):
    """ Returns the second term of the h sum computation """

    betaS = sum([b[i] * vb[i][j] * dx2ker(x, xb[i])[j]
                  for i in range(len(b)) for j in range(len(x))])

    return betaS



def gammaSum(x, g, target):
    """ Returns the third term of the h sum computation """

    gammaS = sum([g[i] * dx2ker(x, target.getTranspose()[0])[i] 
                                                      for i in range(len(x))])

    return gammaS



def ker(x1, x2, gamma=0.5):
    """ Returns the result of applying a RBF kernel to vectors x1, x2 with 
        a given gamma (default: gamma = 0.5)
    """

    k = exp(- gamma * sum([(x1[i] - x2[i]) ** 2 for i in range(len(x1))]))

    return k



def dx2ker(x1, x2, gamma=0.5):
    """ Returns the result of applying the derivative of a RBF kernel to 
        vectors x1, x2 with a given gamma (default: gamma = 0.5)
    """

    dk = [2 * gamma * ker(x1, x2) * (x1[i] - x2[i]) for i in range(len(x1))]

    return dk
