from math import e, pi, sqrt, exp
from time import sleep
from matrix import *
from sys import float_info
from cFunctions import *

REALMIN = float_info.min


class DynamicalSystem():

    def __init__(self, datapath='./'):
        if datapath[-1] != '/' and datapath != '':
            print "Warning! Datapath must include a trailing slash!"
            datapath += '/'
        self.datapath = datapath
        self.loadParameters()


    def simulate(self, speech, motion, names, n, maxLimits=None, minLimits=None):
        """ Performs a simulation of a given Dynamical System. The robot goes 
            to a random position and then execute the Dynamical System until 
            equilibrium is reached. The procedure is repeated n times.
            Inputs: 
                    speech: speech proxy already opened in Nao
                    motion: motion proxy already opened in Nao
                     names: list of strings with the names of the joints involved 
                            in the motion. Ordering of this list and the DS vars
                            should, obviously, match.
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
            self.runUntilEq(motion, names)
            speech.say("equilibrium reached")

        speech.say("Simulation finished")


    def runUntilEq(self, motion, names, dt=0.12, waitTime=0.01):
        """ Runs a Dynamical System over Nao until equilibrium is reached.
            Inputs:
                  motion: motion proxy already opened in Nao
                   names: ordered list of strings with the considered joint names
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
        equilibrium = False
        while not equilibrium:
            equilibrium = self.runStep(motion, names, dt)
            sleep(waitTime)


    def runStep(self, motion, names, dt=0.1, tol=1):
        """ Runs a step in the Dynamical System
            Inputs: 
                    motion: motion proxy already opened in Nao
                     names: list of strings with the names of the joints involved 
                            in the motion. Ordering of this list and the DS vars
                            should, obviously, match.
                        dt: time interval of the step, used to compute dx = v dt
                            Default: 0.1
                       tol: equilibirum tolerance factor, used to amplify or 
                            decrease the predefined tolerances of the joints.
                            Default: 1
        """
        angles = readAngles(motion, names)
        x = getX(angles, names)
        velocity = self.GMR(x)
        dx = [dt*v for v in velocity]
        moveAngles(motion, names, dx)
        equilibrium = self.equilibriumReached(names, dx, tol)

        return equilibrium


    def equilibriumReached(self, names, speeds, tol=1):
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


    def GMR(self, x):
        """ Returns the velocity assigned by a Gaussian Mixture represented
            Dynamical System to a point x.
            Inputs:
                   x: Position vector as a list of numbers.
            Output:
                   y: A list with the numerical velocities in each direction
        """

        x = Matrix.fromList([x]).getTranspose()  

        # Adjusting intercept if necessearly
        if self.xTrans is not None:
            x = x - self.xTrans

        d = self.Mu[0].getRank()[0]/2    # Dimensions
        K = len(self.Sigma)              # Number of gaussians

        h = [self.Priors[j] * self.gaussPDF(x, j) for j in range(K)]

        # Normalising
        totalh = max(sum(h), REALMIN)
        h = [hi / totalh for hi in h]

        A = [self.Sigma[i].getSlice(range(d, 2*d),range(d)) * self.SigmaInv[i] for i in range(K)]

        b = [(self.Mu[j].getSlice(range(d, 2*d), [0]) + 
              A[j] * (x - self.Mu[j].getSlice(range(d), [0]))) for j in range(K)]

        y = Matrix(d,1)

        for j in range(K):
            y += sprod(h[j], b[j])

        return y.getTranspose().rows[0]


    def gaussPDF(self, x, k):
        """ Returns the Probability Density Function of a multivariate Gaussian 
            represented by means and covariance matrices for a given point x.
            Input:
                      x: position vector (as a Matrix object)
                      k: index of the desired Gaussian
            Output:
                      p: a single scalar representing the Probability of the point
        """
        gamma = 0.6              # RBF factor
        d = x.getRank()[0]       # Dimension
        mu = self.Mu[k].getSlice(range(d), [0])  # Spatial part of Mu[k]

        # Argument of the exponential
        arg = (((x - mu).getTranspose() * self.SigmaInv[k]) * (x - mu))[0][0]

        # Coefficient of the probability
        coef = 1 / (sqrt((2 * pi) ** d * (abs(self.SigmaDet[k]) + REALMIN)))

        p = coef * (e ** (- gamma * arg)) 

        return p


    def loadParameters(self):  
        """ Load all the DS parameters generated form the MATLAB libraries given
            the path of the files. Path must include trailing slash: path/
            Loaded parameters:
                     Mu: list of matrices (Matrix objects)
                  Sigma: list of matrices (Matrix objects)
               SigmaInv: list of matrices (Matrix objects)
               SigmaDet: list of numbers
                 Priors: list of numbers
                 xTrans: list of matrices (Matrix objects)
        """
        self.Mu = self.loadMatrices(self.datapath + 'Mu*')
        self.Sigma = self.loadMatrices(self.datapath + 'SigmaMat*')
        self.SigmaInv = self.loadMatrices(self.datapath + 'SigmaInv*')

        xTrans = self.loadMatrices(self.datapath + 'xTrans')
        if xTrans == []:
            self.xTrans = None
        else:
            self.xTrans = self.loadMatrices(self.datapath + 'xTrans')[0]

        SigmaDet = self.loadMatrices(self.datapath + 'SigmaDet*')
        self.SigmaDet = SigmaDet[0].getTranspose().rows[0]

        Priors = self.loadMatrices(self.datapath + 'Priors')
        if Priors[0].n == 1:
            self.Priors = Priors[0].getTranspose().rows[0]
        else:
            self.Priors = Priors[0].rows[0]


    def loadMatrices(self, globpattern):
        """ Loads from files a list of all matrices in files given a name with a
            wildcard. If a single name is provided, the output is still a list
            with a single element
        """

        matrices = []

        for fname in sorted(glob(globpattern)):
            matrices.append(Matrix.readGrid(fname))

        return matrices



class MultiClassDynamicalSystem():

    def __init__(self, datapaths):
        if len(datapaths) == 1 and datapaths != '':
            print "Warning: only one datapath provided!"
            print "Are you sure this is a multiclass DS?"
        self.cardinality = len(datapaths)
        self.datapaths = datapaths
        self.DS = [DynamicalSystem(self.datapaths[i]) for i in self.cardinality]



class SingleClassASVM():

    def __init__(self, datapath='./'):
        self.datapath = datapath
        self.loadParameters()
        self.kernelG = 0.5


    def modVelocity(self, x, steps=None):
        """ Returns the a-svm modulated velocity given a vector point x as a list.
            Parameters:
             step = numerical vector (dx1, dx2, ..., dxn) for the gradient (list)
        """

        # Modulation Function h
        h = self.getH(x)

        # Gradient of the modulation function Dh
        Dh = self.gradH(x, steps, h)
        Dh = Matrix.fromList([Dh])

        # Unitary vector in Dh direction Dh1
        Dh1 = sprod(1 / max(Dh.getNorm(), REALMIN), Dh)  # avoids dividing by zero

        # Nominal velocity according to DS + stabiliser 
        velocity = self.DS.GMR(x)
        velocity = Matrix.fromList([velocity]).getTranspose() 
        velocity = self.stabiliseVelocity(velocity, x)

        orthVel = sprod(self.computeLambda(Dh1, h, velocity), Dh1).getTranspose()
        parVel = velocity - sprod((Dh1 * velocity)[0][0], Dh1).getTranspose()

        modVel = parVel + orthVel

        return modVel.getTranspose().rows[0]


    def stabiliseVelocity(self, velocity, x):
        """ Returns stabilised velocity given a velocity vector, attractor 
            position target and current position x 
        """  

        position = Matrix.fromList([x]).getTranspose()
        stabilisationConstant = 0.3 * velocity.getNorm()

        badDir = self.target - position
        badDir = sprod(1 / max(badDir.getNorm(), REALMIN), badDir)

        badComponent = velocity.getNorm()
        coefficient =  max(badComponent, stabilisationConstant)

        velocity = velocity + sprod(coefficient - badComponent, badDir)

        return velocity


    def computeLambda(self, Dh1, h, velocity):
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


    def gradH(self, x, steps=None, h=None):
        """ Returns the numerically computed gradient of the modulation function h
            given a vector point x (as a list), a step vector defining the step
            taken in every direction (as lists)
            Parameters:
             steps = list of the steps (dx1, dx2, ..., dxn) for the gradient. You 
                    can use step instead if all dimensions have the same step
                 h = the current h value (can be provided to improve performance)
        """

        dim = len(x)
        step = 1E-5

        if h is None:
            h = self.getH(x)

        if steps is None:
            steps = [step for i in range(dim)]

        # As we want to measure the change in each dimension, we prepare a 
        # different x_final for each dimension
        xf = [x for i in range(dim)]

        for i in range(dim):
            xf[i][i] +=  steps[i]    

        gradH = [(self.getH(xf[i]) - h) / steps[i] for i in range(dim)]

        return gradH


    def getH(self, x):
        """ Returns the modulation function value for a vector point x (as a list) """
        h = self.alphaSum(x) + self.betaSum(x) - self.gammaSum(x) + self.bias
        return h


    def alphaSum(self, x):
        """ Returns the first term of the h sum computation """

        alphaS = sum([self.a[i] * self.ker(x, self.xa.getTranspose()[i]) for i in range(self.xa.n)])

        return alphaS


    def betaSum(self, x):
        """ Returns the second term of the h sum computation """

        betaS = sum([self.b[i] * self.vb[i][j] * self.dx2ker(x, self.xb[i])[j]
                                for i in range(len(self.b)) for j in range(len(x))])

        return betaS


    def gammaSum(self, x):
        """ Returns the third term of the h sum computation """

        gammaS = sum([self.g[i] * self.dx2ker(x, self.target.getTranspose()[0])[i] 
                                                            for i in range(len(x))])

        return gammaS


    def ker(self, x1, x2):
        """ Returns the result of applying a RBF kernel to vectors x1, x2 with 
            a given gamma (default: gamma = 0.5)
        """

        k = exp(- self.kernelG * sum([(x1[i] - x2[i]) ** 2 for i in range(len(x1))]))

        return k


    def dx2ker(self, x1, x2):
        """ Returns the result of applying the derivative of a RBF kernel to 
            vectors x1, x2 with a given gamma (default: gamma = 0.5)
        """

        dk = [2 * self.kernelG * self.ker(x1, x2) * (x1[i] - x2[i]) for i in range(len(x1))]

        return dk


    def loadParameters(self):
        """ Load a single class A-SVM (i.e. an A-SVM with a positive class where 
            everything else is part of the negative class) from files generated 
            with the MATLAB libraries.
            Parameters:
                DS Parameters (loaded as a DS object):
                     Mu: list of matrices (Matrix objects)
                  Sigma: list of matrices (Matrix objects)
               SigmaInv: list of matrices (Matrix objects)
               SigmaDet: list of numbers
                 Priors: list of numbers
                ASVM Parameters:
                targets: list of matrices (Matrix objects)
                a, b, g: alpha/beta/gamma: three lists of numbers
                   bias: a single number
             xa, xb, vb: three matrices (Matrix objects)
        """
        self.DS = DynamicalSystem(self.datapath)
        self.target = loadMatrices(self.datapath + 'Target')[0]

        try:
            alpha = loadMatrices(self.datapath + 'Alpha')
            gamma = loadMatrices(self.datapath + 'Gamma')
            bias = loadMatrices(self.datapath + 'Bias')
            xa = loadMatrices(self.datapath + 'aPoints')
        except IOError:   # files don't exist
            print "ASVM files not found"
            alpha = beta = gamma = xa = xb = vb = bias = Matrix(0,0,init=False)

        self.a = alpha[0].getTranspose().rows[0]
        self.xa = xa[0]

        beta = loadMatrices(self.datapath + 'Beta')
        xb = loadMatrices(self.datapath + 'bPoints')
        vb = loadMatrices(self.datapath + 'bVels')

        if len(beta) == 0:
            self.xb = []
            self.vb = []
            self.b = []
            print 'No beta files found! :S'
        else:   
            self.xb = xb[0].getTranspose()
            self.vb = vb[0].getTranspose()
            self.b = beta[0].getTranspose()[0]

        self.g = gamma[0].getTranspose().rows[0]
        self.bias = bias[0].getTranspose().rows[0][0]


    def loadMatrices(self, globpattern):
        """ Loads from files a list of all matrices in files given a name with a
            wildcard. If a single name is provided, the output is still a list
            with a single element
        """

        matrices = []

        for fname in sorted(glob(globpattern)):
            matrices.append(Matrix.readGrid(fname))

        return matrices



class MultiClassASVM():

    def __init__(self, datapaths):
        if len(datapaths) == 1 and datapaths != '':
            print "Warning: only one datapath provided!"
            print "Are you sure this is a multiclass DS?"
        self.cardinality = len(datapaths)
        self.datapaths = datapaths
        self.ASVM = [SingleClassASVM(self.datapaths[i]) for i in range(self.cardinality)]


    def simulate(self, speech, motion, names, n=10, minI=None, maxI=None):
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
                         n: Number of tries. Default: 10
                  max/minI: optionally, the upper/bottom limits of the positions
                            can be set to test DS defined only in a bounded region
                            as lists of numbers. Both limits should be provided.
                            Otherwise, the system will use the mechanical limits
                            of Nao for both, max and min bounds. 
        """
        for i in range(n):
            speech.say("Going to initial position")
            x = getRandomPosition(motion, names, minI, maxI)
            goToAngles(motion, names, x)
            sleep(2.0)
            angles = readAngles(motion, names)
            speech.say("Starting A.S.V.M")
            sleep(1.0)
            gK = self.runUntilEq(motion, names, 0.25, 0.02)
            speech.say("Equilibrium reached at attractor: " + str(gK))

        speech.say("Simulation finished")  
        
        return None


    def runUntilEq(self, motion, names, dt=0.1, waitTime=0.01):
        """ Runs a SVM motion until the equilibrium has been reached.
            Inputs:
               motion: motion proxy already opened in Nao
                names: list of strings with the names of the joints involved in 
                       the motion
                   dt: time interval used to compute dx = v dt. Default: 0.1
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
        angles = readAngles(motion, names)

        equilibrium = False
        while not equilibrium:
            gK, equilibrium = self.runStep(motion, names, dt)
            sleep(waitTime)

        return gK


    def runStep(self, motion, names, dt=0.07, tol=1):
        """ Runs an ASVM step given its parameters and a motion proxy.
            Inputs:
               motion: motion proxy already opened in Nao
                names: list of strings with the names of the joints involved 
                       in the motion.
                  dt: time interval used to compute dx = v dt. Default: 0.1
                  tol: multiplicative factor to amplify/decrease the tolerances of 
                       the joints. Default: 1
            Outputs:
                   gK: the index of the used attractor in the step
          equilibrium: boolean variable indicating if the step finished in an 
                       equilibrium situation (i.e. velocities under the threshold)
        """
        angles = readAngles(motion, names)
        x = getX(angles, names)
        gk = self.getLargerHIndex(x)
        velocity = self.ASVM[gk].modVelocity(x)
        dx = [dt*v for v in velocity]
        moveAngles(motion, names, dx)
        equilibrium = self.ASVM[gk].DS.equilibriumReached(names, dx, tol)

        return [gk, equilibrium]


    def getLargerHIndex(self, x):
        """ Returns the class index with the larger h value. 
            Input: x as a list of numbers representing the current point vector
            Output: the index of the class with greater h
        """

        h = [self.ASVM[i].getH(x) for i in range(self.cardinality)]

        return h.index(max(h))  

