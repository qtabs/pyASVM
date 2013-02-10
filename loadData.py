from glob import glob
from matrix import Matrix


def loadMatrices(globpattern):
    """ Loads from files a list of all matrices in files given a name with a
        wildcard. If a single name is provided, the output is still a list
        with a single element
    """

    matrices = []

    for fname in sorted(glob(globpattern)):
        matrices.append(Matrix.readGrid(fname))

    return matrices



def loadDSParameters(path=''):  
    """ Load all the DS parameters generated form the MATLAB libraries given
        the path of the files. Path must include trailing slash: path/
        Returns the parameters in the format used by the rest of the libraries.
        Outputs:
                 Mu: list of matrices (Matrix objects)
              Sigma: list of matrices (Matrix objects)
           SigmaInv: list of matrices (Matrix objects)
           SigmaDet: list of numbers
             Priors: list of numbers
             xTrans: list of matrices (Matrix objects)
    """

    if path[-1] != '/' and path != '':
        print "LOADING... PATH MUST INCLUDE A TRAILING SLASH!!"

    Mu = loadMatrices(path + 'Mu*')
    Sigma = loadMatrices(path + 'SigmaMat*')
    SigmaInv = loadMatrices(path + 'SigmaInv*')
    SigmaDet = loadMatrices(path + 'SigmaDet*')
    Priors = loadMatrices(path + 'Priors')
    xTrans = loadMatrices(path + 'xTrans')

    return [Mu, Sigma, SigmaInv, 
            SigmaDet[0].getTranspose().rows[0], 
            Priors[0].getTranspose().rows[0],
            xTrans[0]]



def loadMulticlassDSParameters(dataPath):
    """ Load multiclass DS parameters generated form the MATLAB libraries 
        given the path of the files. Returns the parameters as a set of lists 
        in the format used by the rest of the library.
        Inputs:
            dataPath: path to the data files as a list of strings. Each item 
                      in the list should contain the parameters of a single DS
        Outputs:
                 Mu: A list of lists of matrices (Matrix objects)
              Sigma: A list of lists of matrices (Matrix objects)
           SigmaInv: A list of lists of matrices (Matrix objects)
           SigmaDet: A list of lists of numbers
             Priors: A list of lists of numbers
             xTrans: A list of lists of matrices (Matrix objects)
    """
    for i in range(len(dataPath)):
        M, S, SI, SD, P, xT = loadDSParameters(dataPath[i])
        Mu.append(M)
        Sigma.append(S)
        SigmaInv.append(SI)
        SigmaDet.append(SD)
        Priors.append(P)
        xTrans.append(xT)

    return [Mu, Sigma, SigmaInv, SigmaDet, Priors, xTrans]



def loadSingleClassASVM(path=''):
    """ Load a single class A-SVM (i.e. an A-SVM with a positive class where 
        everything else is part of the negative class) from files generated 
        with the MATLAB libraries.
        Inputs:
           dataPath: path to the data files containing the parameters of the
                     single class A-SVM
        Outputs:
                 Mu: list of matrices (Matrix objects)
              Sigma: list of matrices (Matrix objects)
           SigmaInv: list of matrices (Matrix objects)
           SigmaDet: list of numbers
             Priors: list of numbers
            targets: list of matrices (Matrix objects)
            a, b, g: alpha/beta/gamma: three lists of numbers
               bias: a single number
         xa, xb, vb: three matrices (Matrix objects)
    """

    if path[-1] != '/' and path != '':
        print "LOADING... PATH MUST INCLUDE A TRAILING SLASH!!"

    Mu = loadMatrices(path + 'Mu*')
    Sigma = loadMatrices(path + 'SigmaMat*')
    SigmaInv = loadMatrices(path + 'SigmaInv*')
    SigmaDet = loadMatrices(path + 'SigmaDet*')
    Priors = loadMatrices(path + 'Priors')
    Targets = loadMatrices(path + 'Target')

    try:
        alpha = loadMatrices(path + 'Alpha')
        gamma = loadMatrices(path + 'Gamma')
        bias = loadMatrices(path + 'Bias')
        xa = loadMatrices(path + 'aPoints')
    except IOError:   # files don't exist
        print "ASVM files not found"
        alpha = beta = gamma = xa = xb = vb = Matrix(0,0,init=False)

    beta = loadMatrices(path + 'Beta')
    xb = loadMatrices(path + 'bPoints')
    vb = loadMatrices(path + 'bVels')

    if len(beta) == 0:
        xbCool = []
        vbCool = []
        betaCool = []
        print 'No beta files found! :S'
    else:   
        xbCool = xb[0].getTranspose()
        vbCool = vb[0].getTranspose()
        betaCool = beta[0].getTranspose()[0]

    return [Mu, Sigma, SigmaInv,
            SigmaDet[0].getTranspose().rows[0], 
            Priors[0].rows[0],
            Targets[0], 
            alpha[0].getTranspose().rows[0], 
            betaCool, 
            gamma[0].getTranspose().rows[0],
            bias[0].getTranspose().rows[0][0],
            xa[0], xbCool, vbCool]
  


def loadASVMParameters(dataPath):
    """ Load parameters of a multiclass ASVM from files generated with the
        MATLAB libraries.
        Inputs:
           dataPath: path to the data files as a list of strings. Each item 
                     in the list should contain the parameters of a single
                     class A-SVM
        Outputs:
                 Mu: list of lists of matrices (Matrix objects)
              Sigma: list of lists of matrices (Matrix objects)
           SigmaInv: list of lists of matrices (Matrix objects)
           SigmaDet: list of lists of numbers
             Priors: list of lists of numbers
            targets: list of lists of matrices (Matrix objects)
            a, b, g: list of alpha/beta/gamma numerical lists 
               bias: list of numbers
         xa, xb, vb: list of matrices (Matrix objects)
    """
    nAttractors = len(dataPath)

    Mu = [0]*nAttractors
    Sigma = [0]*nAttractors
    SigmaInv = [0]*nAttractors
    SigmaDet = [0]*nAttractors
    Priors = [0]*nAttractors
    target = [0]*nAttractors
    a = [0]*nAttractors
    b = [0]*nAttractors
    g = [0]*nAttractors
    bias = [0]*nAttractors
    xa = [0]*nAttractors
    xb = [0]*nAttractors
    vb = [0]*nAttractors

    for i in range(nAttractors):
        Mu[i], Sigma[i], SigmaInv[i], SigmaDet[i], Priors[i], target[i], a[i], b[i], g[i], bias[i], xa[i], xb[i], vb[i] = loadSingleClassASVM(dataPath[i])

    return [Mu, Sigma, SigmaInv, SigmaDet, Priors, target, a, b, g, bias, xa, xb, vb]
