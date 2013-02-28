""" Configuration file for using the libraries. Use the globals to 
    introduce your specific system Configuration:
            NAO: The host of the Nao platform to be used. If the 
                 package is intended to be executed locally in
                 the robot, use "localhost"
           PORT: The port in which the proxies will be open. In Nao
                 this port is, in general, 9559
          NAMES: Names of the joints which behaviour is defined by 
                 the dynamical system, as a list of strings
       DATAPATH: Path to the data folder. Please, be aware that in 
                 the case of execution from Nao full path 
                 specification is recommended
            TOL: Tolerance (i.e. threshold of the movement of the 
                 joints to consider the joints as not moving) of 
                 the joints as a dictionary. More joints that those
                 described in NAMES can be specified
""" 



NAMES = ["HeadPitch", "RShoulderPitch", "RElbowRoll", "RElbowYaw"]

DATAPATH = "../Data"

