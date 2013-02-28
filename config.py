""" Configuration file for using the libraries. Use the globals to 
    introduce your specific system Configuration:
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
CONSOLE_TOLERANCES = {"A": 0.05,
                      "B": 0.05,
                      "F": 0.05, 
                      "D": 0.05,
                      "E": 0.05}

NAO_TOLERANCES = {"HeadPitch": 0.002,
                  "RElbowRoll": 0.007 ,
                  "RHand": 0.0003, 
                  "RShoulderPitch": 0.012,
                  "RElbowYaw": 0.007}

NAO_PORT = 9559 
NAO_HOST = "naomi.local"
#NAO_HOST = "localhost"
