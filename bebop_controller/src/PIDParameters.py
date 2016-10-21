
from copy import deepcopy
import numpy

class PIDParameters(object):

    def __init__(self):
        self.K = 0.0
        self.Ti = 0.0
        self.Tr = 0.0
        self.Td = 0.0
        self.N = 0.0
        self.Beta = 0.0
        self.H = 0.0
        self.integratorOn = False
