# -*- coding: utf-8 -*-
"""
Created on Thu Oct 12 09:38:57 2017

@author: Thomas
"""

import random
import math
import numpy as np

"""Ray Vectors"""

theta = random.uniform(0,(2*math.pi))
    
phi = random.uniform(0,(2*math.pi))

def RayVectors(i*theta,i*phi):
    vector=np.array([(math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta))])
    return vector


print(RayVectors(theta,phi))