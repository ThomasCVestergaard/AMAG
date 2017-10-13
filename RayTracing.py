# -*- coding: utf-8 -*-
"""
Created on Thu Oct 12 09:38:57 2017

@author: Thomas
"""

import random
import math
import numpy as np

"""Ray Vectors"""

def RayVectors(theta,phi):
        vector=np.array([(math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta))])
        return vector
    
RanVec=[]

for i in range (10):
    
    theta = random.uniform(0,(2*math.pi))
    
    phi = random.uniform(0,(2*math.pi))
    RanVec.append(RayVectors(theta,phi))

    


print(RanVec)