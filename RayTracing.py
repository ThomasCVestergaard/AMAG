import random
import math
import rhinoscriptsyntax as rs
    
"""Source Point"""
SourcePoint=rs.AddPoint(SP)

SPnt=[]
SPnt.append(SourcePoint)
print SPnt

def SourcePoint(SP):
    point=rs.AddPoint(SP)
    return point
    
"""Ray Vector Points"""
def RayPoints(theta,phi):
    point=rs.AddPoint(math.sin(theta)*math.cos(phi), math.sin(theta)*math.sin(phi), math.cos(theta))
    return point
    
"""Ray Vector Origo Points"""
def OrigoPoints(x,y,z):
    point=rs.AddPoint(x,y,z)
    return point
    
"""Ray Vectors between Ray Vector Points and Origo"""
def RayVectors(RayPoints,OrigoPoints):
    Vector=rs.VectorCreate(RayPoints,OrigoPoints)
    return Vector
    
RayVec=[]
print range(int(n))
for i in range (n):
    
    theta = random.uniform(0,(2*math.pi))  
    phi = random.uniform(0,(2*math.pi))
    
    x = 0
    y = 0
    z = 0
    
    RayVec.append(RayVectors(RayPoints(theta,phi),OrigoPoints(x,y,z)))

print RayVec