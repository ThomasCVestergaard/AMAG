import random
import math
import rhinoscriptsyntax as rs
import Rhino as rc
    
Geo=rc.Geometry.Brep.Duplicate(G)

#Ray Vector Points
def RayPoints(theta,phi):
    point=rs.AddPoint(math.sin(theta)*math.cos(phi), math.sin(theta)*math.sin(phi), math.cos(theta))
    return point
    
    
#Ray Vector Origo Points
def OrigoPoints(x,y,z):
    point=rs.AddPoint(x,y,z)
    return point
    
    
#Ray Vectors between Ray Vector Points and Origo
def RayVectors(RayPoints,OrigoPoints):
    Vector=rs.VectorCreate(RayPoints,OrigoPoints)
    return Vector
    
    
#RayVectors
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


#Testing weither source point is contained within Geometry
CS = Geo.IsPointInside(SP,0.01,True)

#Number of Surfaces in Geometry
F=Geo.Surfaces.Count

#Extracting Surfaces from Geometry

Surfaces=[]
def Surf(f):
    surface=rs.ExtractSurface(G,f,copy=True)
    return surface[0]
    
print range (int(F))
for f in range (F):
    
    Surfaces.append(Surf(f))

print Surfaces

#Extracting Surface Points from Geometry Surfaces
SurfPoints=[]
def SurfacePoints(f):
    point=rs.SurfaceAreaCentroid(Surfaces[f])
    return point[0]

print range (int(F))
for f in range (F):
    
    SurfPoints.append(SurfacePoints(f))
    

print SurfPoints

#Surface Normal Vectors

Param=[]
def UV_Param(f):
    point=rs.SurfaceClosestPoint(Surfaces[f],SurfPoints[f])
    return point
    
print range (int(F))
for f in range (F):
    
    Param.append(UV_Param(f))
    
SurfNormal=[]
def SurfaceNormal(f):
    vector=rs.VectorReverse(rs.SurfaceNormal(Surfaces[f],param))
    return vector
    
print range (int(F))
for f in range (F):
    
    SurfNormal.append(SurfaceNormal(f))
    
print SurfNormal
