import random
import math
import rhinoscriptsyntax as rs
import Rhino as rc
    
Geo=rc.Geometry.Brep.Duplicate(G)
Origo=rs.AddPoint(0,0,0)
    
#Testing wether source point is contained within Geometry
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

#Vector From Source to Geometry planes
LenVec=[]
def LengthVectors(f):
    vector=rs.VectorCreate(SurfPoints[f],SP)
    return vector
print range (int(F))
for f in range (F):
    LenVec.append(LengthVectors(f))
print LenVec

#Ray Vector Points
def RayPoints(theta,phi):
    point=rs.AddPoint(math.sin(theta)*math.cos(phi), math.sin(theta)*math.sin(phi), math.cos(theta))
    return point

#Ray Vectors between Ray Vector Points and Origo
def RayVectors(RayPoints,OrigoPoints):
    Vector=rs.VectorCreate(RayPoints,Origo)
    return Vector
    
    
#RayVectors
RayVec=[]
print range(int(n))
for i in range (n):
    theta = random.uniform(0,(2*math.pi))  
    phi = random.uniform(0,(2*math.pi))
    RayVec.append(RayVectors(RayPoints(theta,phi),OrigoPoints(x,y,z)))
print RayVec


#Distance from Source Point to Geometry Surfaces
Dist=[]
def Distance(f,i):
    length=rs.VectorDotProduct(SurfNormal[f],LenVec[f])/rs.VectorDotProduct(RayVec[i],SurfNormal[f])
    return length
print range (int(i))
for i in range (n):
    print range (int(F))
    for f in range (F):
    
        Dist.append(Distance(f,i))
    
#Minimum distance
DistMin=min([n for n in Dist  if n>0])

index=[i for i,x in enumerate(Dist) if x == DistMin]

#Reflection Point

ReflectionPoint=[]
def RefPoint(i):
    Point=SP+RayVec[i]*DistMin
    return Point


for i in range (1):
    ReflectionPoint.append(RefPoint(i))
print ReflectionPoint

InitVec=rs.VectorCreate(ReflectionPoint[0],SP)

#Reflected Vector
ReflectionVec=RayVec[0]-2*(rs.VectorDotProduct(SurfNormal[index[0]],RayVec[0]))*SurfNormal[index[0]]