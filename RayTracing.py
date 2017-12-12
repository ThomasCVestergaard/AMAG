import random
import math
import rhinoscriptsyntax as rs
import Rhino as rc
import clr
clr.LoadTypeLibrary

Geo=rc.Geometry.Brep.Duplicate(G)
Origo=rs.AddPoint(0,0,0)
V = rs.SurfaceVolume(Geo)
    

#Testing wether source point is contained within Geometry
SPC = Geo.IsPointInside(SP,0.01,True)


#Testing wether receiver point is contained within Geometry
RPC = Geo.IsPointInside(RP,0.01,True)


#Number of Surfaces in Geometry
F = Geo.Surfaces.Count


#Extracting Surfaces from Geometry
Surfaces = []
def Surf(f):
    surface = rs.ExtractSurface(G,f,copy=True)
    return surface[0]
for f in range (F):
    Surfaces.append(Surf(f))


#Extracting Surface Points from Geometry Surfaces
SurfPoints = []
def SurfacePoints(f):
    point = rs.SurfaceAreaCentroid(Surfaces[f])
    return point[0]
for f in range (F):
    SurfPoints.append(SurfacePoints(f))


#Surface Normal Vectors
Param = []
def UV_Param(f):
    point = rs.SurfaceClosestPoint(Surfaces[f],SurfPoints[f])
    return point[0]
for f in range (F):
    Param.append(UV_Param(f))
    
SurfNormal = []
def SurfaceNormal(f):
    vector = rs.VectorReverse(rs.SurfaceNormal(Surfaces[f],Param))
    return vector
for f in range (F):
    SurfNormal.append(SurfaceNormal(f))
    
    
#Ray Vector Points
def RayPoints(theta,phi):
    point = rs.AddPoint(math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta))
    return point
#Ray Vectors between Ray Vector Points and Origo
def RayVectors(RayPoints,OrigoPoints):
    Vector = rs.VectorCreate(RayPoints,Origo)
    return Vector
    
#k=math.log10(V[0])
#r=(k*rs.Distance(SP,RP)*math.sqrt(4/n))/(10**3)
r = 250

Messages=[]
Alpha = []
for i in range (F):
    Alpha.append(alpha)
    
if Run == True:
    if SPC == True:
        if RPC == True:
            
            
            
            Energy = []
            DT = []
            for i in range (n):
                
                #RayVectors
                RayVec = []
                for i in range (n):
                    theta = random.uniform(0,(2*math.pi))  
                    phi = random.uniform(0,(2*math.pi))
                    RayVec.append(RayVectors(RayPoints(theta,phi),Origo))
                    
                ReflectionPoints = [SP]
                RefVec = [RayVec[0]]
                Distances = []
                MinDistances = []
                Rays = []
                RayData = []
                RayData.append(SP)
                RayData.append(RayVec[0])
                TravelLength = 0
                E = 1
                while E > 1*10**-6:
                    #Vector From Source to Geometry Surfaces
                    LenVec = []
                    def LengthVectors(f):
                        vector = rs.VectorCreate(SurfPoints[f],RayData[0])
                        return vector
                    for f in range (F):
                        LenVec.append(LengthVectors(f))
                    
                    Dist = []
                    #Distance from Source Point to Geometry Surfaces
                    def Distance(f):
                        length = (rs.VectorDotProduct(SurfNormal[f],LenVec[f]))/(rs.VectorDotProduct(RayData[1],SurfNormal[f]))
                        return length
                    for f in range (F):
                        Dist.append(Distance(f))
                    
                        Distances.append(Distance(f))
                
                    #Minimum distance
                    DistMin = min(n for n in Dist if n > 1*10**-9)
                    index = [i for i,x in enumerate(Dist) if x == DistMin]
                    
                    #Reflection Point
                    ReflectionPoint = RayData[0]+RayData[1]*DistMin
                    
                    ReflectionPoints.append(ReflectionPoint)
                    ReflectPoints=ReflectionPoints[:-1]
                    
                    Rays.append(rs.VectorCreate(ReflectionPoint,RayData[0]))
                    
                    w=rs.VectorCreate(ReflectionPoint,RP)
                    A = rs.VectorDotProduct(RayData[1],w)
                    B = rs.VectorLength(w)**2
                    
                    if rs.VectorDotProduct(RayData[1],w) > 0 and (A**2-B+r**2) >= 0 and DistMin >= -A:
                        Line=rs.AddLine(RayData[0],ReflectionPoint)
                        LineSphereInt=rs.LineSphereIntersection(Line,RP,r)
                        E = math.exp(-0.0009*10**-3*rs.Distance(RayData[0],LineSphereInt[0]))*E
                        TravelLength = TravelLength+rs.Distance(RayData[0],LineSphereInt[0])
                        Energy.append(E)
                        DecayTime = (TravelLength/(10**3))/343
                        DT.append(DecayTime)
                        
                        break
                    else:
                        
                        #Reflected Vector
                        ReflectionVec = rs.VectorSubtract(RayData[1],(2*SurfNormal[index[0]]*rs.VectorDotProduct(SurfNormal[index[0]],RayData[1])))
                        
                        RefVec.append(ReflectionVec)
                        
                        RayData=(ReflectionPoint,ReflectionVec)
                        
                        E = math.exp(-0.0009*10**-3*DistMin)*(1-Alpha[index[0]])*E
                        
                        TravelLength = TravelLength+DistMin
                        
                        DecayTime = (TravelLength/(10**3))/343
                        
                        if E < 1*10**-6:
                            DT.append(DecayTime)
                            Energy.append(E)
                        
                        
        else:
            M1 = ("Receiver Point is outside Geometry")
            Messages.append(M1)
        
    else:
        M2 = ("Source Point is outside Geometry")
        Messages.append(M2)
RT = [0]
SPL = [0]
for i in range (n):
    SPL.append(10*math.log10(Energy[i]))
for i in range (n):
    RT.append(DT[i])

RT1 = RT
SPL1 = SPL
RTSPL = zip(RT1,SPL1)

RTSPL.sort()
SPL_Sorted = [SPL1 for RT1, SPL1 in RTSPL]

RT1 = [list(RT1) for RT1 in zip(*RTSPL)]
SPL = SPL_Sorted
RT = RT1[0]
print RT
print SPL