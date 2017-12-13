"""
RayTrace - Ray Tracing algorithm for 1000 Hz octave band using same alpha values for all surfaces in geomtry

Created by Thomas Charles Vestergaard
MSc. Student in Architectural Engineering
Danish Technical University
Sep-Dec 2017
 
    Args:
        G: Input Geometry as Brep here
        n: Input number of simulated rays here
        SP: Input Source Point position here
        RP: Input Receiver Point position here
        alpha: Input aborptions values for the geometry here (Same alpha value for all surfaces)
        Run: Input Boolean Toggle here (component runs with Toggle = True)
        
    Returns:
        Messages: Error messages - Connect panel
        ReverberationTime: T30 reverberation time for the 1000 Hz octave band - Connect panel
        
"""

#Import of classes
import random
import math
import rhinoscriptsyntax as rs
import Rhino as rc

#Defining the Geometry, Origo point and calculating the volume
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
    
    
#Ray Vector Points - Function which creates points uniformly random around origo by projection of unit vector.
def RayPoints(theta,phi):
    point = rs.AddPoint(math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta))
    return point
#Ray Vectors between Ray Vector Points and Origo - Creates the initial vectors with basepoint in Origo. Later to be moved to the Source Point.
def RayVectors(RayPoints,Origo):
    Vector = rs.VectorCreate(RayPoints,Origo)
    return Vector
    
#k is used to find the receiver sphere size. It was found that using the receiver sphere radius found from r, the receiver sphere became too small. Instead the diamter of the sphere was set to a fixed value of 500 mm.
#k=math.log10(V[0])
#r=(k*rs.Distance(SP,RP)*math.sqrt(4/n))/(10**3)
r = 250

#Creating list of absorption values with the same length as the list of surfaces. 
Alpha = []
for i in range (F):
    Alpha.append(alpha)
    
#Initial conditions set up to ensure that the script only runs for: Boolean Toggle = True and when both the source and receiver point are inside the geometry. If the conditions are not met, error messages will be displayed.
Messages=[]
if Run == True:
    if SPC == True:
        if RPC == True:
            
            #Empty lists set up for the Energy and decaytime
            Energy = []
            DT = []
            
            #For loop running through the chosen number of rays - Input "n"
            for i in range (n):
                
                #RayVectors - Random values are generated to run the function creating the initial ray vectors.
                RayVec = []
                for i in range (n):
                    theta = random.uniform(0,(2*math.pi))  
                    phi = random.uniform(0,(2*math.pi))
                    RayVec.append(RayVectors(RayPoints(theta,phi),Origo))
                    
                """A collection of empty lists are created. The source point and initial ray vector are used as start conditions.
                The most important list here is the RayData list. 
                This lists starts with the source point and initial ray vector. 
                These values are replaced with the reflection point and reflection vector as the loops run.
                Meaning the reflection point and reflection vector are used as the new source point and initial ray vector."""
                ReflectionPoints = [SP]
                RefVec = [RayVec[0]]
                Distances = []
                MinDistances = []
                Rays = []
                RayData = []
                RayData.append(SP)
                RayData.append(RayVec[0])
                
                #The initial travel length and energy for each ray is set to 0 and 1 respectively.
                TravelLength = 0
                E = 1
                
                #While loop set up with a threshhold of 10^(-6) which corresponds to a sound pressure level reduction of 60 dB.
                while E > 1*10**-6:
                    
                    #Vector From Source Point to Geometry Surfaces
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
                
                    #Minimum distance and the index of the hit surface
                    DistMin = min(n for n in Dist if n > 1*10**-9)
                    index = [i for i,x in enumerate(Dist) if x == DistMin]
                    
                    #Reflection Point
                    ReflectionPoint = RayData[0]+RayData[1]*DistMin
                    
                    #All the reflection points are collected in a list for visual display purposes.
                    ReflectionPoints.append(ReflectionPoint)
                    ReflectPoints=ReflectionPoints[:-1]
                    
                    #All rays are collected in a list for visual display purposes.
                    Rays.append(rs.VectorCreate(ReflectionPoint,RayData[0]))
                    
                    #Ray/receiver sphere intersection parameters are created
                    w=rs.VectorCreate(ReflectionPoint,RP)
                    A = rs.VectorDotProduct(RayData[1],w)
                    B = rs.VectorLength(w)**2
                    
                    """If statement checking whether the ray hits the receiver sphere. 
                    If it does, the energy and decay time are appended to their respective lists 
                    and the for loop moves on to the next ray"""
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
                        
                        """If the ray does not hit the receiver sphere, the reflection vector is calculated,
                        the RayData list is updated, energy reduction from air attenuation and surface absorption
                        is applied and appended, the decay time is calculated and if it is below the threshhold it is appended"""
                        
                        #Reflected Vector
                        ReflectionVec = rs.VectorSubtract(RayData[1],(2*SurfNormal[index[0]]*rs.VectorDotProduct(SurfNormal[index[0]],RayData[1])))
                        RefVec.append(ReflectionVec)
                        
                        #Ray Data is updated
                        RayData=(ReflectionPoint,ReflectionVec)
                        
                        #Energy reductions are applied
                        E = math.exp(-0.0009*10**-3*DistMin)*(1-Alpha[index[0]])*E
                        
                        TravelLength = TravelLength+DistMin
                        DecayTime = (TravelLength/(10**3))/343
                        
                        
                        if E < 1*10**-6:
                            DT.append(DecayTime)
                            Energy.append(E)
                        
                        
        #Error messages 
        else:
            M1 = ("Receiver Point is outside Geometry")
            Messages.append(M1)
        
    else:
        M2 = ("Source Point is outside Geometry")
        Messages.append(M2)
else:
    M3 = ("Results are not updated")
    Messages.append(M3)
    
#The energy reductions are converted to dB and appended to new list.
#A new list of decay times is created for processing purposes.
RT = []
SPL = []
for i in range (n):
    SPL.append(10*math.log10(Energy[i]))
for i in range (n):
    RT.append(DT[i])

#The lists of dB reductions and decay times are zipped and sorted with respect to the decay times
RT1 = RT
SPL1 = SPL
RTSPL = zip(RT1,SPL1)

RTSPL.sort()
SPL_Sorted = [SPL1 for RT1, SPL1 in RTSPL]

RT1 = [list(RT1) for RT1 in zip(*RTSPL)]


SPL = SPL_Sorted
RT = RT1[0]

#A list of time steps of 3 ms is created in order to average the simulation results into time step segments.
TimeSteps = range(int(math.floor(min(RT))),int(math.ceil(max(RT)*1000))+3,3)

RT_1000 = []
Indexes = []

#Function converting the decay times to integers
def RT1000(t):
    Item = RT[t]*1000
    return Item
    
#For each recorded decay time it is checked in which time step it is. 
#The index of the decay time and the center value for the respective time segment is appended.
for t in range (len(RT)):
    RT_1000.append(RT1000(t))
    i = 0
    while i < len(TimeSteps):
        if  TimeSteps[i] <= RT_1000[t] <= TimeSteps[1+i]:
            Indx = ([q for q,x in enumerate(RT_1000) if x == RT_1000[t]])
            Indexes.append([Indx[0],(TimeSteps[i]+TimeSteps[1+i])/2])
            break
        else:
            i = i+1
            
#Empty lists for the purpose of collecting the indexes of decay times and corresponding dB reductions which are in the same time segment.
CollectiveSplIndexes = []
CollectiveDecaytimes = []

"""While loop going through each time step one by one.
Inside that while loop another while loop checks all the decay times to see any of them fall within the specific time step.
If it does, then the index of the dB reduction and the decay times are appended to two seperate lists.
This yields two lists each time a decay time is within a time step. These lists are then collected in two other lists,
giving one list with lists of dB reductions within a timestep and another list with lists of decay times within a time step."""
q = 0
while q < len(TimeSteps):
    SplIndexes = []
    Decaytimes = []
    p = 0
    while p < len(Indexes):
        if TimeSteps[q] <= Indexes[p][1] <= TimeSteps[1+q]:
            SplIndexes.append(Indexes[p][0])
            Decaytimes.append(Indexes[p][1])
            break
        else:
            p = p+1
            
    CollectiveSplIndexes.append(SplIndexes)
    CollectiveDecaytimes.append(Decaytimes)
    q = q+1

#All dB reductions that fall within the same time step are then summed and averaged
SPLaverage = []
for i in range (len(CollectiveSplIndexes)):
    if CollectiveSplIndexes[i] == []:
        continue
    else:
        a = 0
        w = 0
        while w < (len(CollectiveSplIndexes[i])):
            a = a + (SPL[CollectiveSplIndexes[i][w]])
            w = w + 1
    SPLaverage.append(a/len(CollectiveSplIndexes[i]))

#The decay times are converted back into floats
DecayTimes = []
for i in range (len(CollectiveDecaytimes)):
    if CollectiveDecaytimes[i] == []:
        continue
    else:
        b = CollectiveDecaytimes[i][0]*10**(-3)
    DecayTimes.append(b)

DecayValues = [0] + SPLaverage
DecayTimes = [0] + DecayTimes

#SPL values just below and above -5 dB
SPLlowmin = min(n for n in DecayValues if n > -5)
lowminindex = [i for i,x in enumerate(DecayValues) if x == SPLlowmin]

SPLlowmax = max(n for n in DecayValues if n < -5)
lowmaxindex = [i for i,x in enumerate(DecayValues) if x == SPLlowmax]

#Interpolation for the -5 dB point
FivedBDecay = (((-5-SPLlowmin)*(DecayTimes[lowmaxindex[0]]-DecayTimes[lowminindex[0]]))/(SPLlowmax-SPLlowmin))+DecayTimes[lowminindex[0]]

#SPL values just below and above -35 dB
SPLhighmin = min(n for n in DecayValues if n > -35)
highminindex = [i for i,x in enumerate(DecayValues) if x == SPLhighmin]

SPLhighmax = max(n for n in DecayValues if n < -35)
highmaxindex = [i for i,x in enumerate(DecayValues) if x == SPLhighmax]

#Interpolation for the -35 dB point
ThirtyfivedBDecay = (((-35-SPLhighmin)*(DecayTimes[highmaxindex[0]]-DecayTimes[highminindex[0]]))/(SPLhighmax-SPLhighmin))+DecayTimes[highminindex[0]]

"""The T30 reveberation time is then calculated as the difference between the interpolated
-5 dB decay time and the interpolated -35 dB decay time, multiplied by 2"""
ReverberationTime = (ThirtyfivedBDecay - FivedBDecay)*2