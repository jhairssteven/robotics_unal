from cmath import sqrt
import numpy as np
import math

L1 = 145
L2 = 106.3
L3 = 106.3
L4 = 89.7

def getJointValues(pose, degrees=True):
    try:
        q = np.double([[0, 0, 0, 0], [0, 0, 0, 0]])
        desiredPos = pose[0:4,3] 
        desiredOrientation = pose[0:3,0:3]
        approach = pose[0:4,0]

        #%Cálculo de la primera articulación en radianes
        q[0,0] = math.atan2(desiredPos[1], desiredPos[0]) #Codo abajo
        q[1,0] = q[0,0] 

        #Desacople de muñeca:
        #Cálculo de la posición de la muñeca W
        wristPos = desiredPos -  L4*approach
        H10 = GetH10(q[0,0])
        wristPos_1 = H10*wristPos

        x = float(wristPos_1[0])
        y = float(wristPos_1[1])


        cos_theta3 = (x**2+y**2-L2**2-L3**2)/(2*L2*L3)
        sin_theta3 = np.real(sqrt(1-cos_theta3**2))
        theta3 = math.acos(cos_theta3)
        q[0,2] = -theta3 + (math.pi/2) #Codo abajo
        q[1,2] = theta3 + (math.pi/2)  #Codo arriba

        #Segunda articulación en radianes sin offset
        k1 = L2+L3*cos_theta3
        k2 = L3*math.sin(theta3)

        q[0,1] = math.atan2(y,x) + math.atan2(k2, k1) #Codo abajo
        q[1,1] = math.atan2(y,x) - math.atan2(k2, k1) #Codo arriba

        #Teniendo en cuenta offset en la segunda articulación
        q[0,1] = q[0,1] - math.pi/2
        q[1,1] = q[1,1] - math.pi/2;   

        #Obtención de valor de cuarta articulación
        approach_1 = H10[0:3,0:3]*approach[0:3,0]
        phi = math.atan2(approach_1[1],approach_1[0])
        q[0,3] = phi - q[0,1] -q[0,2]
        q[1,3] = phi - q[1,1] -q[1,2]
        
        if degrees:
            return np.degrees(q)
        return q
    except ValueError:
        print("Unreachable")
        return np.double([[0, 0, 0, 0], [0, 0, 0, 0]])
    
def GetH10(theta1):
    cos = math.cos(theta1)
    sin = math.sin(theta1)
    H10 = np.matrix([[cos,sin,0,0],
                        [0,0,1,-L1],
                        [sin,-cos,0,0] ,
                        [0,0,0,1]])
    return H10

def rad2deg(q):
    return q*180 / math.pi
