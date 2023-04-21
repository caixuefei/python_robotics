import matplotlib.pyplot as plt
import numpy as np
import math

lut = [      [1],           # n=0
            [1,1],          # n=1
           [1,2,1],         # n=2
          [1,3,3,1],        # n=3
         [1,4,6,4,1],       # n=4
        [1,5,10,10,5,1],    # n=5
       [1,6,15,20,15,6,1]]  # n=6
bezier_N = 5

def Tranform(relativePos, pt):
    resPt = [0, 0]
    resPt[0] = math.cos(relativePos[2]) * pt[0] - math.sin(relativePos[2]) * pt[1] + relativePos[0]
    resPt[1] = math.sin(relativePos[2]) * pt[0] + math.cos(relativePos[2]) * pt[1] + relativePos[1]
    return resPt

def DrawAMR(ax, amrPos, width=0.2, length=0.3):
    amrHeadPt_basedAMR = [0.5*length, 0]
    amrBackLeftPt_basedAMR = [-0.5*length, 0.5*width]
    amrBackRightPt_basedAMR = [-0.5*length, -0.5*width]
    amrHeadPt_basedMap = Tranform(amrPos, amrHeadPt_basedAMR)
    amrBackLeftPt_basedMap = Tranform(amrPos, amrBackLeftPt_basedAMR)
    amrBackRightPt_basedMap = Tranform(amrPos, amrBackRightPt_basedAMR)
    DrawPathLine(ax, amrHeadPt_basedMap, amrBackLeftPt_basedMap, '#F97306')
    DrawPathLine(ax, amrBackLeftPt_basedMap, amrBackRightPt_basedMap, '#F97306')
    DrawPathLine(ax, amrBackRightPt_basedMap, amrHeadPt_basedMap, '#F97306')
    
def DrawPathLine(ax, startPt, targetPt, color='g'):
    ax.plot([startPt[0], targetPt[0]], [startPt[1], targetPt[1]], color)

def DrawPathBezier(ax, bezier_CtrlPs, color='y'):
    bezier_sampleCnts = 40
    bezier_x = np.zeros(bezier_sampleCnts+1)
    bezier_y = np.zeros(bezier_sampleCnts+1)
    for ind in range(bezier_sampleCnts+1):
        bezier_t = ind * 1.0 / bezier_sampleCnts
        for i in range(bezier_N+1):
            bezier_x[ind] = bezier_x[ind] + lut[bezier_N][i] * math.pow(1-bezier_t, bezier_N-i) * math.pow(bezier_t, i) * bezier_CtrlPs[i, 0]
            bezier_y[ind] = bezier_y[ind] + lut[bezier_N][i] * math.pow(1-bezier_t, bezier_N-i) * math.pow(bezier_t, i) * bezier_CtrlPs[i, 1]

    ax.plot(bezier_x, bezier_y, color)

def WrapTo2PiDe(angle):
    tmp = angle % (2*math.pi)
    return tmp

def WrapToNegPi2PosPi(angle):
    tmp = angle % (2*math.pi)
    if tmp > math.pi:
        tmp = -2*math.pi + tmp
    return tmp

def GetAngleDiff(angle1, angle2):
    angle1 = WrapTo2PiDe(angle1)
    angle2 = WrapTo2PiDe(angle2)
    dif = angle1 - angle2
    if dif > math.pi:
        dif = -2*math.pi + dif
    elif dif < -1*math.pi:
        dif = 2*math.pi + dif
    return dif