from ctypes import *
import os

# 需要用到的接口
# MultiAxisInit(&multiAxis,&multiAxisParams);
# 准备这些接口的输入参数

class Py_Point_t(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float)
    ]

class Py_Position_t(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("heading", c_float)
    ]

(PathType_Line, PathType_Bezier) = (0, 1)
(ActionType_Append, ActionType_Immediate) = (0, 1)
BezierOrd = 6

class Py_PathTskParams_t(Structure):
    _fields_ = [ 
        ("pathType", c_int),
        ("actionType", c_int),
        ("CtrlPt", Py_Point_t * BezierOrd),
        ("maxVel", c_float),
        ("maxAcc", c_float),
        ("creepVel", c_float),
        ("creepDistance", c_float),
        ("dir", c_int),
        ("pathId", c_int),
        ("CtrlPtDev", Py_Point_t * (BezierOrd-1)),
        ("CtrlPtDev2", Py_Point_t * (BezierOrd-2)),
        ("PathLength", c_float),
        ("startDirection", c_float),
        ("endDirection", c_float),
        ("startCurvature", c_float),
        ("endCurvature", c_float),
        ("ClosetPose", Py_Position_t)
    ]

class Py_TrajPt_t(Structure):
    _fields_ = [
        ("vel", c_float),
        ("acc", c_float),
        ("omega", c_float),
        ("jerk", c_float),
        ("relativeTime", c_float),
        ("x", c_float),
        ("y", c_float),
        ("theta", c_float),
        ("kappa", c_float),
        ("dkappa", c_float),
        ("accumulate_s", c_float)
    ]

class Modules_DLL:
    def __init__(self):
        # 报错无法找到dll时尝试切换下列添加方法
        self.cdll = cdll.LoadLibrary(os.path.dirname(__file__)+'/libModules.dll')   #在3.8.8版本能运行。在3.9.6不可
        # self.cdll = CDLL(os.path.dirname(__file__)+'\libModules.dll',winmode=0)       #在3.8.8版本，3.9.6都可
        return
    
    def PathCalcParams(self, pathTskParams, curPose):
        func = self.cdll.PathCalcParams

        func.restype = c_int
        func.argtypes = [POINTER(Py_PathTskParams_t), POINTER(Py_Position_t)]

        return func(byref(pathTskParams), byref(curPose))

    def WrapMultiAxisInit(self):
        func = self.cdll.WrapMultiAxisInit

        return func()

    def WrapMultiAxisAddPath(self, pathTskParams):
        func = self.cdll.WrapMultiAxisAddPath

        func.argtypes = [POINTER(Py_PathTskParams_t)]

        return func(byref(pathTskParams))
    
    def WrapGetCurPos(self, setPt, setLinearVel, t):
        func = self.cdll.WrapGetCurPos

        func.argtypes = [POINTER(c_float), POINTER(c_float), POINTER(c_float)]

        return func(byref(setPt), byref(setLinearVel), byref(t))

    def WrapGetPointInPath(self, setPt, setLinearVel, t):
        func = self.cdll.WrapGetPointInPath

        func.restype = Py_TrajPt_t
        func.argtypes = [c_float, c_float, c_float]

        return func(setPt, setLinearVel, t)

    def WrapGetTotalT(self):
        func = self.cdll.WrapGetTotalT

        func.restype = c_int

        return func()


class LibModules:
    def __init__(self):
        self.moduleDll = Modules_DLL()
        self.moduleDll.WrapMultiAxisInit()
        self.pathTskParams = Py_PathTskParams_t()

    def PathCalcParams(self, pathTskParams_dict, curPose_list):
        if "line" == pathTskParams_dict["pathType"]:
            self.pathTskParams.pathType = PathType_Line
            self.pathTskParams.CtrlPt[0].x = pathTskParams_dict["CtrlPt"][0][0]
            self.pathTskParams.CtrlPt[0].y = pathTskParams_dict["CtrlPt"][0][1]
            self.pathTskParams.CtrlPt[5].x = pathTskParams_dict["CtrlPt"][1][0]
            self.pathTskParams.CtrlPt[5].y = pathTskParams_dict["CtrlPt"][1][1]
        else: 
            self.pathTskParams.pathType = PathType_Bezier
            for i in range(BezierOrd):
                self.pathTskParams.CtrlPt[i].x = pathTskParams_dict["CtrlPt"][i][0]
                self.pathTskParams.CtrlPt[i].y = pathTskParams_dict["CtrlPt"][i][1]
        self.pathTskParams.actionType = pathTskParams_dict["actionType"]
        self.pathTskParams.maxVel = pathTskParams_dict["maxVel"]
        self.pathTskParams.maxAcc = pathTskParams_dict["maxAcc"]
        self.pathTskParams.creepVel = pathTskParams_dict["creepVel"]
        self.pathTskParams.creepDistance = pathTskParams_dict["creepDistance"]
        self.pathTskParams.dir = pathTskParams_dict["dir"]
        self.pathTskParams.pathId = pathTskParams_dict["pathId"]

        curPose = Py_Position_t()
        curPose.x = curPose_list[0]
        curPose.y = curPose_list[1]
        curPose.heading = curPose_list[2]

        return self.moduleDll.PathCalcParams(self.pathTskParams, curPose)

    def MultiAxisAddPath(self):
        self.moduleDll.WrapMultiAxisAddPath(self.pathTskParams)

    def GetCurPos(self):
        tmp_setPt = c_float(0.0)
        tmp_setLinearVel = c_float(0.0)
        tmp_t = c_float(0.0)

        self.moduleDll.WrapGetCurPos(tmp_setPt, tmp_setLinearVel, tmp_t)

        return tmp_setPt.value, tmp_setLinearVel.value, tmp_t.value

    def GetPointInPath(self, setPt, setLinearVel, t):
        tmp_setPt = c_float(setPt)
        tmp_setLinearVel = c_float(setLinearVel)
        tmp_t = c_float(t)

        trajPt = Py_TrajPt_t()

        trajPt = self.moduleDll.WrapGetPointInPath(tmp_setPt, tmp_setLinearVel, tmp_t)

        trajPt_dict = {"vel":         trajPt.vel,
                       "acc":         trajPt.acc,
                       "omega":       trajPt.omega,
                       "jerk":        trajPt.jerk,
                       "relativeTime":trajPt.relativeTime,
                       "x":           trajPt.x,
                       "y":           trajPt.y,
                       "theta":       trajPt.theta,
                       "kappa":       trajPt.kappa,
                       "dkappa":      trajPt.dkappa,
                       "accumulate_s":trajPt.accumulate_s}
        return trajPt_dict

    def GetTotalT(self):
        T = self.moduleDll.WrapGetTotalT()
        return T

