
# coding=utf-8
#启动文件
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
import math
from libModules import LibModules
import struct
from utilityPG import *

#路径存储
#1)直线类型 "line"
#  字典key:  type, points(2*2), targetPos(3), vel, acc, pathID
#2)贝塞尔类型 "bezier"
#  字典key:  type, points(6*2), targetPos(3), vel, acc, pathID
#3)轨迹点类型 "trajPts"
#  字典key:  type, targetPos(3), trajPtSize, points(字典list)
#  points的元素是一个字典，字典key: vel, acc, omega, jerk, relativeTime, x, y, theta, kappa, dkappa, accumulate_s

# 注：
# 1. 这个类中涉及到两个坐标系：
#    1）路径最终运行的地图坐标系，记作cur, 或者 map
#    2）最开始生成路径的参考坐标系，记作ref

class PathGen():
    # 参数x0,y0,theta_0是两个坐标系的相对位姿，因为小车运行在平面上，所以这里的相对位姿用x,y,和偏航角theta来表示
    # 参数trajPtSampleT 表示生成轨迹点时，每隔多少ms采样一次，trajPtSampleT需要是10ms的倍数
    def __init__(self,x_0 = 0.0, y_0 = 0.0, theta_0 = 0.0, trajPtSampleT = 50):
        self.paths=[] #存储路径
        self.theta_0 = theta_0 / 180 * math.pi
        self.theta_0 = WrapToNegPi2PosPi(self.theta_0)
        self.x_0 = x_0
        self.y_0 = y_0  
        self._cal_T_to_RefCoorSys()                  # 计算坐标系变换矩阵，T_cur2ref
        self._cal_T_from_RefCoorSys()                # 计算坐标系变换矩阵，T_ref2cur
        self.colors = ['#15B01A', '#FFFF14', '#9A0EEA', '#00FFFF']
        self.startPos = [0, 0, 0]                   #[m, m, rad], 起点小车的位姿
        self.lastPos = [0, 0, 0]                    #[m, m, rad]，上一条路径终点的小车位姿
        self.libModules = LibModules()
        self.trajPtSizeMax = 36
        tmp = round(trajPtSampleT / 10)
        self.trajPtTimeDeltaCnt = tmp                 # 轨迹点的采样间隔，module模块中的默认采样是10ms，这里生成轨迹点的实际采样间隔是：0.01 * self.trajPtTimeDeltaCnt
        self.pathTypeLine = "line"
        self.pathTypeBezier = "bezier"
        self.pathTypeTrajPts = "trajPts"
    def Clear(self):
        self.paths=[]
        self.theta_0 = 0.
        self.x_0 = 0.
        self.y_0 = 0.
        self._cal_T_to_RefCoorSys()
        self._cal_T_from_RefCoorSys()

    # 接口：设置两个坐标系的相对位姿。
    def SetRelativePos(self, x_0, y_0, theta_0):
        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0 / 180 * math.pi
        self.theta_0 = WrapToNegPi2PosPi(self.theta_0)
        self._cal_T_to_RefCoorSys()
        self._cal_T_from_RefCoorSys()
    # 由theta角，获得两个平面坐标系的旋转矩阵
    def _get_R_2D(self, theta):
        R = np.zeros((2,2))
        R[0][0] =  math.cos(theta)
        R[0][1] =  -1.0 * math.sin(theta)
        R[1][0] = math.sin(theta)
        R[1][1] = math.cos(theta)
        return R
    # 接口：根据小车在两个坐标系下的位姿（[x,y,theta(deg)]）,获得两个坐标系的相对位姿（[x0,y0,theta0]）
    def CalRelativePos(self, pos_RefCoorSys, pos_CurCoorSys):
        theta_1 = pos_RefCoorSys[2] / 180 * math.pi
        theta_2 = pos_CurCoorSys[2] / 180 * math.pi
        self.theta_0 = theta_1 - theta_2
        self.theta_0 = WrapToNegPi2PosPi(self.theta_0)
        R1 = self._get_R_2D(theta_1)
        R2 = self._get_R_2D(theta_2)
        t1 = np.array([[pos_RefCoorSys[0]],
                       [pos_RefCoorSys[1]]])
        t2 = np.array([[pos_CurCoorSys[0]],
                       [pos_CurCoorSys[1]]])
        t0 = -1.0*np.dot(np.dot(R1, R2.T), t2) + t1
        self.x_0 = t0[0]
        self.y_0 = t0[1]
        self._cal_T_to_RefCoorSys()
        self._cal_T_from_RefCoorSys()
    # 用3维量表示的相对位姿（[x0,y0,theta0]），计算坐标系变换矩阵（T_cur2ref）
    def _cal_T_to_RefCoorSys(self):
        self.T_to_RefCoorSys = np.zeros((3,3))
        self.T_to_RefCoorSys[0][0] = math.cos(self.theta_0)
        self.T_to_RefCoorSys[0][1] = -1.0 * math.sin(self.theta_0)
        self.T_to_RefCoorSys[0][2] = self.x_0
        self.T_to_RefCoorSys[1][0] = math.sin(self.theta_0)
        self.T_to_RefCoorSys[1][1] = math.cos(self.theta_0)
        self.T_to_RefCoorSys[1][2] = self.y_0
        self.T_to_RefCoorSys[2][0] = 0
        self.T_to_RefCoorSys[2][1] = 0
        self.T_to_RefCoorSys[2][2] = 1
    # 用3维量表示的相对位姿（[x0,y0,theta0]），计算坐标系变换矩阵（T_ref2cur）
    def _cal_T_from_RefCoorSys(self):
        self.T_from_RefCoorSys = np.zeros((3,3))
        self.T_from_RefCoorSys[0][0] = math.cos(self.theta_0)
        self.T_from_RefCoorSys[0][1] = math.sin(self.theta_0)
        self.T_from_RefCoorSys[0][2] = -1.0 * (math.cos(self.theta_0) * self.x_0 + math.sin(self.theta_0) * self.y_0)
        self.T_from_RefCoorSys[1][0] = -1.0 * math.sin(self.theta_0)
        self.T_from_RefCoorSys[1][1] = math.cos(self.theta_0)
        self.T_from_RefCoorSys[1][2] = math.sin(self.theta_0) * self.x_0 - math.cos(self.theta_0) * self.y_0
        self.T_from_RefCoorSys[2][0] = 0
        self.T_from_RefCoorSys[2][1] = 0
        self.T_from_RefCoorSys[2][2] = 1
    # 把在ref坐标系下的位姿pos，转换到cur坐标系下的表示
    def _transformOnePos(self, pos):
        pos[2] = pos[2] - self.theta_0
        pos[2] = WrapToNegPi2PosPi(pos[2])
        tmp_translation = np.array([[pos[0]],
                                    [pos[1]],
                                      [1]   ])
        tmp_translation = np.dot(self.T_from_RefCoorSys, tmp_translation)
        pos[0] = tmp_translation[0][0]
        pos[1] = tmp_translation[1][0]
    # 把ref坐标系下的一个点point，转换到cur坐标系下的表示
    def _transformOnePoint(self, point):
        tmp_translation = np.array([[point[0]],
                                    [point[1]],
                                      [1]   ])
        tmp_translation = np.dot(self.T_from_RefCoorSys, tmp_translation)
        point[0] = tmp_translation[0][0]
        point[1] = tmp_translation[1][0]
    
    # 根据参考位姿lastPose, 和相对位姿relative_pos，计算目标位姿
    def _poseFrom(self, lastPose, relative_pos): #relative_pos[m,m,deg]
        relative_pos[2] = relative_pos[2] * math.pi / 180
        newpose = [0, 0, 0]
        newpose[0] = math.cos(lastPose[2]) * relative_pos[0] - math.sin(lastPose[2]) * relative_pos[1] + lastPose[0]
        newpose[1] = math.sin(lastPose[2]) * relative_pos[0] + math.cos(lastPose[2]) * relative_pos[1] + lastPose[1]
        newpose[2] = lastPose[2] + relative_pos[2]
        newpose[2] = WrapToNegPi2PosPi(newpose[2])
        newpose[2] = newpose[2] * 180 / math.pi
        return newpose

    # 伸缩变换 （optional）
    def _scaleOnePoint(self, factor, centorPt, pt):
        pt[0] = factor * (pt[0]-centorPt[0]) + centorPt[0]
        pt[1] = factor * (pt[1]-centorPt[1]) + centorPt[1]
        return pt
    # TODO：增加对轨迹点路径类型的伸缩变换
    # 接口：伸缩所有的路径（optional）
    def ScalePaths(self, factor, centorPt):
        for path in self.paths:
            if self.pathTypeLine == path["type"]:
                for point in path["points"]:
                    self._scaleOnePoint(factor, centorPt, point)
                targetPt = self._scaleOnePoint(factor, centorPt, path["targetPos"][:-1])
                path["targetPos"][:-1] = targetPt
            elif self.pathTypeBezier == path["type"]:
                for point in path["points"]:
                    self._scaleOnePoint(factor, centorPt, point)
                targetPt = self._scaleOnePoint(factor, centorPt, path["targetPos"][:-1])
                path["targetPos"][:-1] = targetPt

    # 接口：对所有的路径进行坐标变换，从相对于ref坐标系，变换到相对于cur坐标系
    def TransformAllPoints(self):
        for path in self.paths:
            self._transformOnePos(path["targetPos"])
            if path["type"] == self.pathTypeLine or path["type"] == self.pathTypeBezier:
                for point in path["points"]:
                    self._transformOnePoint(point)
            if path["type"] == self.pathTypeTrajPts:
                for trajPt in path["points"]:
                    tmp_pos = [trajPt["x"], trajPt["y"], trajPt["theta"]]
                    self._transformOnePos(tmp_pos)
                    trajPt["x"] = tmp_pos[0]
                    trajPt["y"] = tmp_pos[1]
                    trajPt["theta"] = tmp_pos[2]


    # 接口：设置路径起点， [x(m),y(m),theta(deg)]
    def SetStartPoint(self, startPos):
        self.paths = []
        self.lastPos[0] = startPos[0]
        self.lastPos[1] = startPos[1]
        self.lastPos[2] = startPos[2] * math.pi / 180
        self.lastPos[2] = WrapToNegPi2PosPi(self.lastPos[2])
        self.startPos = self.lastPos.copy()
    # 接口：由目标的绝对位姿，添加直线路径
    # targetPos ([x(m),y(m),theta(deg)])
    def LineTo(self,targetPos,pathID=0,vel=0.3,acc=0.3):
        linePts = []
        linePts.append(self.lastPos.copy()[:-1])
        linePts.append(targetPos.copy()[:-1])
        self.lastPos = targetPos.copy()
        self.lastPos[2] = self.lastPos[2] * math.pi / 180
        self.lastPos[2] = WrapToNegPi2PosPi(self.lastPos[2])
        targetPos[2] = targetPos[2] * math.pi / 180
        targetPos[2] = WrapToNegPi2PosPi(targetPos[2])
        self.paths.append({"type":self.pathTypeLine,"points":linePts,"targetPos":targetPos,"vel":vel,"acc":acc,"pathID":pathID})
    # 接口：由确定的控制点，目标绝对位姿，添加贝塞尔路径
    # targetPos ([x(m),y(m),theta(deg)])
    def BezierTo(self,ctrlPt1,ctrlPt2,ctrlPt3,ctrlPt4,targetPos,pathID=0,vel=0.3,acc=0.3):
        bezierPts = []
        bezierPts.append(self.lastPos.copy()[:-1])
        bezierPts.append(ctrlPt1)
        bezierPts.append(ctrlPt2)
        bezierPts.append(ctrlPt3)
        bezierPts.append(ctrlPt4)
        bezierPts.append(targetPos.copy()[:-1])
        self.lastPos = targetPos.copy()
        self.lastPos[2] = self.lastPos[2] * math.pi / 180
        self.lastPos[2] = WrapToNegPi2PosPi(self.lastPos[2])
        targetPos[2] = targetPos[2] * math.pi / 180
        targetPos[2] = WrapToNegPi2PosPi(targetPos[2])
        self.paths.append({"type":self.pathTypeBezier,"points":bezierPts,"targetPos":targetPos,"vel":vel,"acc":acc,"pathID":pathID})
    # 接口：由目标的相对位姿，添加直线路径
    # 参数：relative_pos ([x(m),y(m),theta(deg)])
    def LineToRelaviePos(self, relative_pos, pathID=0, vel=0.3, acc=0.3):
        lastPose = self.lastPos
        targetPos = self._poseFrom(lastPose, relative_pos)
        self.LineTo(targetPos, pathID, vel, acc)

    # 计算当前位姿，往前len长度的位置
    def _ptFront(self,curPose,len):
        newPose=[0,0]
        newPose[0] = curPose[0]+math.cos(curPose[2])*len
        newPose[1] = curPose[1]+math.sin(curPose[2])*len
        return newPose
    # TODO：现在的中间控制点是由固定长度生成的，如0.2m，0.5m，对于不同尺寸的贝塞尔路径不适用
    # 接口：由目标的相对位姿，添加贝塞尔路径
    # 参数：relative_pos ([x(m),y(m),theta(deg)])
    # bezierType: “auto”，默认起点位姿与路径相切，终点位姿与路径相切
    # bezierType: "xy","yx", "s_xyx", "s_yxy"，详见readme.md
    def BezierToRelativePos(self, relative_pos, pathID=0, bezierType="auto", vel=0.3, acc=0.3):
        lastPose = [0, 0, 0]
        targetPos = relative_pos.copy()
        ctrlPs = np.zeros((4,2))

        if "xy" == bezierType or "yx" == bezierType:
            tmpP = np.array([targetPos[0], lastPose[1]])
            if "yx" == bezierType:
                tmpP = np.array([lastPose[0], targetPos[1]])
            ctrlPs[0] = np.array([lastPose[:-1]]) + 0.2 * (tmpP - np.array([lastPose[:-1]]))
            ctrlPs[1] = np.array([lastPose[:-1]]) + 0.5 * (tmpP - np.array([lastPose[:-1]]))
            ctrlPs[2] = np.array([targetPos[:-1]]) + 0.5 * (tmpP - np.array([targetPos[:-1]]))
            ctrlPs[3] = np.array([targetPos[:-1]]) + 0.2 * (tmpP - np.array([targetPos[:-1]]))
        
        if "s_xyx" == bezierType or "s_yxy" == bezierType:
            tmpP = np.array([targetPos[0], lastPose[1]])
            if "s_yxy" == bezierType:
                tmpP = np.array([lastPose[0], targetPos[1]])
            ctrlPs[0] = np.array([lastPose[:-1]]) + 0.2 * (tmpP - np.array([lastPose[:-1]]))
            ctrlPs[1] = np.array([lastPose[:-1]]) + 0.5 * (tmpP - np.array([lastPose[:-1]]))
            tmpP = np.array([lastPose[0], targetPos[1]])
            if "s_yxy" == bezierType:
                tmpP = np.array([targetPos[0], lastPose[1]])
            ctrlPs[2] = np.array([targetPos[:-1]]) + 0.5 * (tmpP - np.array([targetPos[:-1]]))
            ctrlPs[3] = np.array([targetPos[:-1]]) + 0.2 * (tmpP - np.array([targetPos[:-1]]))

        if "auto" == bezierType:
            targetPos[2] = targetPos[2] / 180 * math.pi
            ctrlPs[0] = np.array(self._ptFront(lastPose, 0.2))
            ctrlPs[1] = np.array(self._ptFront(lastPose, 0.5))
            ctrlPs[2] = np.array(self._ptFront(targetPos, -0.5))
            ctrlPs[3] = np.array(self._ptFront(targetPos, -0.2))

        lastPose = self.lastPos.copy()
        targetPos = self._poseFrom(lastPose, relative_pos)
        ctrlPt1 = self._poseFrom(lastPose, [ctrlPs[0][0], ctrlPs[0][1], 0])
        ctrlPt2 = self._poseFrom(lastPose, [ctrlPs[1][0], ctrlPs[1][1], 0])
        ctrlPt3 = self._poseFrom(lastPose, [ctrlPs[2][0], ctrlPs[2][1], 0])
        ctrlPt4 = self._poseFrom(lastPose, [ctrlPs[3][0], ctrlPs[3][1], 0])
        self.BezierTo([ctrlPt1[0],ctrlPt1[1]],[ctrlPt2[0],ctrlPt2[1]],[ctrlPt3[0],ctrlPt3[1]],[ctrlPt4[0],ctrlPt4[1]],targetPos,pathID,vel,acc)

    # TODO：只对两条贝塞尔路径做了测试，对直线路径没测
    # 接口：把若干条连续的路径转换为路径点。
    # 参数要求：pathList的id是连着的，如[2,3]， 不连续的[2,4]是不可以的
    def GenerateTrajPts(self, pathList):
        curPose = self.startPos.copy()
        if len(pathList) > 0 and pathList[0] < len(self.paths) and pathList[0] > 0:
            curPose = self.paths[pathList[0]-1]["targetPos"]
        for pathID in pathList:
            if pathID >= len(self.paths) or self.paths[pathID]["type"] == self.pathTypeTrajPts:
                print("ERROR: GenerateTrajPts ERROR, wrong pathList")
                return
        for pathID in pathList:
            tmpPath = self.paths[pathID]
            pathTskParam = {"pathType":      tmpPath["type"],
                            "actionType":    0,
                            "CtrlPt":        tmpPath["points"],
                            "maxVel":        tmpPath["vel"],
                            "maxAcc":        tmpPath["acc"],
                            "creepVel":      0.01,
                            "creepDistance": 0.01,
                            "dir":           1,
                            "pathId":        tmpPath["pathID"]
                            }   
            pathRtn = self.libModules.PathCalcParams(pathTskParam, curPose)
            if pathRtn != 0:
                print("ERROR when PathCalcParams: %d", pathRtn)
                return
            self.libModules.MultiAxisAddPath()
            sleep(0.2)
        
        targetPos = self.paths[pathList[-1]]["targetPos"]
        del self.paths[pathList[0]:pathList[-1]+1]

        sCurvePG_totalT = self.libModules.GetTotalT()
        sCurvePG_totalT = sCurvePG_totalT + 10
        sampleCnt = 0

        #轨迹点类型
        #字典key:  type, targetPos(3), trajPtSize, points
        #points的元素是一个字典，字典key: vel, acc, omega, jerk, relativeTime, x, y, theta, kappa, dkappa, accumulate_s
        trajPath = {"type":self.pathTypeTrajPts,"points":[],"targetPos":targetPos,"trajPtSize":sampleCnt}
        
        time_delta = self.trajPtTimeDeltaCnt  #每0.05s采样一次
        for i in range(sCurvePG_totalT):
            setPt, setLinearVel, t = self.libModules.GetCurPos()
            if i % time_delta == 0 or i == sCurvePG_totalT-1:
                trajPt = self.libModules.GetPointInPath(setPt, setLinearVel, t)
                trajPath["points"].append(trajPt.copy())
                sampleCnt = sampleCnt+1
        print("trajPtSize: %d, totalT: %d" %(sampleCnt, sCurvePG_totalT))
        trajPath["trajPtSize"] = sampleCnt
        self.paths.insert(pathList[0], trajPath)

    # 接口：检查轨迹点路径是否满足主控板代码中对外部轨迹的要求
    def CheckPathofTrajPts(self):
        for path in self.paths:
            if self.pathTypeTrajPts == path["type"]:
                size = path["trajPtSize"]
                if size <= 1:
                    return
                xPre = path["points"][0]["x"]
                yPre = path["points"][0]["y"]
                thetaPre = path["points"][0]["theta"]
                time_delta_cnt = self.trajPtTimeDeltaCnt
                for i in range(1, size):
                    if math.fabs(xPre - path["points"][i]["x"]) > (0.02*time_delta_cnt) or \
                        math.fabs(yPre - path["points"][i]["y"]) > (0.02*time_delta_cnt) or \
                        math.fabs(GetAngleDiff(thetaPre, path["points"][i]["theta"])) > (0.02*time_delta_cnt):
                        print("ERROR: CheckTrajPts Failed at TrajPt_%d"%i)
                    xPre = path["points"][i]["x"]
                    yPre = path["points"][i]["y"]
                    thetaPre = path["points"][i]["theta"]

    # 接口：可视化路径
    def Display(self, ax):
        origin_RefCoorSys = [0, 0]
        X_RefCoorSys = [4, 0]
        Y_RefCoorSys = [0, 4]
        self._transformOnePoint(origin_RefCoorSys)
        self._transformOnePoint(X_RefCoorSys)
        self._transformOnePoint(Y_RefCoorSys)
        # ax.plot([origin_RefCoorSys[0],X_RefCoorSys[0]], [origin_RefCoorSys[1],X_RefCoorSys[1]], 'b--')
        # ax.text(X_RefCoorSys[0], X_RefCoorSys[1], "X_ref")
        # ax.plot([origin_RefCoorSys[0],Y_RefCoorSys[0]], [origin_RefCoorSys[1],Y_RefCoorSys[1]], 'b--')
        # ax.text(Y_RefCoorSys[0], Y_RefCoorSys[1], "Y_ref")
        pathid = 1
        for path in self.paths:
            pathid = pathid+1
            if self.pathTypeLine == path["type"]:
                line_startPt = path["points"][0]
                line_targetPt = path["points"][1]
                DrawPathLine(ax, line_startPt, line_targetPt, color=self.colors[pathid%4])
                DrawAMR(ax, path["targetPos"])
            elif self.pathTypeBezier == path["type"]:
                bezier_CtrlPs = np.array([path["points"][0],
                                          path["points"][1],
                                          path["points"][2],
                                          path["points"][3],
                                          path["points"][4],
                                          path["points"][5]
                                        ])
                DrawPathBezier(ax, bezier_CtrlPs, color=self.colors[pathid%4])
                DrawAMR(ax, path["targetPos"])
            elif self.pathTypeTrajPts == path["type"]:
                x_list = [0] * path["trajPtSize"]
                y_list = [0] * path["trajPtSize"]
                theta_list = [0] * path["trajPtSize"]
                vel_list = [0] * path["trajPtSize"]
                omega_list = [0] * path["trajPtSize"]
                relative_time = [0] * path["trajPtSize"]
                for i in range(path["trajPtSize"]):
                    x_list[i] = path["points"][i]["x"]
                    y_list[i] = path["points"][i]["y"]
                    theta_list[i] = path["points"][i]["theta"]
                    vel_list[i] = path["points"][i]["vel"]
                    omega_list[i] = path["points"][i]["omega"]
                    relative_time[i] = path["points"][i]["relativeTime"]
                    if i % 50 == 0:
                        DrawAMR(ax, [x_list[i], y_list[i], theta_list[i]])
                ax.scatter(x_list, y_list, color=self.colors[pathid%4], marker = '*')
                DrawAMR(ax, path["targetPos"])
                fig1, axs1 = plt.subplots(3, 1)
                axs1[0].plot(relative_time, vel_list)
                axs1[0].set_title("vel")
                axs1[1].plot(relative_time, omega_list)
                axs1[1].set_title("omega")
                axs1[2].plot(relative_time, theta_list)
                axs1[2].set_title("theta")
                fig2, axs2 = plt.subplots(3, 1)
                axs2[0].plot(relative_time, x_list)
                axs2[0].set_title("x")
                axs2[1].plot(relative_time, y_list)
                axs2[1].set_title("y")
                axs2[2].plot(relative_time, theta_list)
                axs2[2].set_title("theta")
        ax.set_aspect(1)
    
    # 接口：生成路径脚本
    def MoveScript(self):
        script=""
        for path in self.paths:
            if self.pathTypeLine==path["type"]:
                script=script+"46.MovePath %d %d %d %d "%(path["vel"]*1000,path["acc"]*1000,4000,4000)
                for pt in path["points"]:
                    script=script+"%d %d "%(pt[0]*1000,pt[1]*1000)
                dirAngle = path["targetPos"][-1]
                script=script+"%d %d %d \n"%(dirAngle/math.pi*18000, path["pathID"], 0)
            if self.pathTypeBezier==path["type"]:
                script=script+"46.MovePath %d %d %d %d "%(path["vel"]*1000,path["acc"]*1000,4000,4000)
                script=script+"%d %d "%(path["points"][0][0]*1000,path["points"][0][1]*1000)
                script=script+"%d %d "%(path["points"][5][0]*1000,path["points"][5][1]*1000)
                dirAngle = path["targetPos"][-1]
                script=script+"%d %d %d "%(dirAngle/math.pi*18000, path["pathID"], 1)
                for i in range(1,5):
                    script=script+"%d %d "%(path["points"][i][0]*1000,path["points"][i][1]*1000)  
                script=script+"\n"      
            if self.pathTypeTrajPts==path["type"]:
                segment_split_num = math.ceil(path["trajPtSize"] / self.trajPtSizeMax)
                tmp_mod = path["trajPtSize"] % self.trajPtSizeMax
                seg_ptSize = self.trajPtSizeMax
                seg_ptStart = 0
                for seg_ind in range(segment_split_num):
                    if seg_ind == segment_split_num - 1 and tmp_mod > 0:
                        seg_ptSize = tmp_mod
                    script=script+"35.IdealTraJectory %d %d %d %d %d %d "%(0, 0, path["targetPos"][0]*1000, 
                                                    path["targetPos"][1]*1000, path["targetPos"][2]/math.pi*18000, seg_ptSize)    
                    for i in range(seg_ptStart, seg_ptStart+seg_ptSize):
                        trajPt = path["points"][i]
                        print("trajPt_%d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f"%(i, 
                                            trajPt["vel"], 
                                            trajPt["acc"],
                                            trajPt["omega"],
                                            trajPt["jerk"],
                                            trajPt["relativeTime"],
                                            trajPt["x"],
                                            trajPt["y"],
                                            trajPt["theta"],
                                            trajPt["kappa"],
                                            trajPt["dkappa"],
                                            trajPt["accumulate_s"]))
                        bytes_trajPt = struct.pack('<4h3ih3i',int(1000*trajPt["vel"]),
                                                        int(1000*trajPt["acc"]),
                                                        int(trajPt["omega"]/math.pi*18000),
                                                        int(1000*trajPt["jerk"]),
                                                        int(1000*trajPt["relativeTime"]),
                                                        int(1000*trajPt["x"]),
                                                        int(1000*trajPt["y"]),
                                                        int(trajPt["theta"]/math.pi*18000),
                                                        int(trajPt["kappa"]),
                                                        int(trajPt["dkappa"]),
                                                        int(trajPt["accumulate_s"]))    
                        # str_trajPt = ('%s'%bytes_trajPt).replace('\\','\\\\')
                        # script=script + '%s'%str_trajPt
                        # if i != seg_ptStart+seg_ptSize-1:
                        #     script = script + ","
                        tmpBuf = struct.unpack('34B', bytes_trajPt)
                        for byte in tmpBuf:
                            script = script + str(byte) + ","
                    script=script.strip(",")+"\n" 
                    seg_ptStart = seg_ptStart + seg_ptSize
        return script         



if __name__ == '__main__':  
    pathGen = PathGen()

    # test_sdPath1()

    plt.show()


