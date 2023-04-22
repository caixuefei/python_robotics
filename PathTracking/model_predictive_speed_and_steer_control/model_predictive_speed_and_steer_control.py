"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
import sys
import pathlib
import gif
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
print (sys.path)
from pathGen.pathGenerator import PathGen
from PathPlanning.CubicSpline import cubic_spline_planner

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 1.0, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 0.1  # goal distance
STOP_SPEED = 0.1  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 1.2  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.01  # [s] time tick

# Vehicle parameters
LENGTH = 2.0  # [m]
WIDTH = 1.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.2  # [m]
WHEEL_WIDTH = 0.1  # [m]
TREAD = 0.5  # [m]
WB = 0.0  # [m]

MAX_OMEGA = np.deg2rad(30.0) # 最大角速度
MAX_SPEED = 2.0  # maximum speed [m/s]
MIN_SPEED = -2.0  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, omega):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT 

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = 0

    return A, B, C


def plot_car(x, y, yaw, axis, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y

    axis.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    axis.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    axis.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    axis.plot(x, y, "*")


def update_state(state, a, omega):

    # input check
    # if delta >= MAX_STEER:
    #     delta = MAX_STEER
    # elif delta <= -MAX_STEER:
    #     delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + omega * DT
    state.v = state.v + a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, oomega, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, omegai, i) in zip(oa, oomega, range(1, T + 1)):
        state = update_state(state, ai, omegai)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, omegaref, oa, oomega):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if oa is None or oomega is None:
        oa = [0.0] * T
        oomega = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, oomega, xref)
        poa, poomega = oa[:], oomega[:]
        oa, oomega, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, omegaref)
        du = sum(abs(oa - poa)) + sum(abs(oomega - poomega))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, oomega, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, omegaref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    omegaref: reference chassis omega
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(xref[2, t], xref[3, t], omegaref[0,t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            # constraints += [cvxpy.abs(u[1, t]) <= MAX_OMEGA * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_OMEGA]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        oomega = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, oomega, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, oomega, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, pind):
    xref = np.zeros((NX, T + 1))
    omegaref = np.zeros((1, T + 1))
    ncourse = len(cx)

    # ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    # if pind >= ind:
    #     ind = pind
    ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    omegaref[0, 0] = sp[ind]*ck[ind]  # 小车角速度

    travel = 0.0

    for i in range(T + 1):
        # travel += abs(state.v) * DT
        # dind = int(round(travel / dl))

        if (ind + i) < ncourse:
            xref[0, i] = cx[ind + i]
            xref[1, i] = cy[ind + i]
            xref[2, i] = sp[ind + i]
            xref[3, i] = cyaw[ind + i]
            omegaref[0, i] = sp[ind + i]*ck[ind + i]
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            omegaref[0, i] = sp[ncourse - 1]*ck[ncourse - 1]

    return xref, ind, omegaref


def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False


def do_simulation(cx, cy, cyaw, ck, sp, initial_state):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    a = [0.0]
    omega = [0.0]
    velref = [0.0]
    # target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    oomega, oa = None, None

    cyaw = smooth_yaw(cyaw)

    frames = []

    fig,axes = plt.subplots(2,1)

    indTime = 0
    while MAX_TIME >= time:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, indTime)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, oomega, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, oomega)

        if oomega is not None:
            omegai, ai = oomega[0], oa[0]

        state = update_state(state, ai, omegai)
        indTime += 1 
        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        omega.append(omegai)
        a.append(ai)
        velref.append(xref[2,0])

        if check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plot_track(ox,oy,x,y,cx,cy,xref,target_ind,state,time,fig,axes)

            axes[1].plot(t, v, "-r", label="state speed")
            axes[1].plot(t, velref, "-b", label="reference speed")
            axes[1].grid(True)
            axes[1].set_xlabel("Time [s]")
            axes[1].set_ylabel("Speed [m/s]")
            # frame = plot_track(ox,oy,x,y,cx,cy,xref,target_ind,state,time)
            # frames.append(frame)
            # plt.cla()
            # # for stopping simulation with the esc key.
            # plt.gcf().canvas.mpl_connect('key_release_event',
            #         lambda event: [exit(0) if event.key == 'escape' else None])
            # if ox is not None:
            #     plt.plot(ox, oy, "xr", label="MPC")
            # plt.plot(cx, cy, "-r", label="course")
            # plt.plot(x, y, "ob", label="trajectory")
            # plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            # plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            # plot_car(state.x, state.y, state.yaw)
            # plt.axis("equal")
            # plt.grid(True)
            # plt.title("Time[s]:" + str(round(time, 2))
            #           + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            # plt.pause(0.0001)

    # gif.save(frames,'track_ani.gif',duration=3.5)
    return t, x, y, yaw, v, omega, a

# @gif.frame
def plot_track(ox,oy,x,y,cx,cy,xref,target_ind,state,time,fig,axes):
    axes[0].cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    if ox is not None:
        axes[0].plot(ox, oy, "xr", label="MPC")
    axes[0].plot(cx, cy, "-r", label="course")
    axes[0].plot(x, y, "ob", label="trajectory")
    axes[0].plot(xref[0, :], xref[1, :], "xk", label="xref")
    axes[0].plot(cx[target_ind], cy[target_ind], "xg", label="target")
    plot_car(state.x, state.y, state.yaw, axes[0])
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].set_title("Time[s]:" + str(round(time, 2))
                + ", speed[m/s]:" + str(round(state.v, 2))
                + ", X[m]:" + str(round(state.x, 2)))
    plt.pause(0.0001)


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

def get_straight_course_test1(dl):
    ax = [0.0, 0.2, 0.6, 1.0, 1.2, 1.6, 2.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course2(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course3(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


def main():
    print(__file__ + " start!!")

    # pos generator
    pathGen = PathGen(trajPtSampleT=10)             #轨迹点每隔 trajPtSampleT ms 采样，trajPtSampleT需要是10ms的倍数
    # step1: start Pos in AMR_frame and in Map_frame
    pos_in_AMR = [0,0,0]
    pos_in_Map = [0,0,0]                           # 根据实际地图的路径起点修改，如果用里程计定位的方法，且重置了小车位置，则pos_in_Map = [0,0,0]
    # step2: calculate relative pos from AMR_frame to Map_frame
    pathGen.CalRelativePos(pos_in_AMR, pos_in_Map)
    # step3: set start Point, based on AMR_frame
    pathGen.SetStartPoint(pos_in_AMR[:])
    # step4: add PathLine, and PathBezier
    #        params in pathGen.LineToRelaviePos: (relative_pos, pathID=0, vel=0.3, acc=0.3)
    #                relative_pos:  relative to the position of the target of the last path
    #                               [x(m),y(m),theta(deg)]
    #                pathID:        id
    #        params in pathGen.BezierToRelativePos: (relative_pos, pathID=0, bezierType="auto", vel=0.3, acc=0.3)
    #                relative_pos:  relative to the position of the target of the last path, 
    #                               [x(m),y(m),theta(deg)]
    #                pathID:        id
    #                bezierType:    the available value is in {'auto', 'xy', 'yx', 's_yxy', 's_xyx'}
    #                               default: 'auto', the curve shape depends on relative_pos[2], 
    #                                         in other words, start heading align with the curve's start tangent dir;
    #                                                         target heading align with the curve's end tangent dir.
    pathID = -1
    vel = 0.4
    acc = 0.4
    pathID = pathID + 1
    relative_pos = [1, 0, 0]
    pathGen.LineToRelaviePos(relative_pos, pathID, vel, acc)
    # step4.5: generate trajectory points from some paths， and check 
    # pgPaths中必须是连续的pathID
    pgPaths = [0]
    pathGen.GenerateTrajPts(pgPaths)
    pathGen.CheckPathofTrajPts()
    # step5: transform
    pathGen.TransformAllPoints()
    # step6: generate the script
    script = pathGen.MoveScript()
    print(script)
    # step7: display
    fig_posgen,ax_posgen = plt.subplots()
    # ax_posgen = fig_posgen.add_subplot()
    pathGen.Display(ax_posgen)
    # plt.show()
    for path in pathGen.paths:
        cx = [0] * path["trajPtSize"]
        cy = [0] * path["trajPtSize"]
        cyaw = [0] * path["trajPtSize"]
        ck = [0] * path["trajPtSize"]
        sp = [0] * path["trajPtSize"]
        ct = [0] * path["trajPtSize"]
        for i in range(path["trajPtSize"]):
            cx[i] = path["points"][i]["x"]
            cy[i] = path["points"][i]["y"]
            cyaw[i] = path["points"][i]["theta"]
            ck[i] = path["points"][i]["kappa"]
            sp[i] = path["points"][i]["vel"]
            ct[i] = path["points"][i]["relativeTime"]
    #######################################################################################
    # simple generator
    # dl = 0.1  # course tick
    # cx, cy, cyaw, ck = get_straight_course(dl)
    # cx, cy, cyaw, ck = get_straight_course2(dl)
    # cx, cy, cyaw, ck = get_straight_course3(dl)
    # cx, cy, cyaw, ck = get_forward_course(dl)
    # cx, cy, cyaw, ck = get_switch_back_course(dl)
    # cx, cy, cyaw, ck = get_straight_course_test1(dl)

    # sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)
    ########################################################


    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, initial_state)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()


def main2():
    print(__file__ + " start!!")

    dl = 1.0  # course tick
    cx, cy, cyaw, ck = get_straight_course3(dl)

    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()


if __name__ == '__main__':
    main()
    # main2()
