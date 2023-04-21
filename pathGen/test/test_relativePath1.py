# coding=utf-8
import imp
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.dirname(__file__)+'/..')
print (sys.path)
from pathGen import PathGen


def test_relativePath1():
    pathGen = PathGen()

    # step1: start Pos in AMR_frame and in Map_frame
    pos_in_AMR = [0,0,0]
    # pos_in_Map = [0,0,0]
    pos_in_Map = [16.43,27.708,-178]

    # step2: calculate relative pos from AMR_frame to Map_frame
    pathGen.CalRelativePos(pos_in_AMR, pos_in_Map)

    # step3: set start Point, based on AMR_frame
    pathGen.SetStartPoint(pos_in_AMR[:])

    # step4: add PathLine, and PathBezier
    #        params in pathGen.LineToRelaviePos: [relative_pos (m), relative_pos (m), relative_pos (deg)], pathID, vel, acc
    #                relative to the position of the target of the last path
    #        params in pathGen.BezierToRelativePos: ([relative_pos (m), relative_pos (m), relative_pos (deg)], pathID, type, vel, acc)
    #                relative_pos: relative to the position of the target of the last path
    #                type: the available value is in {'xy', 'yx', 's_yxy', 's_xyx'}
    pathID = 0
    vel = 0.3
    acc = 0.3

    pathID = pathID + 1
    relative_pos = [1, 0, 0]
    pathGen.LineToRelaviePos(relative_pos, pathID, vel, acc)

    pathID = pathID + 1
    relative_pos = [1, 1, 0]
    tmp_type = 'xy'
    pathGen.BezierToRelativePos(relative_pos, pathID, bezierType=tmp_type, vel=vel, acc=acc)

    pathID = pathID + 1
    relative_pos = [1, 1, 90]
    tmp_type = 'yx'
    pathGen.BezierToRelativePos(relative_pos, pathID, bezierType=tmp_type, vel=vel, acc=acc)

    pathID = pathID + 1
    relative_pos = [1, 1, 0]
    tmp_type = 's_yxy'
    pathGen.BezierToRelativePos(relative_pos, pathID, bezierType=tmp_type, vel=vel, acc=acc)

    pathID = pathID + 1
    relative_pos = [-1, 1, 180]
    tmp_type = 's_xyx'
    pathGen.BezierToRelativePos(relative_pos, pathID, bezierType=tmp_type, vel=vel, acc=acc)

    pathID = pathID + 1
    relative_pos = [1, 1, 90]
    tmp_type = 'xy'
    pathGen.BezierToRelativePos(relative_pos, pathID, bezierType=tmp_type, vel=vel, acc=acc)

    # step5: transform
    pathGen.TransformAllPoints()

    # step6: generate the script
    script = pathGen.MoveScript()
    print(script)

    # step7: display
    fig = plt.figure()
    ax = fig.add_subplot()
    pathGen.Display(ax)



if __name__ == '__main__':  

    test_relativePath1()
    plt.show()
