import imp
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.dirname(__file__)+'/..')
print (sys.path)
from pathGen import PathGen

def test_relativePath_Demo():
    pathGen = PathGen()

    # step1: start Pos in AMR_frame and in Map_frame
    pos_in_AMR = [0,0,0]
    pos_in_Map = [0,0,0]                           # 根据实际地图的路径起点修改，如果用里程计定位的方法，且重置了小车位置，则pos_in_Map = [0,0,0]
    # pos_in_Map = [16.43,27.708,90]

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

    pathID = pathID + 1 
    relative_pos = [1, 1, 90]
    pathGen.BezierToRelativePos(relative_pos, pathID, vel=vel, acc=acc)

    pathID = pathID + 1
    relative_pos = [0, 2, 90]
    pathGen.LineToRelaviePos(relative_pos, pathID, vel, acc)

    pathID = pathID + 1
    relative_pos = [0, 1, 180]
    pathGen.LineToRelaviePos(relative_pos, pathID, vel, acc)

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

    test_relativePath_Demo()
    plt.show()
