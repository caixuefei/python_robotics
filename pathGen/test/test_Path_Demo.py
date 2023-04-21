# coding=utf-8
import imp
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.dirname(__file__)+'/..')
print (sys.path)
from pathGen import PathGen


def test_Path_Demo():
    pathGen = PathGen()

    # step1: start Pos
    pos_in_Map1 = [0,0,0]  #[m, m, deg]
    # pos_in_Map2 = [0,0,0]  #[m, m, deg]               #在不涉及坐标系转换时，不需要设置

    # step2: calculate relative pos from map1 to map2
    # pathGen.CalRelativePos(pos_in_Map1, pos_in_Map2)  #在不涉及坐标系转换时，不需要进行

    # step3: set start Point, based on sd_Map
    pathGen.SetStartPoint(pos_in_Map1[:])

    # step4: add PathLine, and PathBezier
    #        params in pathGen.LineTo: [target_x (m), target_y (m), target_heading (deg)], pathID, vel, acc
    #        params in pathGen.BezierTo: (ctrlP1, ctrlP2, ctrlP3, ctrlP4,  [target_x (m), target_y (m), target_heading (deg)], pathID, vel, acc)
    pathID = -1
    vel = 0.3
    acc = 0.3
    
    amr_pos = [1, 0, 0]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [2, 1, 90]
    pathID = pathID + 1
    pathGen.BezierTo([1.2, 0],[1.5, 0],[2, 0.5],[2, 0.8],amr_pos, pathID, vel, acc)

    amr_pos = [1, 1, 180]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [0, 1, 180]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [0, 0, 0]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    # step5: transform
    # pathGen.TransformAllPoints()                     #在不涉及坐标系转换时，不需要进行

    # step6: generate the script
    script = pathGen.MoveScript()
    print(script)

    # step7: display
    fig = plt.figure()
    ax = fig.add_subplot()
    pathGen.Display(ax)

    pass



if __name__ == '__main__':  

    test_Path_Demo()
    plt.show()
