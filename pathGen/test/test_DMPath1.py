# coding=utf-8
import imp
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.dirname(__file__)+'/..')
print (sys.path)
from pathGen import PathGen


# 上海二维码地图路径
def test_DMPath1():
    pathGen = PathGen()

    # step1: start Pos 
    pos_in_DM_Map = [107,100,90]  #[m, m, deg]
    pos_in_Map = [107,100,90]

    # step2: calculate relative pos
    pathGen.CalRelativePos(pos_in_DM_Map, pos_in_Map)

    # step3: set start Point
    pathGen.SetStartPoint(pos_in_DM_Map[:])

    # step4: add PathLine, and PathBezier
    #        params in pathGen.LineTo: [target_x (m), target_y (m), target_heading (deg)]
    #        params in pathGen.BezierTo: (ctrlP1, ctrlP2, ctrlP3, ctrlP4,  [target_x (m), target_y (m), target_heading (deg)])
    pathID = -1
    vel = 0.3
    acc = 0.3
    
    pathID = pathID+1
    amr_pos = [107,101,90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [107,102,90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [107,103,90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [107,104,90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [107,105,90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)
    
    pathID = pathID+1
    amr_pos = [106,105,180]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [106,104,-90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [106,103,-90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [106,102,-90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [106,101,-90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [106,100,-90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    pathID = pathID+1
    amr_pos = [107,100,90]
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    # step5: transform
    pathGen.TransformAllPoints()

    # step6: generate the script
    script = pathGen.MoveScript()
    print(script)

    # step7: display
    fig = plt.figure()
    ax = fig.add_subplot()
    pathGen.Display(ax)

    pass



if __name__ == '__main__':  

    test_DMPath1()
    plt.show()
