# coding=utf-8
import imp
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.dirname(__file__)+'/..')
print (sys.path)
from pathGen import PathGen


def test_sdPath1():
    pathGen = PathGen()

    # step1: start Pos in sd_Map and in sh_Map
    pos_in_sd_Map = [36.5,20,0]  #[m, m, deg]
    pos_in_sh_Map = [11.26,19.08,91.46]

    # step2: calculate relative pos from sd_Map to sh_Map
    pathGen.CalRelativePos(pos_in_sd_Map, pos_in_sh_Map)

    # step3: set start Point, based on sd_Map
    pathGen.SetStartPoint(pos_in_sd_Map[:])

    # step4: add PathLine, and PathBezier
    #        params in pathGen.LineTo: [target_x (m), target_y (m), target_heading (deg)], pathID, vel, acc
    #        params in pathGen.BezierTo: (ctrlP1, ctrlP2, ctrlP3, ctrlP4,  [target_x (m), target_y (m), target_heading (deg)], pathID, vel, acc)
    pathID = 1
    vel = 0.3
    acc = 0.3
    
    amr_pos = [38, 20, 0]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [39.5, 20, 0]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [41, 20, 0]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [43, 20, 0]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [44, 19.25, -90]
    pathID = pathID + 1
    pathGen.BezierTo([43.25, 20.0],[43.6, 20.0],[44.0, 19.85],[44.0, 19.6],amr_pos, pathID, vel, acc)

    amr_pos = [44, 17.75, -90]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [44, 16, -90]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [44, 14.5, -90]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)


    # go back
    amr_pos = [44, 16, 90]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [44, 17.75, 90]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [44, 19.25, 90]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [43, 20, 180]
    pathID = pathID + 1
    pathGen.BezierTo([44.0, 19.6],[44.0, 19.85],[43.65, 20.0],[43.3, 20.0],amr_pos, pathID, vel, acc)

    amr_pos = [41, 20, 180]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [39.5, 20, 180]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [38, 20, 180]
    pathID = pathID + 1
    pathGen.LineTo(amr_pos, pathID, vel, acc)

    amr_pos = [36.5, 20, 0]
    pathID = pathID + 1
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

    # # step8(optional): scale the path
    # pathGen.ScalePaths(0.45, pathGen.points[0])
    # script = pathGen.MoveScript()
    # print(script)
    # pathGen.Display(ax)
    pass



if __name__ == '__main__':  

    test_sdPath1()
    plt.show()
