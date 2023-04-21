## 生成路径脚本的py工具

测试test文件夹下的示例时，可能会报错：ModuleNotFoundError: No module named 'pathGen'
方法：在powershell下运行，或者在vscode下调试运行

0. [文件说明](#文件说明)

1. [功能概述](#功能概述)

2. [具体使用情况说明](#具体使用情况说明)

    2.1 [使用绝对位姿生成路径脚本](#情况2.1)
    - 2.1.1 [步骤](#2.1.1步骤)
    - 2.1.2 [测试用例](#2.1.2测试用例)

    2.2 [使用相对路径的接口，生成一组路径脚本](#情况2.2)
    - 2.2.1 [步骤](#2.2.1步骤)
    - 2.2.2 [贝塞尔路径的类型](#2.2.2贝塞尔路径的类型)
    - 2.2.3 [测试用例test_relativePath1()](#2.2.3测试用例)
    - 2.2.4 [测试用例test_relativePath2()](#2.2.4测试用例)

    2.3 [由若干条可接续路径生成轨迹点](#情况2.3)
    - 2.3.1 [测试用例test_relativePath2()](#2.3.1测试用例)
    - 2.3.2 [测试用例test_trajPtsPath_Demo()](#2.3.2测试用例)

    2.4 [用地图1的路径点，生成可在地图2运行的路径脚本](#情况2.4)
    - 2.4.1 [步骤](#2.4.1步骤)
    - 2.4.2 [测试用例](#2.4.2测试用例)



### 0 文件说明 {#文件说明}

```
1. pathGen.py                     # 生成路径脚本的接口
2. libModules.dll, libModules.py  # 由MCUBoard_Main中的Modules模块编译的库，和相应的python封装
                                  # 使用的MCUBoard_Main的分支：feature/test_add_wrap; commit: 2e9734183df8c5b11a6272fcc1e6c62db540912f
3. utilityPG.py                   # 一些功能函数
4. test                           # 一些测试例
```

### 1 功能概述 {#功能概述}

1. 使用绝对位姿生成路径脚本，也就是直接用路径点生成路径脚本
2. 使用相对位姿生成路径脚本
3. 在生成的路径基础上，由若干条可接续路径生成外部轨迹点
4. 用地图1的路径点，生成可在地图2运行的路径脚本

### 2 一些具体使用情况的说明 {#具体使用情况说明}

#### 2.1 使用绝对位姿生成路径脚本，也就是直接用路径点生成路径脚本 {#情况2.1}

##### 2.1.1 步骤 {#2.1.1步骤}

1. 确定路径起点位姿

2. （无操作）

3. 设置路径起点

4. 设置路径

5. （无操作）

6. 生成路径脚本

7. 可视化

##### 2.1.2 测试用例：```test_Path_Demo()``` {#2.1.2测试用例}

#### 2.2 使用相对路径的接口，生成一组路径脚本 {#情况2.2}

说明：以AMR车体坐标系为参考系，根据下一条路径的终点相对位姿，生成路径。

##### 2.2.1 步骤 {#2.2.1步骤}

1. 确定总路径起点在AMR车体坐标系的位姿，和在地图坐标系的位姿。
注：总路径起点在AMR车体坐标系的位姿，总是[0, 0, 0]

2. 计算起始时，车体坐标系与地图坐标系的变换矩阵

3. 基于AMR车体坐标系，设置路径起点

4. 用相对位姿的方法，添加直线路径，和贝塞尔路径

    - 直线路径：
        - 接口：pathGen.LineToRelaviePos(relative_pos, pathID=0, vel=0.3, acc=0.3)
        - 参数：relative_pos：相对于上一条路径终点位姿的相对位姿。

    - 贝塞尔路径：
        - 接口：pathGen.BezierToRelativePos(relative_pos, pathID=0, bezierType="auto", vel=0.3, acc=0.3)
        - 参数：
            - relative_pos：相对于上一条路径终点位姿的相对位姿。
            - type: 可选值有{'auto', 'xy', 'yx', 's_yxy', 's_xyx'}, 默认值 'auto'
                - 'auto' 时，贝塞尔曲线满足起点heading和曲线切线方向一致；终点heading与曲线切线方向一致
                - 其他值时，详见后面贝塞尔路径的类型

5. 把相对于起始时车体坐标系的路径，转换到地图坐标系下

6. 生成路径脚本

7. 可视化

##### 2.2.2 贝塞尔路径的类型 {#2.2.2贝塞尔路径的类型}

0. type: 'auto'
    
    说明：贝塞尔曲线满足起点heading和曲线切线方向一致；终点heading与曲线切线方向一致。曲线的形状由终点相对位姿的theta角决定。
    | [1,1,0]      | [1,-1,0]        |[1,1,90]      |
    | -------------| ----------------|--------------|
    | <img src=fig/bezier_auto_1.png width="300" /> | <img src=fig/bezier_auto_2.png width="300" /> |<img src=fig/bezier_auto_3.png width="300" />|
    
    | [1,-1,-90]   | [0,1,180]       |[0,-1,180]      |
    | -------------| ----------------|--------------|
    | <img src=fig/beizer_auto_4.png width="300" /> | <img src=fig/bezier_auto_5.png width="300" /> | <img src=fig/bezier_auto_6.png width="300" /> |

1. type：'xy'

    说明：以上一条路径终点位姿为参考系，倾向于先沿着其x轴方向移动，再沿着其y轴方向移动

    ```py
    relative_pos = [2, 2, 0]
    type = 'xy'
    pathGen.BezierToRelativePos(relative_pos, type)
    ```

    <div align=center>
    <img src=fig\bezier_type_xy.png alt="bezier_type_xy" width="500" />
    </div>

2. type：'yx'

    说明：以上一条路径终点位姿为参考系，倾向于先沿着其y轴方向移动，再沿着其x轴方向移动

    ```py
    relative_pos = [2, 2, 0]
    type = 'yx'
    pathGen.BezierToRelativePos(relative_pos, type)
    ```

    <div align=center>
    <img src=fig\bezier_type_yx.png alt="bezier_type_yx" width="500" />
    </div>

3. type：'s_xyx'

    说明：以上一条路径终点位姿为参考系，走s型的贝塞尔，倾向于先沿着其x轴方向移动，然后沿着其y轴方向移动，再沿着其x轴方向移动

    ```py
    relative_pos = [2, 2, 0]
    type = 's_xyx'
    pathGen.BezierToRelativePos(relative_pos, type)
    ```

    <div align=center>
    <img src=fig\bezier_type_s_xyx.png alt="bezier_type_s_xyx" width="500" />
    </div>

4. type：'s_yxy'

    说明：以上一条路径终点位姿为参考系，走s型的贝塞尔，倾向于先沿着其y轴方向移动，然后沿着其x轴方向移动，再沿着其y轴方向移动

    ```py
    relative_pos = [2, 2, 0]
    type = 's_yxy'
    pathGen.BezierToRelativePos(relative_pos, type)
    ```

    <div align=center>
    <img src=fig\bezier_type_s_yxy.png alt="bezier_type_s_yxy" width="500" />
    </div>

##### 2.2.3 测试用例：```test_relativePath1()``` {#2.2.3测试用例}

**Step1**. 确定总路径起点在AMR车体坐标系的位姿，和在地图坐标系的位姿
```py
pos_in_AMR = [0,0,0]
pos_in_Map = [16.43,27.708,-178]
```

**Step4**. 用相对位姿的方法，添加直线路径，和贝塞尔路径

直线路径：
```py
relative_pos = [1, 0, 0]
pathGen.LineToRelaviePos(relative_pos)
```

贝塞尔路径：
```py
relative_pos = [1, 1, 0]
type = 'xy'
pathGen.BezierToRelativePos(relative_pos, type)
```

。。。

**结果**：
转换到地图坐标系下的路径
<div align=center>
<img src=fig\test_relativePath.png alt="test_relativePath" width="500" />
</div>

生成的脚本：
```bash
46.MovePath 300 300 4000 4000 16430 27708 15430 27673 -17800 2 0
46.MovePath 300 300 4000 4000 15430 27673 14466 26638 -17800 3 1 15230 27666 14930 27655 14448 27138 14459 26838
46.MovePath 300 300 4000 4000 14466 26638 13501 25604 -8800 4 1 14473 26438 14483 26139 14001 25621 13701 25611
46.MovePath 300 300 4000 4000 13501 25604 14535 24640 -8800 5 1 13701 25611 14001 25621 14036 24622 14336 24633
46.MovePath 300 300 4000 4000 14535 24640 15500 25674 9200 6 1 14528 24839 14518 25139 15517 25174 15507 25474
46.MovePath 300 300 4000 4000 15500 25674 14466 26638 18200 7 1 15493 25874 15482 26174 14965 26656 14665 26645
```

</br>

##### 2.2.4 测试用例：```test_relativePath2()``` {#2.2.4测试用例}

bezier_type，选择auto的情况

#### 2.3 在生成的路径基础上，由若干条可接续路径生成外部轨迹点 {#情况2.3}

接口：
```
# pgPaths中必须是连续的pathID
pgPaths = [1, 2]
pathGen.GenerateTrajPts(pgPaths)     #把参数列表中的路径，转换为轨迹点
pathGen.CheckPathofTrajPts()         #检查生成的轨迹点的相邻点的间隔是否满足主控板代码对外部轨迹点的要求
```

使用方法：
在前述使用情况1/2所介绍的流程中，增加第4.5步，参考下面测试用例

##### 2.3.1 测试用例：```test_relativePath2()``` {#2.3.1测试用例}
##### 2.3.2 测试用例：```test_trajPtsPath_Demo()``` {#2.3.2测试用例}


#### 2.4 用地图1的路径点，生成可在地图2运行的路径脚本 {#情况2.4}

##### 2.4.1 步骤 {#2.4.1步骤}

1. 确定地图1的路径起点位姿，和地图2的路径起点位姿

2. 计算两个地图坐标系的变换矩阵

3. 基于地图1，设置路径起点

4. 基于地图1，设置路径

5. 把地图1的路径，转换到地图2

6. 生成可在地图2运行的路径脚本

7. 可视化

8. 缩放路径（可选，缩放后的路径的贝塞尔路径的曲率变化会不一样？）

##### 2.4.2 测试用例：```test_sdPath1()``` {#2.4.2测试用例}

顺德的长运地图如下：
<div align=center>
<img src=fig\sdMap.png alt="sdMap" width="700" />
</div>

上海一楼的地图简化图如下：
<div align=center>
<img src=fig\shMap.png alt="shMap" width="300" />
</div>

**Step1**. 确定地图1的路径起点位姿，和地图2的路径起点位姿
```py
pos_in_sd_Map = [36.5,20,0]  #[m, m, deg]
pos_in_sh_Map = [11.26,19.08,91.46]
```

**Step4**. 基于地图1，设置路径

直线路径：
```py
#params in pathGen.LineTo: [target_x (m), target_y (m), target_heading (deg)], pathID, vel, acc
amr_pos = [38, 20, 0]
pathGen.LineTo(amr_pos, pathID, vel, acc)
```

贝塞尔路径：
```py
#params in pathGen.BezierTo: (ctrlP1, ctrlP2, ctrlP3, ctrlP4,  [target_x (m), target_y (m), target_heading (deg)], pathID, vel, acc)
amr_pos = [44, 19.25, -90]
pathGen.BezierTo([43.25, 20.0],[43.6, 20.0],[44.0, 19.85],[44.0, 19.6],amr_pos, pathID, vel, acc)
```

。。。

**结果**：
原始路径（图中红线）：
<div align=center>
<img src=fig\path_sdMap.png alt="path_sdMap" width="500" />
</div>

转换到上海地图坐标系下的路径：
<div align=center>
<img src=fig\path_shMap.png alt="path_shMap" width="700" />
</div>

生成的脚本：
```bash
46.MovePath 300 300 4000 4000 11260 19079 11221 20579 9146 2 0 
46.MovePath 300 300 4000 4000 11221 20579 11183 22079 9146 3 0
46.MovePath 300 300 4000 4000 11183 22079 11145 23578 9146 4 0
46.MovePath 300 300 4000 4000 11145 23578 11094 25577 9146 5 0
46.MovePath 300 300 4000 4000 11094 25577 11818 26596 145 6 1 11088 25827 11079 26177 11218 26581 11468 26587
46.MovePath 300 300 4000 4000 11818 26596 13318 26634 145 7 0
46.MovePath 300 300 4000 4000 13318 26634 15067 26679 145 8 0
46.MovePath 300 300 4000 4000 15067 26679 16567 26717 145 9 0
46.MovePath 300 300 4000 4000 16567 26717 15067 26679 18146 10 0
46.MovePath 300 300 4000 4000 15067 26679 13318 26634 18146 11 0
46.MovePath 300 300 4000 4000 13318 26634 11818 26596 18146 12 0
46.MovePath 300 300 4000 4000 11818 26596 11094 25577 27146 13 1 11468 26587 11218 26581 11077 26227 11086 25877
46.MovePath 300 300 4000 4000 11094 25577 11145 23578 27146 14 0
46.MovePath 300 300 4000 4000 11145 23578 11183 22079 27146 15 0
46.MovePath 300 300 4000 4000 11183 22079 11221 20579 27146 16 0
46.MovePath 300 300 4000 4000 11221 20579 11260 19079 9146 17 0
```

</br>
