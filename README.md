# XBOTCON2022无限机甲杯视觉开源代码说明

## 一.代码运行环境：

| 硬件条件 | 操作系统 | 运行库 | 编译工具链 |
| --- | --- | --- | --- |
| Raspberry Pi zero 2w | Respbian 11 (bullseye) | OpenCv-4.5.4  OpenCV_contrib-4.5.4 | Respbian:cmake3+gcc7+g++7 |
| Raspberry Pi Camera Module v1 X1 | ubuntu 20.04(最初测试时使用) | Eigen3 |  |
| 树莓派散热风扇 X1 |  | wiring pi |  |
| 亚博智能激光传感器VL53L0X（暂未使用） |  |  |  |

## 二.编译安装

### 1.下载源码

可以通过git clone 命令或者在PC上下载源码之后发送至树莓派。

### 2.更改文件路径

将CMakeLists.txt中的文件路径替换为当前环境对应的路径

### 3.标定相机

使用calibration.cpp中的代码，标定相机并更改代码，根据实际情况调整params.h中的各项参数。

### 4.编译运行

```cpp
//在文件目录下输入下列命令
cmake .&& make -j4
./XBOTCON
```

## 三.文件目录结构

```cpp
.
├── 图片及视频
│   ├── 公式1.png
│   ├── 时序图.pdf
│   ├── 运行速度.png
│   └── armer test.avi
├── calib
│   ├── calibration_result.txt//相机标定结果
│   ├── image10.jpg//相机标定用到的图片
│   ├── image1.jpg
│   ├── image2.jpg
│   ├── image3.jpg
│   ├── image4.jpg
│   ├── image5.jpg
│   ├── image6.jpg
│   ├── image7.jpg
│   ├── image8.jpg
│   └── image9.jpg
├── CMakeLists.txt
├── include
│   ├── DEBUG.h//存放一些DEBUG用宏定义
│   ├── head.h//主要功能函数头文件和装甲板结构体封装
│   ├── params.h//存放主要参数
│   ├── serial.h//已弃用
├── main.cpp
├── platform//传感器API（暂未使用）
│   ├── inc
│   └── src
├── README.md
└── src//源文件
    ├── calibration.cpp//相机标定，参考https://blog.csdn.net/huangshulang66/article/details/78219363
    ├── color_picker.cpp//已弃用
    ├── distancedetection.cpp//测距和重力误差消除
    ├── getTarget2dPosition.cpp//将识别到的装甲板角点位置按顺序重新排列，便于后续处理
    ├── ImgPreProcess.cpp//图像预处理
    ├── kalman_filter.cpp//卡尔曼滤波预测装甲板中心点的位置
    ├── objClassifier.cpp//识别和筛选图像中的灯条，两两匹配成完整的装甲板并确定最佳的打击目标
    ├── position_adjust.cpp//已弃用
    ├── serial.cpp//串口通信相关函数定义
    ├── threads.cpp//子线程函数定义,包括串口发送，接受线程，图像获取线程
    └── vl53l0x_ContinuousRanging_Example.c//传感器示例，暂未使用
```

## 四.程序时序流程图

![流程图](https://github.com/for-all-fistance/test_code/blob/master/pictures_and_videos/timeline.pdf)

# 五.主要代码原理：

- 图像预处理：
    
    1.对相机的曝光时间进行调整，曝光时间调整为20，该过程使用lLinux上的qV4L2工具完成，未在代码中体现。
    
    ```cpp
    //使用以下命令可以下载qv4l2工具
    sudo apt install qv4l2
    ```
    
    2.由于装甲板的颜色为蓝色，故将图像进行通道分离，将蓝色通道减去红色通道得到最好的成像效果。
    3.之后对图像进行阈值化，阈值化准备有两套方案，手动设定阈值和自适应阈值化，手动调参鲁棒性较差，而OTSU法自适应阈值化在背景光比较强时分割效果很差，且增加了时间开销，需要根据具体情况考虑选择哪种方案。
    
- 装甲板检测与筛选：
    
    筛选装甲板灯条的条件有：轮廓面积（可以筛除一些小干扰点）轮廓最小外接矩形的长宽比，轮廓最小外接矩形和轮廓面积的比值，轮廓的凸度（最小外接凸多边形与轮廓面积比值）
    
    灯条匹配的条件：两个倾角相差最小的灯条匹配为一个完整的装甲板。
    
- 卡尔曼滤波器预测目标运动轨迹：
    
    卡尔曼滤波的原理参考：[【学习笔记】卡尔曼滤波超详细推导和理解举例（以RoboMaster目标预测为例）_安河桥北以北的博客-CSDN博客_卡尔曼滤波适用条件](https://blog.csdn.net/Fosu_Chenai/article/details/113112833)
    
    [blog](https://www.researchgate.net/publication/356209829_jiyukaermanlubodemubiaoshibiegenzongyushejixitongsheji?utm_oi=1333859106986459136)
    
- pnp测距和重力误差消除：
    
    使用OpenCV中的solvepnp（）通过装甲板的四个角点解算旋转向量rvecs，经过rodrigues变换获得旋转矩阵，根据以下公式可以算出三维点坐标信息。
    
    ![公式1](https://github.com/for-all-fistance/test_code/blob/master/pictures_and_videos/equation1.png)
    
    得到距离后由简单的物理知识可以得到重力引起的误差。重力误差体现在图像上可以由以下公式近似计算(其中H1为装甲板实际高度，hscreen为装甲板在图像上的高度):
    
    $Δyscreen=ΔH*H1/hscreen$
    
- 角度计算
    
    角度计算公式：
    
    $$
    tanθx=Z/X=(xscreen−cx)/fx
    
    $$
    
    $$
    tanθy=Y/X=(yscreen−cy)/fy
    $$
    
    $$
    θx​=arctan(tanθx​)
    $$
    
    $$
    θy​=arctan(tanθy​)
    $$
    

## 六.通信协议

**pi—>stm:**

N: 没有检测到目标。

x±12@y±12@ 目标角度。由于电控无法精确控制云台和底盘旋转的角度至一度以下，传输时直接截断成int类型即可，角度不足两位在前方补0。

**stm—>pi :** 

&：表示开始识别打击符文板 。（暂未使用，后续可能考虑将该部分改为打击静止的装甲板，&打击移动的装甲板，并设置两套不同的参数以适应不同的场景）

%：表示开始识别打击装甲板.

 ~ ：表示待机

# 七.目前代码仍存在的问题和未来的优化方向：

1. pnp测距精度不高，后续会考虑使用激光测距模块提高测距精度。
2. 读取图片时间开销太高（摄像头仅有30帧），考虑将读取图片作为子线程独立出去，防止阻塞主线程，或者使用帧率更高的摄像头。（已使用子线程的方式，但性能提升不明显且锁的竞争极为繁复,甚至导致程序难以自发结束，降低了代码的可读性）
3. 考虑到装甲板的运动速度并不快，可以在检测到装甲板后，取装甲板周围一定区域为ROI，以减少图像处理的时间开销。此外为了防止跟丢，需要在一定帧后对全图进行一次检测。
4. 可移植性差，需要做大量的更改才能在其他机器上运行
5. 变量使用不规范（太懒了不想改）。
6. 筛选装甲板的条件设置不合理，当环境干扰较强时，容易误识别。

## 八.程序运行效果

**一.运行速度**

程序整体基本可以稳定在30帧左右。

**二.识别率**
对于静止物体识别率超过95%，对于运动的物体识别率也在90%以上。
![测试视频](https://github.com/for-all-fistance/test_code/blob/master/pictures_and_videos/test%20result.mp4)

# 九.Notes

- 由于比赛规则临时更改，且截至此时官方并没有最终敲定比赛规则，故在代码中保留了大量预编译指令和未优化的代码，以确保随机应变和完整的呈现代码框架。
- 受到本人代码水平的限制，代码并没有采用OOP的编写方式。

**作者：碱基互补配队  自动化科学与工程学院 21级 李佳梁**