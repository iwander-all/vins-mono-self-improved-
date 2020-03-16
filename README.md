# vins-mono(self-improved)
an improved version of vins-mono

I: OK, now, let me introduce to you an improved version of ...

you: Hey,hey,these seem like, well, how to say, exactly the same as the original masterpiece VINS-Mono, so what have you done man???

I: OK, admittedly, I do not change the ideas, the variables and the functions...

you: hahahahahaha,xixixixixi,233333333...

I: I establish include for each packages and put all the .h into them.

you:oh-hoh?

I: And when I read the original codes, I think the main vio functions, estimator.cpp, they are too long! Maybe it is not easy for new-comer to figure out the strcture.

So I extract codes out of some functions like initialStructure(). optimization()...

And,I put the steps initialization and backend-optimization out of estimator.h/cpp into new files initial.h/cpp and backend.h/cpp and form 2 new classes: class Backend and class Initial.

As you can see, their are much fewer lines in estimator.cpp, from 1200 to 220 lines. There are only main processes in the .cpp file. So the structure is much clear.

The official link is:
https://github.com/HKUST-Aerial-Robotics/VINS-Mono

To run this project is exactly the same as the original one, take it easy. Put all the files above into src, and put src into catkin_ws.

Open terminal, type in "catkin_make" and drink a cup of coffee.

Then, open one terminal, type "roscore", of course you have to install ROS in advance.

And open 2 terminals and in the path of devel,type "source setup.bash", then fill in:

"roslaunch vins_estimator euroc.launch"

"roslaunch vins_estimator vins_rviz.launch"

Then another ternimal with:

"rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag" 


Good time. And work have not done for me yet. I will develop a dual-vio based on vins-mono, not on vins-fusion.

you: hahaha, wait for good news!


4.流程

4.1 调整结构

修改目的：把主流程中的代码全部封装出去，从而提升逻辑的清晰性，代码的可读性和代码的模块化，另一方面方便对某一个模块或者函数进行整体性的修改或重写。
C++注释规范 https://www.cnblogs.com/aspiration2016/p/8433122.html 

需要注意的是，有很多局部变量，构造新函数的时候，需要传参。

这一部分已经完成，放在catkin_ws1中。

4.1.1 每个package增加include文件夹，把hpp文件都放进去

(1)注意修改h文件开头声明：#ifndef，#define，#endif。

(2)注意修改#include路径。

刚开始，h和cpp文件在同一个目录下，目标是在每一个package下面新建一个onclude文件夹，然后把h文件放进去，就像很多别人家的工程一样。需要注意的是，每一个cpp文件的include路径都需要根据相对路径的方式进行修改。


4.1.2 把初始化，非线性优化等各个步骤封装成函数，从主流程中提出来

(1)feature_tracker.h/cpp
 
把主流程函数readImage()里的 直方图均匀化、光流追踪、新特征点提取 这三个流程封装成为三个独立的成员函数，已经在h和cpp中更改；

(2)estimator_node.cpp
 
主要修改process()主线程；

这个线程主要包含3步：测量量的提取和对齐；对测量量的操作(vio主流程)；状态量的更新；

测量量的提取和对齐：没有进一步封装；

vio主流程：封装为processMeasurement(),然后进一步封装了processIMU(), setReloFrame(), processVIo(), visualize()。

update()没有进一步封装。

(3)estimator.cpp——processImage()
 
这个是vio主函数，里面主要包含了5个功能：

a.判断marg_old还是marg_new并将来自feature_tracker_node的特征点放入f_manager中；

b.创建数据结构all_image_frame服务于初始化；

c.标定camera->IMU的外参(封装为CalibrationExRotation()函数)；

d.初始化(封装为initial()函数)；

e.后端非线性优化(封装为backend()函数)；

后续，我会把初始化和后端从estimator.h里面拿出来放在一个新的h文件里，创建新的类。

(3)estimator.cpp——processImage()——initial()——initialStructure()

initial()这部分的主流程代码已经很清晰了，不用再封装，对其中的initialStructure()进行封装；

initialStructure()可以分成6步：

a.判断IMU是否有足够的运动激励，封装为checkOMUObservibility()函数；
b.创建数据结构sfm_f用于SfM过程，封装为buildSFMFeature()函数；
c.寻找滑窗里的第l帧，它与最新帧有足够的视差，求帧号l和最新帧到第l帧的 旋转平移；
e.SfM
f.对所有帧用PnP来求解他们在l帧上的位姿，封装为solvePnPForAllFrame()函数；
g.IMU和SfM粗耦合联合初始化visualInitialAlign()；
h. visualInitialAlign()包含两步，第一步就是IMU与视觉的粗耦合求各个待优化的状态量，第二步是把所有帧的位姿和landmarks的坐标转移到w系上，这部分封装为recoverStatusValuesFromInitial()函数中。

(4) estimator.cpp——processImage()——backend()——solveOdometry()——optimization()

optimization()是estimator.cpp中最长的单一函数了，我第一次看的时候总是陷入到单行代码中而没弄清楚总体流程。实际上这个函数主要干了2件事：

a.非线性优化，这部分封装为nonLinearOptimization()函数；
b.边缘化，其中marg_old封装为margOld()函数，marg_new封装为margNew()函数。
封装完成后，函数代码从原先的300行压缩到20行，能够一眼看出来函数逻辑和结构，也方便以后对它的模块化更改。

4.1.3 给hpp文件中各个成员变量和函数全部增加注释

4.2 把backend部分全部提取构造成类

这项工作是基于4.1的工作基础上的。在4.1，完成了对不同函数的进一步提取和所属功能的分类，现在，对vins_estimator文件夹下的estimator.h/cpp文件进行进一步操作，把属于后端优化的函数和变量全部提取出来，创建一个新的backend.h/cpp文件，并创建Backend类。不过，还有一种更简单的方式，就是新建h/cpp，但是不创建类，直接放函数。

这一部分已经完成，放在catkin_ws2中。

(1)构建backend类

由于初始化会用到优化部分的代码，所以先构建backend类。
如果新建一个h/cpp文件，会涉及到很多函数和变量的迁移和引用。考虑到后端会用到一些estimator.cpp的通用函数，所以需要传入estimator的this指针；
考虑到后端的作用是对状态量的更新，所以对backend类中需要只输入和输出状态量，而辅助的变量可以从estimator迁移到backend中去。

这里有一个非常麻烦的地方，就是estimator和backend这两个文件/类之间存在相互include/调用的情况。另一方面，对于迁移的变量，需要把estimator中对这些变量的操作也迁移过来，对于没有迁移的变量，需要在这里进行指针操作。攻略见：
https://blog.csdn.net/xiqingnian/article/details/41214539?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task
 
(2)步骤

a.把后端相关函数和变量提取出来放到backend.h/cpp中；
b.把estimator.h和backend.h都要#include的头文件从estimator.h拿出放到backend.h里面；
c.backend.h中，不可以#include“estimator.h”；
d.backend.h中，在class Backend前面需要声明class Estimator，在class Backend内需要创建Estimator *estimator，注意，这个是指针；
e.backend.cpp中，需要同时include“estimator.h”和“backend.h”；
f. estimator.h中，需要include “backend.h”，在class Estimator前，需要声明class Backend,在class Estimator内，需要创建Backend backend，注意，这个是实例。
g.estimator.cpp中，需要include“estimator.h”，并且凡事在使用后端函数的时候，都需要传入this指针，这是由Backend的成员函数的定义确定的。
h.vins_estimator的CmakeLists.txt中的add_executable添加backend.cpp

(3)理由

class Estimator是主类，内部包含backend对象，创建backend成员变量时遇到一个问题，如何确定给backend这个数据成员分配多少内存呢？我们知道backend也需要使用estimator的方法，如果在backend内部使用的是estimator的指针，这样即能实现对estimator的调用，也能确定backend所需的内存，因为指针的大小是确定的，4个字节。此时，estimator创建的时候，知道给backend分配多少内容，那么就能实现正确创建。对于class Backend而言，因为一直使用的是estimator的指针，所以创建他的时候，也能确定自己所需要的内存。
所以，在Backend类中，一律创建和使用estimator的指针，而在Estimator类中，可以创建一个backend对象。

4.3 把inital部分全部提取构造成类

这项工作是基于4.2的工作基础上的。在4.2中，把所有的后端提取出来了。之所以先提取后端，是因为初始化步骤用到了后端的内容，在完成一次初始化后，紧接着跟着一次非线性优化和滑窗。所以，仅仅把初始化部分的initialStructure()提取出来放在initial.h/cpp中，而没有必要把后面的优化和滑窗操作也放进来，否则在initial.h/cpp中要传入backend和estimator两个大类的指针，这是没有必要。

(1)构建inital类

在上一步的基础上，这个操作就很容易了。这一部分已经完成，放在catkin_ws3中。完成后，estimator.cpp文件从1200行减少到220行。


