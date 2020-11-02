# RoboticsToolbox ver.C++
### by YangHaixin, SCU, 941705864@qq.com
![](https://img.shields.io/badge/Language-C++-blue.svg)
![](https://img.shields.io/badge/Ubuntu-18.0.4-orange?style=flat&logo=Ubuntu&logoColor=ffffff)
![](https://img.shields.io/badge/Editor-VS%20Code-blueviolet?style=flat&logo=Visual%20Studio%20Code&logoColor=ffffff)
![](https://img.shields.io/badge/Based-Eigen-red?style=flat&logo=Matrix&logoColor=ffffff)
## 前言
作为一名研究机器人方向的研究生，我最近在进行C++的学习，但苦于没有专业项目练手，故考虑使用C++对Matlab上的RoboticToolbox进行移植，一来是为了锻炼自己的C++能力，二来也顺便复习了机器人知识，希望自己能够坚持下来吧！

**在我完成这个库之前，本文档是我的开发日志和心得体会**

## 开发日志 
### 2020.10.22
开始了移植工作，此时离我完成C++大致学习过了３天，目前将会使用到`Eigen`这个库来帮助我进行矩阵运算。今天因为有课，还有做项目的关系，只写了个欧拉角求旋转矩阵的函数`EulerRot`。

### 2020.10.24
这两天断断续续地在写，因为做实验花了很多时间。之前还在思考如何用类、设计模式来设计我的这个机器人库，后来发现这样有点太浪费时间了，不如先以函数的形式写好大部分功能，然后再系统的学习一下设计模式，再开始重构一下代码，合理OOP。新增了以下功能：

- `Skew`:求出一个三维向量对应的反对称矩阵，也称为叉乘矩阵，该矩阵可用于向量的叉乘运算:$A\times B=[A]B$，其中$[A]$表示将向量$A$转为反对称矩阵形式；

- `RotX, RotY, RotZ`:绕单坐标轴旋转的旋转矩阵；

- `SE2, SE3`:分别为二维和三维的欧几里得群(Special Euclidean group)，即齐次变换矩阵，目前采用的还是**欧拉角旋转+位移向量**的形式求出齐次变换矩阵，*待完善*；

- `RotMatExp`:矩阵指数的方法求出旋转矩阵，其实就是指定一个在被旋转的坐标系下表示的旋转轴向量，然后指定一个绕该轴旋转的旋转角度，即可求出旋转矩阵；

- `GetRotAxis`:矩阵指数的反运算，矩阵对数。由旋转矩阵反解出矩阵指数所需的旋转轴向量，该向量是在原坐标系下表示的；

- `GetRotTheta`:矩阵对数，求出绕某一轴旋转的角度，该函数需要配合上一个`GetRotAxis`函数使用，目前没想到怎么整合它们两个，*待完善*。

今天就到这里了，周末在家果然不适合学习，再加上导师最近也安排了一些控制理论的学习任务，还是要平衡一下工作节奏。

### 2020.10.27
今天比较忙，上午才慢悠悠返校，下午批改了数据库的作业，顺便学习了一下SQL Server的用法，感觉可以使用数据库存放机械臂信息，然后调用相应数据来进行运动学、动力学计算。今天就只有晚上稍微写了一下代码，还搞忘了一些概念，看了会儿书。

**今日更新**

- `AdjMapMat`:根据齐次变换矩阵求出对应的坐标系转换关系下的伴随变换矩阵，该矩阵主要用于运动旋量的坐标转换；

- `SE3Twist`:根据螺旋轴信息来求解齐次变换矩阵，现在有个问题就是如何准备描述一个螺旋轴？因为之前学习的时候都是直接将这个方法带入了机械臂正运动学中，没有单独进行螺旋轴描述过，所以在测试的时候就有点不知所措，不过目前看来程序是没有问题的；

**Debug**
- 之前的`RotMatExp`函数忘记了将旋转轴化为单位轴，现在是增加了一个判断语句来进行归一化。
- 优化了齐次变换矩阵元素挨次赋值的循环语句，将一些语句放在了外侧循环中，减少了语句的循环次数，虽然这是一个尝试，不过也是之前没有注意到的细节啊！

睡觉。

### 2020.10.27
今日新增：
- `GetTwist`:由齐次变换矩阵求出其对应的螺旋轴
- `GetTwistTheta`:解出螺旋轴对应的旋转角度

今天完成的都是一些非常简单的功能，不过已经大致把刚体运动的部分写完了，接下来就是写机器人运动学的相关内容了，目前正在做的是一个建立机器人DH模型的类`RobotDH`，但是现在这个类有重大bug，且会干扰到`SE3Twist`，使其解算结果莫名其妙的错误，让人焦头烂额。

和一个师兄交流后，他指出了我的代码中很多不合理的地方，比如构造函数太复杂了、变量在类里只声明不要定义，我也觉得我的构造函数过于复杂了，可能会引起一些莫名其妙的问题，影响到了`SE3Twist`，现在这俩还不能一起使用，我接下来打算先去看看别人的代码，学习一下规范构造这些，然后将这个机器人类优化一下，再测试测试。

看代码去咯。

### 2020.10.28
**还是没解决**

尝试解决昨天使用`RobotDH`类时，会干扰了`SE3Twist`的问题。该问题的现象就是：使用这个类时，根据类声明的位置的不同，会使得`SE3Twist`解出来的矩阵数值不同，起先我怀疑是不是使用了动态大小的Eigen库函数导致的，将类中的所有成员注释后，修改了`SE3Twist`中使用了动态大小的地方后，问题得到了暂时的解决，但是当我取消掉类成员的注释后，问题又出现了！

在查阅了一些资料后发现，Eigen库的**内存对齐**问题一直受人诟病，内存对齐这一块我也不是很懂，今天就再去看看相关资料吧。然后在eigen的文档网站发现[在类中使用Eigen](http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html)的方法。如果要在类里面调用Eigen的函数，则必须重载` operator new`，以便它生成16字节对齐的指针，只需要在类的public声明中加上一句宏定义`EIGEN_MAKE_ALIGNED_OPERATOR_NEW`，然后就可以在类中使用eigen的函数来定义变量了。但是该方法是针对静态大小，对我的问题也不适用。

然后晚上无意中在StackOverflow中发现了一个关于`<<`的重载问题，然后我就发现我的代码出问题的地方就涉及到了好几个`<<`，我就怀疑可能是迷之重载出了问题，于是就把构造函数中给向量赋值用的`<<`**全部删除**，换为挨个赋值，他妈的有时候可以有时候又不行了？调不来了，休息了

### 2020.10.29
心态炸裂，昨天那个机器人类，如果我把所有的成员属性在构造函数里进行赋值，那个问题又出现了！

**晚上成功解决该bug，被自己蠢哭了**

没错，这个神秘bug解决了……我之前gdb调试的时候，一直没有进入if语句，我还以为是我用`next`的操作不对，今晚无意中发现，马勒戈壁，if的条件写错了！！！！！！！！怪不得`SE3Twist`的输出值经常是随机数！因为没有赋值！把if的条件修改后，成功解决该bug，`RobotDH`类是无辜的，错怪它了，唉！被自己蠢哭了，不过成功debug，还是挺爽的。

在本次Debug的过程中，通过查询Eigen文档、百度、StackOverflow，学到了以下知识：
- 内存对齐：计算机读取数据时，并不是一个字节一个字节的读，而是直接读一大片内存，如果不使用内存对齐，比如一个４字节的变量，前３字节在第一次读的那一片内存里，此时需要舍弃第一个字节，后１字节在另一片内存，需要再读一次，然后舍弃三个字节，就很麻烦，所以将各个类型的变量进行内存对齐，计算机就可以一次性读取数据了
- GDB调试：`s`是进入函数内部，`n`是直接执行完这个函数，`-tui`或者`Ctrl + X + A`可以启动图形化界面，方便知道当前执行到哪了
- Eigen作为函数形参，进行值传递时，官网建议使用`const`的形式来进行值传递，我已经将之前的函数进行了修改
- 变量的声明和定义写近一点，声明矩阵变量后，注意定义的时候是否有没有赋值的元素？建议矩阵在声明的时候直接定义成0矩阵
- 遇到问题不要慌着认为是库的问题，检查一下自己的**语法**和**逻辑**

除此之外，今天还学习了怎么在markdown的readme中，增加图标功能，我在本文档的开头增加了语言和更新日期的图标，非常简单，需要使用[Shield.io](https://shields.io/category/version)网站，来生产一个代码：
```
![](https://img.shields.io/badge/language-C++-orange.svg)
```
其中，badge后面就是标签的头、尾、颜色了，非常简单。还可以利用[Simple Icons](https://simpleicons.org/?q=linux)网站来获得图标名字，直接增加一个logo项，即可增加图标了
```
![](https://img.shields.io/badge/Linux-Ubuntu-orange?style=flat&logo=Linux&logoColor=ffffff)
```

### 2020.10.30

**MakeFile**

简单写了一下Makefile，直接在其路径下使用`make`命令即可自动运行Makefile，自动编译，对于没有修改的文件还可以不重复编译，比我之前直接g++ -o xxxxxxxxxx快了一些，方便得多了，以后增加了文件只需要修改一下Makefile即可，如果要增加GDB调试，则需要在可执行文件链接的那一句在文件名之后再加上-g
```makefile
bin/robot: obj/main.o obj/BodyMotion.o obj/RobotBuild.o
	g++ -o bin/robot -g obj/main.o obj/BodyMotion.o obj/RobotBuild.o 

obj/main.o: src/main.cpp src/BodyMotion.hpp src/RobotBuild.hpp
	g++ -c -g src/main.cpp -o obj/main.o

obj/BodyMotion.o: src/BodyMotion.cpp src/BodyMotion.hpp
	g++ -c -g src/BodyMotion.cpp -o obj/BodyMotion.o

obj/RobotBuild.o: src/RobotBuild.cpp src/BodyMotion.hpp src/RobotBuild.hpp
	g++ -c -g src/RobotBuild.cpp -o obj/RobotBuild.o

```

### 2020.11.02

周末去重庆找女朋友玩了。今天上午把以前的笔记本拿去修了，下午做实验，晚上稍微写了点。

**今日更新**

- `RobotTwist`类，通过旋量来构建机器人模型，该方法比DH参数法更优越；
- `RobotTwist::FKTwist`函数，该函数是在旋量机器人模型类中的成员函数，可以用来计算基于指数积模型的正运动学，目前验证是正确的；
- 修正了之前代码无用的`static`声明，因为我不会使用局部变量地址、引用来作为返回值，故不需要。

更新的有点少，精力有限，慢慢来吧。

**学习总结**

- Eigen库的可变大小类，如`VectorXf`，在赋值之前，需要对其大小进行声明，否则容易出错；
- 对`Vector`类进行截取时，不能像Matlab那样直接V(i, j)，而是要使用segment函数`omega.segment(3*(DoF - 1 - i), 3)`截取omega的3*(DoF-1-i)开始的３个元素。
