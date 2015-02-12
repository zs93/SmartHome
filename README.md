# SmartHome
based on CC2530 and Mini2440
#终端程序的使用
基于z-stack 2.51a 版本
##demo的使用
demo是针对串口，网段修改，去掉LCD的显示

具体修改地方请查看 demo目录下的README
>这个ZIGBEE的demo就是仅仅是在z-stack version 5.21a 版本上做的非常小的一点修改，给予那些希望直接使用z-stack version 5.21a的人进行使用

##termial的使用
在demo基础上的修改，添加了自动安全组网（该原理的链接：[点击这里](http://www.cnblogs.com/samuelwnb/p/4250859.html)）然后可以对使用想要自己定义的IO口和功能的人使用！
>具体修改地方请查看 termial目录下的README

##termialv2的使用
在termialv2中，已经把所有的可以使用的IO口做了详细的划分！将相应的IO口都做了很好的分析
>具体修改地方请查看 termialv2目录下的README

#终端电路图
在PCB文件夹中，详细介绍了硬件电路图的制作
#ARM控制台
arm控制台基本成型，在本目录下的server目录下，具体使用方法也是该目录下的README文件中！
#手机客户端
正在编写中
#问题与反馈
如果您对我的项目非常感兴趣的话，想与我一起参与我的项目中，那么请您发电子邮箱：188101696@qq.com与我取得联系！
##现存在的问题
* IAR在编译过程中，协调者中的definded symbols中 DEV_COOR 还是无法有效，期望有对IAR研究的朋友能够帮忙解决！