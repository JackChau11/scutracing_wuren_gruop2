# 更新日志  

## Version 3:
### 创建者：朱子远  
### 更新时间：  
2026.6.15 2：16  
### 更新内容：
1.将适配gazebo classic的仿真代码改写为适配gazebo harmonic版本。
2.修改了车端插件，将自定义的C++ StdMsgVehiclePlugin 改为Harmonic 内置 AckermannSteering，接口不变（都是 /cmd_vel Twist）。
3.修正转向连杆惯量三角形不等式不正确导致的dartsim 拒绝加载车辆bug
4.仿真启动文件 — 用 ros_gz_sim 启动 gz sim，7 个 *ros_gz_bridge* 桥接全部话题，NVIDIA 渲染配置，将TF坐标系名从**world** 转为**map**（替换旧 Gazebo Classic 启动）
## 存在问题
1.小车现在无法跑直线
2.小车的启动只能靠手动发目标点让车开始导航

## Version 2:
### 创建者：周宇扬  
### 更新时间：  
2026.6.14 20：30  
### 更新内容：  
1.基本完成仿真、建图、规划、控制全流程  
2.相比Version 1,改进了规划和控制的部分。规划的路径搜索使用A*算法，控制使用纯跟踪  
3.更新了车辆模型（从网上找的），自己做了改动使其符合控制部分的需要（本来是跟着鱼香ROS的教学做了一个靠差速控制转向的三轮小车，但是当把它改为4轮时无论是前驱还是后驱都无法实现转向，遂暂时放弃）  
4.没有使用车队给的感知节点，而是自己跟着别人的项目手写感知节点，效果好像还可以）    
###存在的问题和需要改进的地方：   
1.最大的问题是本人的电脑由于硬件原因无法在Gazebo Harmonic中使用激光雷达，所以退回Gazebo Classic，需要大家把它桥接到Harmonic上  
2.A*算法表现不佳，无法一次过让赛车完成整个直角弯的行驶，会切弯  
3.纯跟踪做得又蠢又慢  
4.需进一步调整激光雷达的设置，现在赛道的锥桶在目前的设置下检测得不是很好，Rviz2里看太小了  
### 参考资料：  
https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f  
https://fishros.com/d2lros2/#/humble/chapt5/get_started/6.%E5%85%BC%E5%AE%B9%E4%BB%BF%E7%9C%9F%E5%B7%A5%E5%85%B7-Gazebo?id=_1gazebo-vs-rviz2  

## Version 1:  
### 创建者：周宇扬
### 更新时间：  
2026.6.13 11：30  
### 更新内容：  
1.完成了基本的仿真环境搭建    
2.完成了车辆URDF建模，赛车和赛道在Gazebo和Rviz2中显示基本正常  
3.基本完成了建图、规划、控制的部分（使用slam_toolbox建图， Nav2规划和控制）  
### 存在的问题和需要改进的地方：  
1.TF坐标树断裂，四个车轮无法连接到map和world  
2.临时使用键盘对车辆进行控制，但是无法使车辆运动（已排除/cmd_vel话题的问题，成功echo该话题发布的信息）  
3.能够获取雷达发布的信息，但Rviz2中无法可视化点云，导致建图、规划、控制无法进行  
