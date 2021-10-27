# THU Xcar Project

### 文件框架

* **xcar-main**包含xcar的launch文件
* **xcar-hardware**包含xcar的硬件package
	- vesc
	- imu
	- lidar
	- ...
* **xcar-simulation**包含xcar仿真平台，支持同步和异步仿真
	- 同步仿真：直接在Julia代码中调用仿真函数
	- 异步仿真：运行一个ROS节点，模拟真车的接口
* **xcar-control**包含xcar的控制程序
	- **control-v1**是第一个版本的漂移控制程序，包括julia仿真和实车实验的程序


### 运行方式

* nvidia jetson tx2

1. `ssh nvidia@192.168.31.17`远程登录xcar
2. `roslaunch xcar auto.launch`启动xcar的ros各个node

* caslx server
1. `ssh yangbo@192.168.31.60`远程登录server，密码:`yangb21`
2. `conda activate ros`启动环境
2. `jupyter-lab`启动jupyter
3. 运行**xcar**里的notebook文件
