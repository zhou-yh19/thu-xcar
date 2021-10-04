# THU Xcar Project

### 文件框架

* **xcar-main**包含xcar的launch文件
* **xcar-hardware**包含xcar的硬件package
	- vesc
	- imu
	- lidar
	- ...
* **xcar-control**包含xcar的控制程序
	- **control-v1**是第一个版本的漂移控制程序，包括julia仿真和实车实验的程序


### 运行方式

* nvidia jetson tx2

1. `ssh nvidia@192.168.31.17`远程登录xcar
2. `roslaunch racecar auto.launch`启动xcar的ros各个node

* caslx server
1. `ssh yangbo@192.168.31.60`远程登录server
2. `jupyter-lab`启动jupyter
3. 运行**/control-v1/xcar**里的notebook文件
