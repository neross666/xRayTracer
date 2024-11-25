# 现代计算机图形渲染学习记录
## 主要内容
使用C++实现基于蒙特卡洛路径积分的真实感渲染引擎
- 相机，目前只实现小孔成像模型
- 灯光
	- delta光源
		- 点光源
		- 面光源
	- 面光源
		- 三角形光源
		- 四边形光源
		- 球形光源
- 几何物体[tinyObjectLoad加载]
- 材质
	- 理想漫反射材质
- 场景，集成几何物体、灯光等
- 渲染器
	- CPU串行
	- CPU并行
- 积分器
	- NormalIntegrator：常规积分器，渲染可见性、面照比、熔炉测试
	- DirectIntegrator：直接光照积分器
	- IndirectIntegrator：间接光照积分器
	- GIIntegrator：直接+间接光照积分器
	- WhittedIntegrator：Whitted风格积分器
	- VolumePathTracing：体渲染积分器
- 结果图像存储于显示

## 部分效果图
![c82ee24afb0070a70fbd047e75efc32](https://github.com/user-attachments/assets/e57f970b-5423-402a-b245-bdb5512a45b6)

![80777892e5676e27fa57112e241f3ae](https://github.com/user-attachments/assets/0a999e3e-3a6d-4c75-893b-10152454bfc7)

![00e839cc3d7fb0ba0c4faaa67f144fc](https://github.com/user-attachments/assets/58f0bac9-b542-4c84-8e5c-37425b7e0e9a)

![03c24569943ad1fc328ed98fafaee28](https://github.com/user-attachments/assets/8ee1bdaa-2d0b-40bd-82ae-873283fde240)

## 依赖库
- spdlog
- tinyObjLoader，用于加载obj文件
- OpenVDB，用于加载体数据
- OpenCV，仅用于显示最终渲染图
