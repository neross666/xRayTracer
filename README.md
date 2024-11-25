# 现在计算机图形渲染学习记录
包括：
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
	- VolumePathTracingNEE：体渲染积分器
- 结果图像存储于显示


# Reqiure
spdlog
OpenCV
OpenVDB
