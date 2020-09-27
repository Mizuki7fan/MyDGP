# 0.前置工作
## 1.1 Cmakelists.txt
* [x] 通过option指令来设置使用openmesh/cgal/自建数据结构
* [ ] 通过option指令来设置选用的solver
* [ ] 兼容linux

## 1.2 MyMesh父类
* [ ] 将三种网格数据的方法进行封装，成为统一的类
* [x] 封装openmesh的子类
* [ ] 封装cgal的网格类
* [ ] 封装自建的网格类
* [ ] 尽可能使用迭代器

## 1.3 CGAL网格类处理
* [ ] 学习使用CGAL

## 1.4  自建半边结构
* [ ] 建立方法按照第一课的视频上所说的来
* [ ] 兼容各种格式的网格的读取
* [ ] 尝试用迭代器和运算符重载完成各种功能

## 1.5添加选点
* [ ] 使用kdTree

## 1.6贴纹理
* [ ] 明白框架中的OpenGL指令

## 1.7 MeshParamWidget
* [x] 左侧面板重做

# 1.Discrete Differential Geometry
## 1.1三种局部平均区域
* [x] 求取面积

## 1.2 Normal Vector 法向
* [ ] 展示面法向信息
* [ ] 展示多种方式定义的点法向信息 
 
## 1.3 Barycentric coordinate 重心坐标

## 1.4 梯度
* 平面中任意一点的梯度不变
* 如何计算？

## 1.5 Laplace-Beltrami Operator 离散化的拉普拉斯算子
* No Free Lunch，连续场合满足的5个性质，最大值原理的含义？
* 面/边上的拉普拉斯算子，不关心
* 顶点上的拉普拉斯算子
* 求对称迪利克雷能量
* 常见的离散化拉普拉斯算子的构造和推导

## 1.6 离散曲率
* [x] 求取平均曲率
* [x] 求取绝对平均曲率
* [x] 求取高斯曲率
* [ ] 适当的方式进行可视化
* [ ] 确定正确性

# 2.Smoothing

## Laplacian
* uniform和cotangent
* 如何快速构建uniform？
* 显式/隐式的欧拉积分