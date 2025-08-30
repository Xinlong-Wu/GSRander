# GSRander - 高斯点云渲染器

这是一个使用C++、Vulkan、PCL和Eigen编写的高斯点云渲染器。

## 功能

- 读取高斯点云文件
- 使用Vulkan渲染点云
- 基于PCL和Eigen进行点云处理

## 依赖

- Vulkan SDK
- PCL (Point Cloud Library)
- Eigen3
- CMake 3.10+

## 构建

```bash
cmake .
make
```

## 使用

```bash
./gsrander <point_cloud_file>
```

## 文件格式

当前实现假设点云文件包含每行一个点的文本文件，格式如下：

```
<x> <y> <z> <r> <g> <b>
```

其中(x,y,z)是点的位置坐标，(r,g,b)是颜色值（范围0-1）。

## 注意

当前Vulkan渲染部分仅为占位符实现。要获得完整的渲染功能，需要进一步实现Vulkan相关的代码。