### 修复 PCL 中 pcap/png disabled 的错误提示

>   主要记录系统 PCLConfig.cmake 未正确查找 pcap/png，导致 CMake 提示功能被禁用的问题。

#### 问题描述

>   编译包含 PCL 的工程时出现以下提示：
>
>   ** WARNING ** io features related to pcap will be disabled  
>   ** WARNING ** io features related to png will be disabled

表现：

-   构建完成但 stderr 有上述 warning
-   pcap/png 相关 I/O 能力被禁用

#### 问题原因

>   系统的 `PCLConfig.cmake` 在处理外部依赖时，没有对 `pcap/png` 执行 `find_package`，导致 `PCAP_FOUND/PNG_FOUND` 始终为 false，触发禁用警告。

#### 解决方案

>   修改系统 `PCLConfig.cmake`，在 `find_external_library` 中加入 `pcap/png` 的查找。

定位文件：

```
locate PCLConfig.cmake
/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake
```

定位到以下片段：

```
elseif("${_lib}" STREQUAL "vtk")
  find_VTK()
```

在其后插入：

```
elseif("${_lib}" STREQUAL "pcap")
  find_package(Pcap)
elseif("${_lib}" STREQUAL "png")
  find_package(PNG)
```

完整结构应类似：

```
elseif("${_lib}" STREQUAL "vtk")
  find_VTK()
elseif("${_lib}" STREQUAL "pcap")
  find_package(Pcap)
elseif("${_lib}" STREQUAL "png")
  find_package(PNG)
elseif("${_lib}" STREQUAL "libusb")
  find_libusb()
```

#### 验证方式

>   重新构建工程，确认 stderr 中不再出现 `pcap/png will be disabled`。

建议步骤：

```
colcon build --symlink-install --cmake-clean-cache
```

#### 备注

>   系统包更新后此改动可能被覆盖，需要重新修改。
