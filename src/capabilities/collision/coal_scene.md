# Coal Scene 浅封装使用说明

## 文件位置

浅封装入口：

```text
src/capabilities/collision/include/collision/coal_scene.hpp
src/capabilities/collision/src/coal_scene.cpp
```

业务代码优先 include 这个封装：

```cpp
#include "collision/coal_scene.hpp"
```

不要直接在业务层散用 Coal 内部头。当前 `third_party/coal` 已经被裁剪成 vendored core，`CollisionScene` 才是本项目维护碰撞场景、检查碰撞、Foxglove 可视化的稳定入口。

## 能做什么

`collision::CollisionScene` 当前提供三类能力：

1. 维护碰撞场景
   - `upsertObject(...)` 添加或更新对象
   - `removeObject(...)` 删除对象
   - `clear()` 清空场景
   - `setPose(...)` 更新对象位姿
   - `setEnabled(...)` 启用或禁用对象
   - `ids(...)` 查询对象 id

2. 检查碰撞
   - `checkSelfCollision(...)` 只检查 `Self` 对象之间的碰撞
   - `checkExternalCollision(...)` 只检查 `Self` 和 `External` 对象之间的碰撞
   - `checkAll(...)` 检查自碰和外碰

3. Foxglove 可视化
   - `makeFoxgloveMarkers(...)` 生成 `visualization_msgs::msg::MarkerArray`
   - `Self` 对象默认绿色，`External` 对象默认橙色
   - disabled 对象透明度更低

## 支持的几何体

当前浅封装支持 primitive：

```cpp
collision::CollisionGeometrySpec::makeBox(x, y, z);
collision::CollisionGeometrySpec::makeSphere(radius);
collision::CollisionGeometrySpec::makeCapsule(radius, length);
collision::CollisionGeometrySpec::makeCylinder(radius, length);
```

暂时没有开放 mesh 加载。上游 Coal 的 assimp mesh loader 已经被裁剪掉，后续如果要从 STL/DAE 加载，建议在封装层单独加一个项目内可控的 mesh 转换入口，而不是恢复上游整套 loader。

## 最小示例

```cpp
#include "collision/coal_scene.hpp"

#include <Eigen/Geometry>

void example()
{
  collision::CollisionSceneOptions options;
  options.max_contacts_per_pair = 1;
  options.max_pairs = 16;
  options.security_margin = 0.0;

  collision::CollisionScene scene(options);

  std::string err;

  collision::CollisionObjectSpec link1;
  link1.id = "link1_box";
  link1.role = collision::CollisionObjectRole::Self;
  link1.geometry = collision::CollisionGeometrySpec::makeBox(0.20, 0.08, 0.08);
  link1.pose = Eigen::Isometry3d::Identity();

  if (!scene.upsertObject(link1, err)) {
    // err 里有失败原因
    return;
  }

  collision::CollisionObjectSpec obstacle;
  obstacle.id = "obstacle";
  obstacle.role = collision::CollisionObjectRole::External;
  obstacle.geometry = collision::CollisionGeometrySpec::makeSphere(0.10);
  obstacle.pose = Eigen::Isometry3d::Identity();
  obstacle.pose.translation() = Eigen::Vector3d(0.12, 0.0, 0.0);

  if (!scene.upsertObject(obstacle, err)) {
    return;
  }

  collision::CollisionCheckResult result;
  const bool ok = scene.checkExternalCollision(result);
  if (!ok) {
    // result.message: "collision detected: link1_box <-> obstacle"
    // result.pairs: 具体碰撞对、penetration_depth、position、normal
  }
}
```

## 更新对象位姿

同一个对象每帧只需要更新 pose，不需要重复创建 geometry：

```cpp
Eigen::Isometry3d next_pose = Eigen::Isometry3d::Identity();
next_pose.translation() = Eigen::Vector3d(0.3, 0.0, 0.2);

const bool updated = scene.setPose("link1_box", next_pose);
```

如果对象不存在或 pose 非 finite，`setPose` 返回 `false`。

## 自碰和外碰的角色定义

对象通过 `CollisionObjectRole` 分组：

```cpp
spec.role = collision::CollisionObjectRole::Self;
spec.role = collision::CollisionObjectRole::External;
```

检查逻辑：

```text
checkSelfCollision      Self <-> Self
checkExternalCollision  Self <-> External
checkAll                Self <-> Self 和 Self <-> External
```

`External <-> External` 默认不检查，因为一般只关心机械臂和环境是否碰撞。

## Foxglove 可视化

在 ROS2 node 中创建 MarkerArray publisher：

```cpp
marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/coal_collision_markers", rclcpp::QoS(10));
```

发布：

```cpp
auto markers = scene.makeFoxgloveMarkers("base_link", node->now());
marker_pub_->publish(markers);
```

Foxglove 里订阅 `/coal_collision_markers`，选择 `MarkerArray` 显示即可。

## CMake 依赖

使用这个封装的包需要依赖 `collision`：

```cmake
find_package(collision REQUIRED)

ament_target_dependencies(your_target
  collision
)
```

`collision` 包内部已经链接了 `coal::coal` 和 `visualization_msgs`，普通业务包不需要直接 `find_package(coal)`。

## 注意事项

- `CollisionScene` 内部用 mutex 保护对象表，可以在简单多线程场景下使用。
- 当前检查方式是两两遍历，适合对象数量较少的机器人连杆/环境 primitive 场景。
- 如果后续对象数量很多，再考虑恢复或重写 broadphase，而不是直接暴露上游 broadphase API。
- 如果需要 mesh、Octomap 或点云占据地图，要在 `CollisionScene` 旁边扩展明确的项目 API。不要把裁剪掉的上游大模块直接重新全量接回来。
