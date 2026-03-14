### UTILS
> utils 是 utilities 的缩写，意为公共事业，公共设施
> 作用是提供复用功能包，便于集中管理和统一使用

#### 文件结构
> 包括 log_utils 和 param_utils 

~~~ bash
src/utils/params_utils/
├── CMakeLists.txt
├── config
│   ├── gripper_reset.yaml
│   ├── intent_reset.yaml
│   ├── joint_reset.yaml
│   └── moveit_reset.yaml
├── include
│   └── params_utils
│       ├── common
│       │   ├── gripper_reset_config.hpp
│       │   ├── intent_reset_config.hpp
│       │   ├── joint_reset_config.hpp
│       │   └── moveit_reset_config.hpp
│       ├── core
│       │   ├── descriptors.hpp
│       │   ├── params.hpp
│       │   └── validators.hpp
│       └── param_utils.hpp
└── package.xml
~~~

param_utils 只需提供复用配置和简单配置函数，不封装 ROS ，其中全是静态参数，只服务于业务通信节点
包括 gripper_reset / intent_reset / joint_reset / moveit_reset 设多种的原因是减小颗粒度，防止节点参数过于膨胀
这些基础静态参数有相应的yaml文件，同时也有默认值，只需要在对应launch中找到包含即可
而动态参数需放置到节点私有参数中，这样遇到动态参数修改，只需在节点对应yaml，遇到复用的静态参数修改，只需在param_utils对应yaml
原则：只要在 params_utils 定义的参数，绝不出现在节点私有参数里；宁可多加，不少加

统一的配置来源：通信与能力节点都通过 params_utils + reset yaml 组合加载。
  - 统一的结构风格：Config 都有 load/Load + validate + summary，且 load 在 hpp 内联。
  - 统一的日志：节点启动时打印 summary（你点名的都加了）。

