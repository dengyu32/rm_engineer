### commit 规范
> 项目约定速成的 commit 规范

#### 基础指令
~~~bash
git commit -m " XXX

1.xxx
2.xxx
3.xxx"
~~~
git 会自动识别空行分隔标题和正文

#### 项目规范
> feat(teleop): add teleop_node 
> - create teleop directory 
> - implement teleop_node 
> - add teleop.md documentation

标准结构：
<type>(<scope>): <summary>

<body>

<footer>

type:
| type     | 含义             |
| -------- | ------------- |
| feat     | 新功能（feature）  |
| fix      | 修复 bug        |
| docs     | 文档修改          |
| style    | 代码格式修改（不影响逻辑） |
| refactor | 重构代码          |
| perf     | 性能优化          |
| test     | 测试代码          |
| chore    | 构建、脚本、配置      |

scope:
表示修改影响哪个模块

summary:
简短标题,说明修改内容

body:
详细的修改说明

footer:
脚注，用来关联issue，破坏性更新（BREAKING CHANGE）