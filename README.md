# detect_planner

一个用于点对点的导航逻辑



### 版本解释

- main 分支：原始版本；
- node_version分支：以节点启动版本，解决了几个小小的Bug（手动狗头）；
- action_server分支：代码规范化格式化之后的版本，改写成了action_server的形式，可以应用于ros工程当中，当然也解决了几个小小的Bug（这次一定没有bug了，嗯）。

### 依赖

action_server版本需要依赖robot_msg包（闭源），如果需要，可以手动添加action文件和修改CmakeLists.txt。