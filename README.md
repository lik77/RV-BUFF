搬运自华师rm_vision
本仓库为适配于野狼25赛季视觉自瞄模块
# rm_vision总入口

  ## 再见RM
(https://github.com/chenjunnn/rm_vision)


  ## RV备份

  [rm_vision（源码备份）.zip](https://flowus.cn/preview/85b124e5-8c88-4ed6-9088-8142ce4248f9)

  [rm_vision · GitLab](https://gitlab.com/rm_vision)


# 安装ROS

  [Ubuntu22.04.1安装ROS2入门级教程(ros-humble)_ros humble_Python-AI Xenon的博客-CSDN博客](https://blog.csdn.net/yxn4065/article/details/127352587)


# 创建工作空间

```
mkdir -p ~/ros_ws/src
```


# 下载源代码

  在 `ros_ws/src` 目录下

  ```Shell
git clone https://gitee.com/LihanChen2004/rm_auto_aim
git clone https://gitee.com/LihanChen2004/ros2_hik_camera
git clone https://gitee.com/LihanChen2004/rm_vision
git clone https://gitee.com/LihanChen2004/rm_serial_driver
git clone https://gitee.com/LihanChen2004/rm_gimbal_description
```


```Shell
sudo apt install ros-humble-foxglove-bridge
```


# 编译

  在 `ros_ws` 目录下

  ```Shell
rosdep install --from-paths src --ignore-src -r -y
```


```Shell
colcon build --symlink-install
```


# 运行节点

  ```Shell
sudo chmod 777 /dev/ttyACM0
```


运行每个节点，必须新建终端并输入命令，且运行前需要执行 `source install/setup.bash`

 ```Shell
source install/setup.bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```


  ```Shell
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```


  - 单独运行子模块（一般用不上，写在这只为了有时开发要调用 rv 独立模块调试）

    ```自瞄
source install/setup.bash
ros2 launch auto_aim_bringup auto_aim.launch.py 
```


 ```海康
source install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
```


  ```串口通讯模块
source install/setup.bash
ros2 launch rm_serial_driver serial_driver.launch.py
```


# 启动可视化

  打开新的终端

source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765



  使用 Chromium 浏览器打开 [https://studio.foxglove.dev/](https://studio.foxglove.dev/) ，并打开 connection

  > 建议下载**[foxglove客户端](https://foxglove.dev/download)**，避免Web端视频闪烁

  ![image.png](https://tc-cdn.flowus.cn/oss/f53ee643-9ab0-4a13-b985-ae90719c33a1/image.png?time=1744249500&token=050a53cf23a46b4f972123982e0149d3e4812a42077c927bde1ad19b44fdd2e3&role=sharePaid)

  ![image.png](https://tc-cdn.flowus.cn/oss/bc616571-046d-4ebb-95dd-8253e50948e9/image.png?time=1744249500&token=6c657afc059d30aa9ef4a1ff6d1712b1eb53e637cd8282f4c54787f7a435b4b6&role=sharePaid)

# ROSbridge的参数

[ROSbridge的参数](https://flowus.cn/86e7e54f-a0fc-467d-909d-95d1509f62f2)

# 【拓展】其他操作

  ## 查看相机帧率
ros2 topic hz /camera_info



  ## 关闭所有节点
ros2 node killall






