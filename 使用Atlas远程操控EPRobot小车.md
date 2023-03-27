# 使用Atlas远程操控EPRobot小车

## 注意

ros_udp_remote还没有进行进一步测试以及优化，现仅能完成功能，并在turtlesim中通过测试

## 准备

1. 连接Atlas，并令其联网：https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1013/environment/atlased_04_0012.html
2. 为Atlas安装ros-melodic：http://wiki.ros.org/melodic/Installation/Ubuntu
3. 下载仓库：https://github.com/Dessera/ros_udp_remote.git

## 步骤

1. 将Atlas与小车连接（通过USB或者无线WIFI模块）

2. 在EPRobot上运行小车驱动

   ```shell
   roslaunch eprobot_start EPRobot_start.launch
   ```

3. 在Atlas上构建udp_receiver包

   ```shell
   cd ./udp_receiver
   catkin_make
   ```

4. 在Atlas上编辑~/.bashrc，加入

   ```shell
   export ROS_HOSTNAME= #本机IP
   export ROS_MASTER_URI= #roscore IP 即小车的局域网IP
   ```

5. 在Atlas上启动udp服务器

   ```
   cd ./udp_receiver/devel/
   source ./setup.bash
   rosrun udp_server udp_server <本机IP> <端口>
   ```

6. 连接EPRobot开放的WIFI，并在设备上启动软件包中的udp_sender

   (注：udp_sender并未构建，可以优先使用`npm run tauri build`构建，或者直接使用`npm run tauri dev`使用开发版本)

7. 在软件中填写服务器开放的IP和端口，点击connect
8. 如果连接成功，可以拖拽下方的模拟摇杆对小车进行操控