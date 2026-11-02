# LSLIDAR_CX_V4.3.0_240401_ROS 使用说明

## 1.工程介绍
​		LSLIDAR_CX_V4.3.0_240401_ROS为linux环境下雷达ros驱动，程序在ubuntu 20.04 ros noetic,ubuntu18.04 ros melodic以及ubuntu16.04 ros kinetic下测试通过。适用于C16,C32 3.0版本和C1,C1Plus,C4,C8,C8F,CKM8,C16,MSC16,C16国产,C32W,C32WN,C32WB,C32WP,CH32R 4.0版本以及N301 5.5版本雷达。

## 2.依赖

1.ubuntu20.04 ros noetic/ubuntu18.04 ros melodic/ubuntu16.04 ros kinetic

2.ros依赖 

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions  ros-$ROS_DISTRO-diagnostic-updater
```

3.其他依赖

~~~bash
sudo apt-get install libpcap-dev
~~~



## 3.编译运行

### 3.1 编译

~~~bash
#创建工作空间及src目录  lidar_ws为工作空间名 可自定义
mkdir -p ~/lidar_ws/src
#将驱动压缩包解压缩放到~/lidar_ws/src 目录下
#返回工作空间
cd ~/lidar_ws
#编译及刷新
catkin_make
source devel/setup.bash
~~~

### 3.2 运行

运行单个雷达:

~~~bash
roslaunch lslidar_cx_driver lslidar_cx.launch
~~~

运行多个雷达：

~~~bash
roslaunch lslidar_cx_driver lslidar_double.launch
~~~



## 4.参数介绍

lslidar_cx.launch文件内容如下，每个参数含义见注释说明。

~~~bash
<launch>
  <arg name="device_ip" default="192.168.1.200" />   # 雷达ip 
  <arg name="msop_port" default="2368"/>             # 雷达目的数据端口 
  <arg name="difop_port" default="2369"/>            # 雷达目的设备端口 
  <arg name="use_time_service" default="false" />    # 雷达是否使用授时(GPS PTP NTP) 
  <arg name="pcl_type" default="false" />            # 点云类型   true: xyzi 
  <arg name="packet_rate" default="1695.0"/>		 # 离线播包时每秒读取的pcap包数

  <node pkg="lslidar_cx_driver" type="lslidar_cx_driver_node" name="lslidar_driver_node" output="screen">
    <!--param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/-->#pcap包路径，加载pcap包时打开此注释
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="packet_rate" value="$(arg packet_rate)"/>
    <param name="device_ip" value="$(arg device_ip)" />
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="pcl_type" value="$(arg pcl_type)"/>
    <param name="add_multicast" value="false"/>     # 是否开启组播	ture:启用组播模式
    <param name="group_ip" value="224.1.1.2"/>      # 组播ip
    <param name="frame_id" value="laser_link"/>     # 点云帧id
    <param name="pointcloud_topic" value="lslidar_point_cloud"/>  <!-- 点云话题名 -->
    <param name="distance_min" value="0.15"/>       # 雷达扫描最小距离	默认0.15m
    <param name="distance_max" value="200.0"/>      # 雷达扫描最大距离	默认200米
    <param name="angle_disable_min" value="0"/>     # 雷达扫描最小裁剪角度 单位: 0.01°
    <param name="angle_disable_max" value="0"/>     # 雷达扫描最大裁剪角度 单位: 0.01°
    <param name="distance_unit" value="0.40"/>
    <param name="horizontal_angle_resolution" value="0.18"/>  #10Hz:0.18  20Hz:0.36 5Hz:0.09-->
    <param name="publish_scan" value="false"/>      # 是否发布laserscan  true: 发布laserscan
    <param name="scan_num" value="15"/>             # laserscan线号
    <param name="coordinate_opt" value="false"/>    # 点云0度角对应方向  true: x轴正方向
  </node>

  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find lslidar_cx_driver)/rviz/lslidar.rviz"/>			   # 若不需要启动rivz，请注释或删除改行
 
 <!--node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 0 0 0 0 world laser_link 100" /-->					# tf坐标转换
  
</launch>

~~~

### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~shell
  <param name="add_multicast" value="false"/>               #是否添加组播
  <param name="group_ip" value="224.1.1.2"/>                #组播的ip
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线pcap模式：

- 把录制好的pcap文件，拷贝到lslidar_cx_driver/pcap文件夹下。

- 修改launch文件以下参数

  ~~~shell
  #取消注释
      <param name="pcap" value="$(find lslidar_cx_driver)/pcap/xxx.pcap" />   #pcap包路径，加载pcap包时打开此注释
  ~~~

###  pcl点云类型：

- 修改launch文件以下参数

  ~~~shell
  <param name="pcl_type" value="$(arg pcl_type)"/>         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
  ~~~

  

- 默认false为自定义点云类型，定义参考lslidar_driver/include/lslidar_driver.h头文件

- 改为true,为pcl自带类型 :

  ~~~shell
  pcl::PointCloud<pcl::PointXYZI>
  ~~~



## 5.设置雷达



## 修改雷达授时方式：

~~~bash
#新开一个终端
source devel/setup.bash
~~~

GPS授时：

~~~bash
rosservice call /time_service "time_service_mode: 'gps'
ntp_ip: ''" 
~~~

PTP授时：

仅支持4.0激光雷达

~~~bash
rosservice call /time_service "time_service_mode: 'ptp'
ntp_ip: ''" 
~~~

NTP授时：

仅支持4.0激光雷达

~~~bash
rosservice call /time_service "time_service_mode: 'ntp'
ntp_ip: '192.168.1.102'" 
~~~



### 雷达上下电(只发设备包，不发送数据包，4.0雷达依然转动)：

~~~bash
#新开一个终端
source devel/setup.bash
~~~

上电：

~~~bash
rosservice call /lslidar_control "laser_control: 1"
~~~

下电：

~~~bash
rosservice call /lslidar_control "laser_control: 0"
~~~



### 雷达转动/停止转动(电机停转)：

~~~bash
#新开一个终端
source devel/setup.bash
~~~

转动：

~~~bash
rosservice call /motor_control "motor_control: 1"
~~~

停止转动：

~~~bash
rosservice call /motor_control "motor_control: 0"
~~~



### 设置雷达转速：

~~~bash
#新开一个终端
source devel/setup.bash
~~~

可选频率  5Hz/10Hz/20Hz

~~~bash
rosservice call /set_motor_speed "motor_speed: 20"
~~~



### 设置雷达去除雨雾尘等级：(仅支持该功能的雷达有效)

仅支持4.0激光雷达

~~~bash
#新开一个终端
source devel/setup.bash
~~~

可选等级  0/1/2/3 ，0-3 数字越大，去除越强

~~~bash
rosservice call /remove_control "remove_control: 0" 
~~~



### 设置雷达数据包端口

~~~bash
#新开一个终端
source devel/setup.bash
~~~

~~~bash
rosservice call /set_data_port "data_port: 2368"  #范围[1025,65535]
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达设备包端口

~~~bash
#新开一个终端
source devel/setup.bash
~~~

~~~bash
rosservice call /set_dev_port "dev_port: 2369"  #范围[1025,65535]
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达ip

~~~bash
#新开一个终端
source devel/setup.bash
~~~

~~~bash
rosservice call /set_data_ip "data_ip: '192.168.1.200'"
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达目的ip

~~~bash
#新开一个终端
source devel/setup.bash
~~~

~~~bash
rosservice call /set_destination_ip "destination_ip: '192.168.1.102'"
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



## FAQ

Bug Report

Original version : LSLIDAR_CX_V4.1.4_220425_ROS

Modify:  original version

Date    : 2022-04-25

--------------------------------------------------------------------

Update version : LSLIDAR_CX_V4.1.5_220620_ROS

Modify:  c8雷达，光学垂直角度修改。

Date    : 2022-06-20

------------------------------------------------------------

Update version : LSLIDAR_CX_V4.2.0_221028_ROS

Modify:  1.新增对4.0版本单线雷达， C32 70度和90度雷达的支持

​				2.统一laserscan和pointcloud2坐标系

Date    : 2022-10-28

------------------

Update version : LSLIDAR_CX_V4.2.1_221227_ROS

Modify:  1.scan话题新增强度信息

​				2.fpga升级，C32 90度修改计算公式

​				3.ROS驱动新增修改授时方式的功能

​				4.新增雷达上下电,修改雷达ip，端口，转速等功能。

​				5.修复ntp授时解析问题。

Date    : 2022-12-27

-----

Update version : LSLIDAR_CX_V4.2.2_230322_ROS

Modify:  1.增加使用时长提示

​				2.新增驱动版本提示

Date    : 2023-03-22

------------

Update version : LSLIDAR_CX_V4.2.3_230403_ROS

Modify:  1.fpga变更，修改C32W的计算公式

Date    : 2023-04-03

------



Update version : LSLIDAR_CX_V4.2.4_230705_ROS

Modify:  1.修复雷达切换成低功耗模式后，不能切换成正常模式的问题

​               2.兼容C1Plus 型号雷达

Date    : 2023-07-05

-----



Update version : LSLIDAR_CX_V4.2.5_230913_ROS

Modify:  1.优化代码，降低cpu占用。

​				2.增加nodelet功能。   

​				3. 删除雷达型号参数，解写自动识别雷达型号。

Date    : 2023-09-13

-----



Update version : LSLIDAR_CX_V4.2.6_231012_ROS

Modify:  1.新增兼容n301,5.5版本。

Date    : 2023-10-12

-----



Update version : LSLIDAR_CX_V4.2.7_231020_ROS

Modify:  1.增加设置雷达去除雨雾尘等级功能

Date    : 2023-10-20

-----



Update version : LSLIDAR_CX_V4.2.8_240321_ROS

Modify:  1.兼容C4雷达

​		   	2.兼容CH32R v4.8版本雷达

​			   3.点云角度裁剪改为点云角度保留

​		   	4.优化代码，降低cpu占用

Date    : 2024-03-21

-----



Updateversion : LSLIDAR_CX_V4.2.9_240325_ROS

Modify:  1.兼容C16 C32 v3.0版本雷达

​			   2.兼容C32WN C32WB雷达

​			   3.点云角度保留改为点云角度裁剪(支持负角度裁剪)

Date    : 2024-03-25

-----



Update version : LSLIDAR_CX_V4.3.0_240401_ROS

Modify:  1.修复3.0雷达无法上下电问题

​			   2.限制雷达IP设置范围，禁止将224网段设为雷达IP

Date    : 2024-04-01

-----



Update version : LSLIDAR_CX_V4.3.1_240409_ROS

Modify:  1.增加数据断流重新获取到数据包后再次判断雷达型号

​			   2.修复3.0雷达相邻两个数据包点角度出现连续过零度问题

Date    : 2024-04-09

-----



