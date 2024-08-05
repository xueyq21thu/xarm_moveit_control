# xArm7相关操作（含夹爪）

本package下的文件包含了机械臂跟随，夹爪操作，状态读取等操作。

## 0. 启动机械臂

启动机械臂之前先拧开机械臂的电源。

### 0.1 ROS2连接

首先进入ros空间的最上层目录，打开终端并source。以后每次打开新终端都需要source。

```bash
source install/setup.bash
```


机械臂通过ip与上位机相连，连接成功后会启动rviz，显示机械臂的状态。相关指令如下：

```bash
ros2 launch xarm_planner xarm7_planner_realmove.launch.py robot_ip:="192.168.1.231"
```

其中，三台机械臂的ip分别为：

- *192.168.1.228*（应该是）（最外面的）
- 192.168.1.231（离上位机最近的一台）
- 192.168.1.234（UMI夹爪那一台）

当局域网内要启动多台机械臂时，需要设置`ROS_DOMAIN_ID`，以区分不同的机械臂。例如：

```bash
export ROS_DOMAIN_ID=10
ros2 launch xarm_planner xarm7_planner_realmove.launch.py robot_ip:="192.168.1.231"
```

### 0.2 UI连接

UI是控制该机械臂最基础也是最顶层的设计，往往机械臂报错时需要通过UI控制解除错误并把它挪开。

在上位机上用浏览器打开如下链接：[UI](http://192.168.1.231:18333/control?lang=en&channel=prod)，其中链接中的ip地址为机械臂的ip地址。

## 1. 状态

### 1.1 机械臂状态参数

机械臂运动状态参数如下：
- state：描述机械臂当前的状态，包括：NORMAL（0），MOTION（2），STOP（5）等，需要注意的是，当机械臂处于NORMAL状态时，才能进行运动控制。
- mode：描述机械臂运动（控制）的模式，可以根据自己的需要，选择不同的模式进行控制。以下是全部的模式：

    Mode 0 : xArm controller (Position) mode.

    Mode 1 : External trajectory planner (position) mode.

    Mode 2 : Free-Drive (zero gravity) mode.

    Mode 3 : Reserved.

    Mode 4 : Joint velocity control mode.

    Mode 5 : Cartesian velocity control mode.

    Mode 6 : Joint space online planning mode. (Firmware >= v1.10.0)

    Mode 7 : Cartesian space online planning mode. (Firmware >= v1.11.0)

- error：描述机械臂的错误状态，如力矩偏离理论值过大，工作空间超限等。每当机械臂内有error存在时均无法进行任何move操作，必须先行清除error状态。
- warning：描述机械臂的警告状态。每当机械臂内有warning存在时均无法进行任何move操作，必须先行清除warning状态。
- pose：描述机械臂末端的当前位姿，包括位置和姿态。其中xyz单位为mm，rpy单位为rad。
- angle：描述机械臂当前的关节角度。*单位为rad。*
- tcp_load：描述机械臂末端的负载信息。

以上内容详见[官方文档](https://github.com/xArm-Developer/xarm_ros#6-mode-change)。

### 1.2 机械臂状态读取与设置

机械臂状态参数的读取与设置，可以通过topic和service进行。

xarm状态读取接口如下：
topic：`/xarm/robot_states`, msg：`xarm_msgs/msg/RobotMsg`。 该接口包含当前机械臂的state，mode，error，warning，pose，angle等信息。
service：`/xarm/get_state`, srv: `xarm_msgs/srv/GetInt16`。 该接口可以获取当前机械臂的state. `xarm/get_mode`, srv: `xarm_msgs/srv/GetInt16`

xarm状态设置接口如下：
service：`/xarm/set_state`, srv: `xarm_msgs/srv/SetInt16`。 该接口可以设置当前机械臂的state。`xarm/set_mode`, srv: `xarm_msgs/srv/SetInt16`。 该接口可以设置当前机械臂的mode。`/xarm/clean_error`, srv: `xarm_msgs/srv/Call`, 该接口可以清除当前机械臂的error。`/xarm/clean_warning`, srv: `xarm_msgs/srv/Call`, 该接口可以清除当前机械臂的warning。

## 2. 运动

### 1.1 基于UI的运动控制

UI可以通过设置关节角度来控制机械臂的运动，也可以通过设置手动模式来手动给操作控制机械臂的运动等。机械臂的几乎所有参数均可见。UI有sim与real模式，一黑一白，分别对应仿真与真实模式。无论是不是通过UI控制，如果要realmove，一定要在UI上切换到real上。

### 1.2 基于内置接口的运动
首先连接机械臂，然后通过内置service接口进行运动控制。
主要为`/xarm/set_position`, srv: `xarm_msgs/srv/MoveCartesian`, 调用如下：

```bash
ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian "{pose: [400, 0, 600, 3.14, 0, 0], speed: 200, acc: 50}"
```

其中pose为末端位姿，speed为速度，acc为加速度。
`/xarm/set_joint`, srv: `xarm_msgs/srv/MoveJoint`, 调用如下：

```bash
ros2 service call /xarm/set_joint xarm_msgs/srv/MoveJoint "{joint: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], speed: 200, acc: 50}"
```

其中joint为关节角度，speed为速度，acc为加速度。

需要注意的是，这种方式有一定自碰撞的风险。

### 1.3 基于MoveIt的运动控制

MoveIt是ROS中用于机器人运动规划的一个框架，可以方便地实现机械臂的运动规划。MoveIt的接口包括规划器接口和控制器接口。规划器接口用于生成机械臂的运动轨迹，控制器接口用于控制机械臂按照生成的轨迹运动。

MoveIt的运动控制主要包括以下几个步骤：
1. 初始化MoveIt规划器
2. 设置机械臂的起始位姿和目标位姿
3. 调用规划器生成运动轨迹
4. 调用控制器控制机械臂按照生成的轨迹运动

初始化MoveIt规划器的代码如下（可替换掉planner连接rviz）

```bash
ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py robot_ip:="192.168.1.231"
```

此时rviz中可以使用鼠标拖动末端位姿，然后点击“Plan and Execute”按钮，即可规划并执行运动。（当然也可以先plan然后再execute）。需要说明的是，正常情况下机械臂所有关节处于橘黄色，代表正常，如果机械臂某些关节处于玫红色，代表该关节处于限制状态，无法运动。此时需要打开UI，解除限制并挪走机械臂。

然后调用打包好的service进行运动控制：

```bash
ros2 run xarm_moveit_control xarm_moveit_control_service
```

新建终端，在终端中输入以下命令即可进行运动控制：

```bash
ros2 service call /xarm_set_position xarm_msgs/srv/SetFloat32List "{datas: [0.4, 0, 0.6, 3.14, 0, 0]}"
```

其中datas为末端位姿，单位为m。该服务即可调用MoveIt规划器进行运动控制。

## 3. 夹爪

DH夹爪是通过机械臂的ModBus总线进行通讯的，且通信与电源一体，因此要使用夹爪必须启动机械臂并用ros连接。

要进行正常通讯，还需要设置ModBus总线的timeout与baudrate参数。分别为：500ms与115200。

当然，这么搞非常麻烦，因此我提供了一个简单的打包好的service，可以直接调用。新建一个终端，source后调用：

```bash
ros2 run xarm_moveit_control grip_control_service
```

再新建一个终端，此时可以调用的服务有以下三个：`/init_gripper`，`/set_gripper`，`/get_gripper`。

`/init_gripper`用于初始化夹爪，`/set_gripper`用于设置夹爪的开合程度，`/get_gripper`用于获取夹爪的开合程度。开合程度为一个0-1000的整数，0为全关，1000为全开。使用时，先行调用`/init_gripper`初始化夹爪，此时timeout，baudrate与夹爪都会被初始化，夹爪张到最大。

```bash
ros2 service call /init_gripper std_srvs/srv/Call
```

随后可以调用`/set_gripper`设置夹爪的开合程度，例如：

```bash
ros2 service call /set_gripper xarm_msgs/srv/SetInt16 "{data: 500}"
```

最后可以调用`/get_gripper`获取夹爪的开合程度，例如：

```bash
ros2 service call /get_gripper std_srvs/srv/Call
```

## 4. 通讯

### 4.1 机械臂控制通信

新建终端并运行以下命令，可以通过service进行机械臂的控制。

```bash
ros2 run umi_control tcp_move
```

由于该接口启动了双线程，因此需要看到终端返回机械臂正确的初始位姿与机械臂运动模式与tcpload成功设置后方可连接。
所有配置参数均在`config.json`内。
传输数据为6维ndarray，dtype=np.float64，单位为mm与rad，开启运动后xyz是相对运动，rpy是绝对角度。

### 4.2 夹爪控制通信

新建终端运行以下指令：

```bash
ros2 run umi_control gripper
```

然后等待初始化完成后往里面传width即可。

初始化失败或者卡住ctrl+C重试即可，这个看脸。未来会考虑写一个launch解决所有问题。
