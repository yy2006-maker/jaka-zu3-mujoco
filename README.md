# JAKA Zu 3 in MuJoCo

本仓库包含六轴 JAKA Zu 3、用户提供的新版电动夹爪、桌面、可变形抹布、杯子和盘子的 MuJoCo 场景。夹爪第六轴负责整套末端旋转；第七个控制量用于夹爪活动件的开合演示。

## 快速开始

需要 Git、Python 3.10 和 `pip`。机器人网格、夹爪源 STL 和场景资产已放在仓库中，克隆后不需要访问原来的移动硬盘。

```bash
git clone https://github.com/yy2006-maker/jaka-zu3-mujoco.git
cd jaka-zu3-mujoco

conda create -n jaka_mujoco python=3.10 -y
conda activate jaka_mujoco
python -m pip install -r requirements.txt
python scripts/check_model.py
python view_jaka_zu3.py
```

MuJoCo 窗口中，按 **Tab** 显示或隐藏左侧面板，按 **Shift+Tab** 显示或隐藏右侧面板。在右侧 **Control** 区域可以修改关节目标；角度单位是弧度，建议逐步改变。

## 夹爪控制

- `actuator_joint_1` 到 `actuator_joint_6`：JAKA 原有六个关节；第六轴负责整个末端工具旋转。
- `actuator_joint_7`：新版夹爪的开合演示控制量，只驱动活动夹持件。
- `joint_7 = 0.0`：夹爪闭合参考姿态；逐渐增大到 `0.9`：活动夹爪向外打开；回到 `0.0`：闭合。
- `0–0.9 rad` 是根据 STL 形状设定的预览范围，不是从电机铭牌或 CAD 配合中读取的实物行程。

第七轴的开合效果见 [docs/gripper_open_close.png](docs/gripper_open_close.png)。机械臂和夹爪现已启用与桌面、杯子、盘子及抹布的物理碰撞；碰撞使用 STL 网格的凸包近似，并通过碰撞掩码关闭机器人自身连杆之间的碰撞。杯子和盘子也保留彼此及与桌面的碰撞。

这只让接触力生效，不会自动执行“抓杯子并放到盘子里”。夹爪安装位置、碰撞形状、摩擦和夹持力还没有按真实设备校准；复杂网格的凸包也可能填平空腔。建议先在 MuJoCo 窗口里慢速试探杯子，确认夹爪方向与接触位置，再制作抓取轨迹。

## 目录内容

- `jaka_zu3_mujoco.xml`：六轴机械臂、第七夹爪铰链、桌面、物体和抹布。
- `view_jaka_zu3.py`：以抬起姿态打开仿真窗口，并放置杯子、盘子和抹布。
- `assets/jaka_zu3_meshes/`：JAKA Zu 3 的七个连杆网格。
- `assets/gripper/`：新版 `装配体2.STL`、完整组件网格和装配元数据。
- `scripts/prepare_gripper.py`：针对这份新版 STL 的可重复组件提取脚本。
- `scripts/check_model.py`：检查七个控制量、夹爪运动和抹布与桌面的接触。
- `assets/tabletop/`、`assets/objaverse/`：桌面、杯子和盘子资源。

抹布是 MuJoCo 程序化生成的 15×11 `flexcomp` 网格，共 165 个顶点，不是扫描纹理模型。

## 重新生成新版夹爪

```bash
conda activate lerobot310
cd jaka-zu3-mujoco-release
python scripts/prepare_gripper.py
python scripts/check_model.py
python view_jaka_zu3.py
```

脚本会校验新版源文件 SHA256，确认 27 个连接组件后生成 `fixed_housing.stl`、`fixed_hardware_a.stl`、`fixed_hardware_b.stl` 和 `moving_jaw.stl`。MuJoCo 单个 STL 的面数有限制，因此固定紧固件被拆成两个网格，但仿真中仍显示为一个完整夹爪。

## 资产说明

JAKA 网格来自 JAKA ROS 2 描述，场景物体来自 LeRobot MuJoCo 教程，夹爪来自用户提供的 CAD 导出。公开仓库前请确认这些资产的再分发许可；如果许可不允许，应移除对应网格并改用下载脚本。

## 当前验证

在 Python 3.10、MuJoCo 3.1.6 下验证通过。检查脚本确认：七个执行器存在；第七轴在 `0` 与 `0.9 rad` 之间运动；新版夹爪的全部 27 个源组件已被保留；机械臂/夹爪与杯子的碰撞探针能产生接触约束；抹布仍能落到桌面并产生接触。模型未按真实 JAKA 或夹爪的动力学、摩擦和碰撞公差校准。
