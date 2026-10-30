# JAKA Zu 3 in MuJoCo

本仓库包含六轴 JAKA Zu 3、用户提供的新版电动夹爪、桌面、可变形抹布、杯子和盘子的 MuJoCo 场景。机械臂第六轴负责整套末端旋转；第七个控制量用于夹爪活动件的开合演示。当前还包含笔记本摄像头手势跟随、21 点到 8 维末端指令的标准算法及七关节求解与检查程序。

## 快速开始

已验证的平台是 Linux x86_64、Python 3.10、MuJoCo 3.1.6。需要 Git 和带 `venv` 的 Python 3.10；打开仿真和摄像头窗口需要本地图形桌面和 OpenGL。机器人网格、夹爪源 STL 和场景资产已放在仓库中，克隆后不需要访问原来的移动硬盘，也不需要安装 Isaac Sim、ROS、PyTorch 或 LeRobot。

```bash
git clone https://github.com/yy2006-maker/jaka-zu3-mujoco.git
cd jaka-zu3-mujoco

python3.10 -m venv .venv-handcam
source .venv-handcam/bin/activate
python -m pip install -r requirements.txt
python scripts/check_model.py
python view_jaka_zu3.py
```

如果已有 Python 3.10 的 Conda 环境，可先激活它并用 `python -m venv .venv-handcam` 替代上面的建环境命令。如果没有 Python 3.10，也可用 `conda create -n jaka_mujoco --override-channels -c conda-forge python=3.10 pip -y` 创建，然后 `conda activate jaka_mujoco`，再使用 `python -m venv .venv-handcam`。不要在 Python 3.14 的 `base` 环境中直接安装这些依赖。

Ubuntu 若提示缺少 `venv`，安装 `python3.10-venv`；若提示缺少 OpenGL/GLFW 库，安装 `libgl1 libglib2.0-0 libglfw3`。其他操作系统尚未验证。纯数学及运动学检查不需要打开摄像头或显示窗口。

MuJoCo 窗口中，按 **Tab** 显示或隐藏左侧面板，按 **Shift+Tab** 显示或隐藏右侧面板。在右侧 **Control** 区域可以修改关节目标；角度单位是弧度，建议逐步改变。

## 夹爪控制

- `actuator_joint_1` 到 `actuator_joint_6`：JAKA 原有六个关节；第六轴负责整个末端工具旋转。
- `actuator_joint_7`：新版夹爪的开合演示控制量，只驱动活动夹持件。
- `joint_7 = 0.0`：夹爪闭合参考姿态；逐渐增大到 `0.9`：活动夹爪向外打开；回到 `0.0`：闭合。
- `0–0.9 rad` 是根据 STL 形状设定的预览范围，不是从电机铭牌或 CAD 配合中读取的实物行程。

第七轴的开合效果见 [docs/gripper_open_close.png](docs/gripper_open_close.png)。机械臂和夹爪现已启用与桌面、杯子、盘子及抹布的物理碰撞；碰撞使用 STL 网格的凸包近似，并通过碰撞掩码关闭机器人自身连杆之间的碰撞。杯子和盘子也保留彼此及与桌面的碰撞。

这只让接触力生效，不会自动执行“抓杯子并放到盘子里”。夹爪安装位置、碰撞形状、摩擦和夹持力还没有按真实设备校准；复杂网格的凸包也可能填平空腔。建议先在 MuJoCo 窗口里慢速试探杯子，确认夹爪方向与接触位置，再制作抓取轨迹。

## 笔记本摄像头手势跟随（仿真）

此实验脚本按 21 点到 8 维的几何映射方法，将 MediaPipe 关键点整理为 `ee8=[x,y,z,rx,ry,rz,g,reserved]`：thenar 中点给位置，五点 SVD 平面给手部姿态，拇指/食指指尖间距给连续夹爪开度，第 8 维固定为 0，再通过相对映射和 IK 协调六个机械臂关节。摄像头窗口会显示 21 点、8 维指令、平面残差、夹爪角和关节变化。它只连接 MuJoCo 模型，不连接或控制真实机械臂。

标准米制接口也已实现：`hand_to_ee(kp, R_wrist, T_base_cam)` 接收摄像头坐标系中的 21 个三维点、腕部旋转和完整外参，输出绝对末端指令。它与摄像头版本共用几何计算和 EE8 → 七关节求解入口；摄像头版本仍通过图像位移和掌面尺寸估计相对移动。接口、单位、离线数据入口和验证范围见 [手部到末端算法说明](docs/HAND_TO_EE.md)。

在上面克隆的仓库根目录执行。继续使用刚创建的独立 Python 3.10 环境，安装摄像头依赖：

```bash
.venv-handcam/bin/python --version
.venv-handcam/bin/python -m pip install -r scripts/requirements-hand-teleop.txt
.venv-handcam/bin/python -m pip check
.venv-handcam/bin/python -c "import mediapipe as mp; print(mp.__version__, hasattr(mp, 'solutions'))"
.venv-handcam/bin/python scripts/check_hand_to_ee.py
.venv-handcam/bin/python scripts/check_hand_pipeline.py
.venv-handcam/bin/python scripts/hand_camera_teleop.py --check-ik
.venv-handcam/bin/python scripts/hand_camera_teleop.py --camera 0
```

`.venv-handcam/bin/python --version` 应显示 `Python 3.10.x`，MediaPipe 检查应输出 `0.10.21 True`。此脚本使用 MediaPipe 的 `solutions.hands` 接口，需固定在 `0.10.21`，不要升级为不再包含该接口的新版。依赖文件会自动读取 `scripts/constraints-hand-teleop-py310.txt`，固定本次验证环境的依赖版本。本目录的 `.venv*` 已加入 `.gitignore`；其他人在自己的电脑上按依赖文件重建环境。

如果已在此环境安装了 `mediapipe 0.10.35`，先执行下面的降级命令，再运行上面的版本检查和 IK 检查：

```bash
.venv-handcam/bin/python -m pip install -r scripts/requirements-hand-teleop.txt
```

默认目标活动半径为 20 厘米；也可以在启动时用 `--motion-span 0.10` 改为 10 厘米，参数范围为 `0.01–0.20 m`。修改文件或启动参数后，需要退出旧程序再启动才能生效。

启动后先做一次指尖间距校准：让拇指和食指尽量靠拢并按 **C**，再把两指明显分开并按 **O**。校准完成后按 **Space** 开始跟随；再次按 **Space** 会停在当前位置。按 **R** 可把当前手的位置重新设为中心，按 **Q** 或 **Esc** 退出。摄像头窗口需保持打开并获得键盘焦点。

拇指/食指指尖间距按 C/O 两次校准归一化到 0–1，并连续映射至夹爪 `joint_7` 的 `0–0.9 rad`。摄像头画面的中值滤波降低抖动，夹爪目标按最高 `1.5 rad/s` 限速。thenar 相对按 Space 时的位置控制 TCP：屏幕右方映射为世界 +X，屏幕上方映射为世界 +Z；表观掌面尺寸变化估计相对深度并映射为世界 +Y。尺寸变化小于 2.5% 会被忽略，达到 20% 时 Y 位移达到最大值。手部平面相对旋转通过旋转向量进入 IK，而不是固定 TCP 姿态。XYZ 合成目标位移默认限制在半径 0.20 m；TCP 方向和关节限位仍由 IK 约束，部分方向会出现 `IK HOLD`。手部追踪丢失约 0.45 秒后，脚本解除跟随并保持当时姿态。

世界 +Y/-Y 是仿真坐标方向；相机并未做外参标定，因此“靠近/远离”只是利用单目手掌表观尺寸得到的相对深度提示，不是以米为单位的距离测量。手掌转动、透视和关键点抖动都会影响尺寸。先小幅、缓慢靠近或远离观察屏幕上的 TCP 目标 `Y`；若方向与场景直觉相反，停止跟随后再调整映射符号。此功能仅用于仿真验证，不能视为真实机械臂的精确深度控制。

运行 `scripts/check_hand_to_ee.py` 会检查根据参考说明重建的 A–F 案例、200 次旋转往返、零角与接近 180° 的姿态、坐标系变换不变性和摄像头适配。运行 `scripts/check_hand_pipeline.py` 会检查 5 组米制输入到七关节目标的完整链路、正运动学误差、无效/不可达输入保持与恢复、离线命令行输出。运行 `--check-ik` 会检查 13 个中心及 XYZ 正负方向目标。这些检查均不打开摄像头。

IK 的模型内收敛要求为位置误差 ≤ 0.5 mm、姿态误差 ≤ 0.005 rad（约 0.286°），未收敛时保持机械臂目标；该要求不是相机测量误差或物理执行误差。MediaPipe 的 world landmarks 仍是相对手部坐标，不能直接作为标准接口的绝对摄像头坐标。外参标定、实际三维测量及真实跟随精度尚未验证；当前求解也不执行避障规划。

当前还没有可靠的“手心朝向摄像头／手背朝向摄像头”分类，平面法线主要参考上一帧。快速翻掌、遮挡或重新进入画面可能造成方向歧义。ACT、π0、SmolVLA 训练以及 EasyMimic 的完整复现也不属于当前已实现功能。

## 更新已有副本

先退出运行中的仿真，在自己的仓库根目录执行：

```bash
git pull --ff-only origin main
.venv-handcam/bin/python -m pip install -r scripts/requirements-hand-teleop.txt
.venv-handcam/bin/python -m pip check
.venv-handcam/bin/python scripts/check_model.py
.venv-handcam/bin/python scripts/check_hand_to_ee.py
.venv-handcam/bin/python scripts/check_hand_pipeline.py
.venv-handcam/bin/python scripts/hand_camera_teleop.py --check-ik
.venv-handcam/bin/python scripts/hand_camera_teleop.py --camera 0
```

旧副本没有 `.venv-handcam` 时，先按快速开始创建。如果 `git pull` 提示本地修改冲突，请先保留自己的修改再合并。摄像头无法打开时，关闭其他占用摄像头的软件，或改为 `--camera 1`；C/O/Space 按键需要摄像头窗口获得焦点。

## 目录内容

- `jaka_zu3_mujoco.xml`：六轴机械臂、第七夹爪铰链、桌面、物体和抹布。
- `view_jaka_zu3.py`：以抬起姿态打开仿真窗口，并放置杯子、盘子和抹布。
- `scripts/hand_camera_teleop.py`：摄像头 21 点跟随、8 维末端映射、受限 IK 和离线 IK 自检。
- `scripts/hand_to_ee.py`：米制标准接口、关键点几何、旋转向量和摄像头相对映射。
- `scripts/check_hand_to_ee.py`：重建的 A–F 案例及数学、摄像头适配回归检查。
- `scripts/check_hand_pipeline.py`：米制输入到七关节目标的端到端运动学检查。
- `scripts/retarget_hand_sequence.py`：从米制 NPZ 数据离线求解 EE8 和七个关节目标。
- `docs/HAND_TO_EE.md`：输入契约、公式、验证范围及尚未完成的测量工作。
- `scripts/requirements-hand-teleop.txt`：摄像头跟随实验的可选依赖。
- `scripts/constraints-hand-teleop-py310.txt`：Linux/Python 3.10 验证环境的依赖版本快照。
- `assets/jaka_zu3_meshes/`：JAKA Zu 3 的七个连杆网格。
- `assets/gripper/`：新版 `装配体2.STL`、完整组件网格和装配元数据。
- `scripts/prepare_gripper.py`：针对这份新版 STL 的可重复组件提取脚本。
- `scripts/check_model.py`：检查七个控制量、夹爪运动和抹布与桌面的接触。
- `assets/tabletop/`、`assets/objaverse/`：桌面、杯子和盘子资源。

抹布是 MuJoCo 程序化生成的 15×11 `flexcomp` 网格，共 165 个顶点，不是扫描纹理模型。

## 重新生成新版夹爪

```bash
.venv-handcam/bin/python scripts/prepare_gripper.py
.venv-handcam/bin/python scripts/check_model.py
.venv-handcam/bin/python view_jaka_zu3.py
```

脚本会校验新版源文件 SHA256，确认 27 个连接组件后生成 `fixed_housing.stl`、`fixed_hardware_a.stl`、`fixed_hardware_b.stl` 和 `moving_jaw.stl`。MuJoCo 单个 STL 的面数有限制，因此固定紧固件被拆成两个网格，但仿真中仍显示为一个完整夹爪。

## 资产说明

JAKA 网格来自 JAKA ROS 2 描述，场景物体来自 LeRobot MuJoCo 教程，夹爪来自用户提供的 CAD 导出。公开仓库前请确认这些资产的再分发许可；如果许可不允许，应移除对应网格并改用下载脚本。

## 当前验证

在 Python 3.10、MuJoCo 3.1.6 下验证通过。检查脚本确认：七个执行器存在；第七轴在 `0` 与 `0.9 rad` 之间运动；新版夹爪的全部 27 个源组件已被保留；机械臂/夹爪与杯子的碰撞探针能产生接触约束；抹布仍能落到桌面并产生接触。模型未按真实 JAKA 或夹爪的动力学、摩擦和碰撞公差校准。
