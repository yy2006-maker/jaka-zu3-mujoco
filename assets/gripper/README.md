# 导入的新版夹爪

源文件是用户提供的 `装配体2.STL`，已复制为 `source_assembly.stl`。源 STL 是一个多零件装配体；STL 本身不保存单位、关节、配合约束或电机参数。脚本按连接关系识别出 27 个独立壳体，将所有壳体都保留到仿真中：

| 生成网格 | 用途 | 源组件 | 三角形 |
| --- | --- | --- | ---: |
| `fixed_housing.stl` | 主体、底板和长固定结构，随 Link 6 | 0、1、12、13、25 | 129,743 |
| `fixed_hardware_a.stl` | 第一组螺钉/圆柱紧固件，随 Link 6 | 2–11 | 179,649 |
| `fixed_hardware_b.stl` | 第二组螺钉/圆柱紧固件，随 Link 6 | 14–24 | 32,907 |
| `moving_jaw.stl` | 活动夹持件，由第七关节驱动 | 26 | 1,367 |

新 STL 中的坐标按毫米解释，并在生成网格时转换为米。MuJoCo 的 XML 不再读取原始移动硬盘路径；源文件和坐标记录都在本目录中。`assembly_metadata.json` 保存了输入 SHA256、组件分组、安装原点和范围假设。

`gripper_base` 固定在 JAKA `Link_6` 的外侧端面。`joint_7` 是夹爪开合的演示铰链，因此第六轴仍负责整个末端旋转，第七轴只改变活动夹持件的姿态。`0 rad` 对应闭合参考姿态；正向增大到 `0.9 rad` 时，活动件向外打开。范围是根据 STL 形状设定的可视化范围，不是实物舵机的标定行程。

目前机械臂与夹爪网格使用 MuJoCo 凸包碰撞，并通过 `contype="2" conaffinity="1"` 与桌面、杯子、盘子和抹布发生接触；碰撞掩码避免机器人连杆彼此发生自碰撞。由于凸包会填平网格的凹陷和空腔，某些姿态可能出现过早接触或影响夹持，后续可用更细致的简化碰撞体替换。几何、摩擦和夹持力目前仍是仿真近似值，没有按实物标定。

重新生成网格并检查模型：

```bash
conda activate lerobot310
cd jaka-zu3-mujoco-release
python scripts/prepare_gripper.py
python scripts/check_model.py
python view_jaka_zu3.py
```

如果以后提供了夹爪的真实旋转轴、开合角度或质量参数，只需据此更新脚本和 XML；当前安装位置与第七轴都是仿真预览假设。
