# 落体猫运动

## 运行

**方法一**:尝试双击run.bat。

**方法二**:

运行"bullet3-3.25\execute\App_RobotSimulator_vs2010_x64_release.exe"

## 操作

- <span style="color:red;">F/J/B/N</span>  前进/后退/上/下
- <span style="color:red;">Q/E/A/D</span> 下旋/上仰/左转/右转
- <span style="color:red;">W/G/V/</span>其他功能
- <span style="color:red;">空格键</span> 暂停/继续
- <span style="color:red;">R</span> 重置摄像视角
- 鼠标强制拖动(不推荐)
- <span style="color:red;">RESET_Height</span> 每次猫重置后的高度
- <span style="color:red;">Max_Velocity</span> 猫的速度超过此值系统自动结束
- <span style="color:red;">Allow_Reset</span> 不允许猫落地，高度小于5m自动重置（划到右侧开启）
- <span style="color:red;">draw</span> 是否保存单次下落图像（划到右侧开启）
- <span style="color:red;">Auto</span> 是否开启随机动画（划到右侧开启）
- <span style="color:red;">Quit</span> 退出

- 每次下落都会生成报告，虽然支持随时暂停，但是可能会导致数据混乱。。。
- <span style="color:red;">测试:</span>确保提取开启了draw，关闭了Allow_Reset(不然没法落地).每次的图像文件Test会覆盖，但是报告的不会。

## 功能

1. 实时调整猫的各个关节（合理范围内）
2. 进行随机动画
3. 实时显示猫下落过程的线速度、角速度，多方向(x/y/z/角速度方向)转动惯量评估旋转状态)
4. 支持实时调试、暂停、设置重置高度
5. 支持每次正式测试计算落地冲力，并生成报告
6. 由于时间紧迫，**不支持实时录制和演示图像**， 支持<span style="color:red;">自动保存图像</span>，与<span style="color:red;">手动录制</span>(提供软件包)进行时间上的校对，便可查看特定时间点猫的姿态

## 项目说明

### 主要第三方库

- bullet 充当物理引擎和进行GUI窗口实时渲染
- matplotlibcpp 绘图
- FastNoiseLite 用于随机动画算法

### 猫的结构

<p align="center">
  <img src="resources/cat_structure.png" width="400" alt="落体猫示意图" >
</p>

**其中**:

- F/B/L/R: 前后左右
- toe是固定关节，不可调整;Base_Link是身体，也不可调节
- 长/宽/高:0.3m/0.15m/0.2m(粗略值)
- 关节质量：0.1~0.4kg;身体质量:3kg。



