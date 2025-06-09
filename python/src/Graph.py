import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import mplcursors
from collections import deque
Time_Duration=int(input("请输入图像x坐标轴时长:(单次下落推荐8,调试推荐不低于30"))
# 全局数据存储
class DataMonitor:
    def __init__(self, max_len=1500):
        self.time_data = deque(maxlen=max_len)
        self.height_data = deque(maxlen=max_len)
        self.velocity_data = deque(maxlen=max_len)
        self.inertia_data = deque(maxlen=max_len)
        self.inertia_x_data = deque(maxlen=max_len)
        self.inertia_y_data = deque(maxlen=max_len)
        self.inertia_z_data = deque(maxlen=max_len)
        self.pos_x_data = deque(maxlen=max_len)
        self.pos_y_data = deque(maxlen=max_len)
        self.Angular_speed_data= deque(maxlen=max_len)
        self.drag_data= deque(maxlen=max_len)
        self.lock = threading.Lock()
        self.last_update = 0
        self.update_interval = 0.05  # 50ms更新间隔
        self.paused = True  # 新增暂停状态标志
        self.mass=0

    def update(self, time, height, velocity, inertia, inertias, pos_x, pos_y,drag,Angular_speed_data):
        if not self.paused:  # 只有在非暂停状态时更新数据
            if time - self.last_update >= self.update_interval:
                with self.lock:
                    self.time_data.append(time)
                    self.height_data.append(height)
                    self.velocity_data.append(velocity)
                    self.inertia_data.append(inertia)
                    self.pos_x_data.append(pos_x)
                    self.pos_y_data.append(pos_y)
                    self.inertia_x_data.append(inertias[0])
                    self.inertia_y_data.append(inertias[1])
                    self.inertia_z_data.append(inertias[2])
                    self.Angular_speed_data.append(Angular_speed_data)
                    self.drag_data.append(drag)
                self.last_update = time

def Graph_thread():
    plt.style.use('ggplot')
    fig, (ax1, ax2,ax3) = plt.subplots(3, 1, figsize=(12,16))
    fig.suptitle('Reference Graph')
    fig.text(0.5, 0.95, f'Mass: {data_monitor.mass:.2f} kg', fontsize=14, color='blue', ha='center', va='top')

    # ========== 上子图 (高度和速度) ==========
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Height (m)', color='r')
    line_height, = ax1.plot([], [], 'r-', label='Height', linewidth=2)
    ax1.tick_params(axis='y', labelcolor='r')
    ax1.set_ylim(0, 150)
    ax1.grid(True)

    ax1_r = ax1.twinx()
    ax1_r.set_ylabel('Velocity (m/s)', color='b')
    line_velocity, = ax1_r.plot([], [], 'b--', label='Velocity', linewidth=2)
    ax1_r.tick_params(axis='y', labelcolor='b')
    ax1_r.set_ylim(-5, 100)

    # ========== 下子图 (位置和惯量) ==========
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (dm)', color='m')
    line_pos_x, = ax2.plot([], [], 'm-', label='Pos X', linewidth=2)
    line_pos_y, = ax2.plot([], [], 'c-', label='Pos Y', linewidth=2)
    ax2.tick_params(axis='y', labelcolor='m')
    ax2.set_ylim(-5,5)
    ax2.grid(True)

    ax2_r = ax2.twinx()
    ax2_r.set_ylabel('Inertia (10e3 kg·m²)', color='g')
    line_inertia, = ax2_r.plot([], [], 'g:', label='Inertia', linewidth=3)
    line_inertia_x, = ax2_r.plot([], [],
                                 color='#FF4500',  # 橙色（X轴传统色）
                                 linestyle='--',  # 虚线
                                 marker='',  # 无标记
                                 label='Inertia_x',
                                 linewidth=2)
#
    line_inertia_y, = ax2_r.plot([], [],
                                 color='#32CD32',  # 亮绿色（Y轴传统色）
                                 linestyle='-',  # 实线
                                 marker='^',  # 三角形标记
                                 markersize=5,
                                 markevery=5,  # 每隔5个点显示一个标记
                                 label='Inertia_y',
                                 linewidth=2)
#
    line_inertia_z, = ax2_r.plot([], [],
                                 color='#1E90FF',  # 道奇蓝（Z轴传统色）
                                 linestyle=':',  # 点线
                                 marker='o',  # 圆形标记
                                 markersize=4,
                                 markevery=5,
                                 label='Inertia_z',
                                 linewidth=2)
    ax2_r.tick_params(axis='y', labelcolor='g')
    ax2_r.set_ylim(-20,80)

    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Drag (m)', color='r')
    line_Drag, = ax3.plot([], [], 'r-', label='Drag', linewidth=2)
    ax3.tick_params(axis='y', labelcolor='r')
    ax3.set_ylim(-5, 400)
    ax3.grid(True)

    ax3_r = ax3.twinx()
    ax3_r.set_ylabel('Angular_Speed (rad/s)', color='b')
    line_Angular_velocity, = ax3_r.plot([], [], 'b--', label='Angular_Speed', linewidth=2)
    ax3_r.tick_params(axis='y', labelcolor='b')
    ax3_r.set_ylim(-1, 10)
    # ========== 统一设置 ==========
    for ax in [ax1, ax2,ax3]:
        ax.set_xlim(0,Time_Duration)

    # ========== 合并图例 ==========
    lines = [line_height, line_velocity,
             line_pos_x, line_pos_y, line_inertia,line_inertia_x,line_inertia_y,line_inertia_z,
             line_Drag,line_Angular_velocity]
    labels = [line.get_label() for line in lines]
    fig.legend(lines, labels, loc='upper right',
               bbox_to_anchor=(0.8, 1), ncol=2, fontsize=10)

    # 初始化cursor（放在update函数外部，避免重复创建）
    cursor = mplcursors.cursor(lines, hover=False)  # 设置为点击触发

    @cursor.connect("add")
    def on_add(sel):
        # 设置注解文本
        sel.annotation.set_text(
            f"{sel.artist.get_label()}\nTime: {sel.target[0]:.2f}s\nValue: {sel.target[1]:.2f}"
        )

        # 确保注释保持可见
        sel.annotation.set_visible(True)

        # 获取注解的 Figure 对象并触发重新绘制
        #sel.annotation.figure.canvas.draw_idle()
    def update(frame):
        if data_monitor.paused:  # 暂停时不更新
            return lines
        with data_monitor.lock:
            global xmin,xmax
            if len(data_monitor.time_data) == 0:
                return lines

            t = np.array(data_monitor.time_data)
            current_time = t[-1]
            data = {
                'time': t,
                'height': np.array(data_monitor.height_data),
                'velocity': np.array(data_monitor.velocity_data),
                'pos_x': np.array(data_monitor.pos_x_data),
                'pos_y': np.array(data_monitor.pos_y_data),
                'inertia': np.array(data_monitor.inertia_data),
                'inertia_x': np.array(data_monitor.inertia_x_data),
                'inertia_y': np.array(data_monitor.inertia_y_data),
                'inertia_z': np.array(data_monitor.inertia_z_data),
                'drag': np.array(data_monitor.drag_data),
                'angular_velocity': np.array(data_monitor.Angular_speed_data),
            }
            # 更新X轴范围
            x_min = max(0, current_time - Time_Duration)
            x_max = max(Time_Duration, current_time)

            for ax in [ax1, ax2,ax3]:
                ax.set_xlim(x_min, x_max)
                ax.relim()  # 重新计算数据范围
                ax.autoscale_view()  # 自动调整视图
            for line in lines:
                xdata, ydata = line.get_data()
                if len(xdata) > 0:
                    x, y = xdata[-1], ydata[-1]
                    # 如果已经有注释，更新它
                    if hasattr(line, 'annotation'):
                        line.annotation.xy = (x, y)
                        line.annotation.set_text(f"{line.get_label()}")
                    elif not hasattr(line, 'annotation') or line.annotation is None:
                        line.annotation = line.axes.annotate(
                            f"{line.get_label()}",
                            xy=(x, y),
                            xytext=(5, 5),
                            textcoords='offset points',
                            fontsize=10,
                            color=line.get_color(),
                            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.5),
                        )

            # 更新曲线数据
            line_height.set_data(data['time'], data['height'])
            line_velocity.set_data(data['time'], data['velocity'])
            line_pos_x.set_data(data['time'], data['pos_x'])
            line_pos_y.set_data(data['time'], data['pos_y'])
            line_inertia.set_data(data['time'], data['inertia'])
            line_inertia_x.set_data(data['time'], data['inertia_x'])
            line_inertia_y.set_data(data['time'], data['inertia_y'])
            line_inertia_z.set_data(data['time'], data['inertia_z'])
            line_Drag.set_data(data['time'], data['drag'])
            line_Angular_velocity.set_data(data['time'], data['angular_velocity'])
        return lines

    ani = FuncAnimation(
        fig,
        update,
        blit=False,
        interval=300,
        cache_frame_data=False  # 避免内存累积
    )

    plt.tight_layout()
    plt.subplots_adjust(right=0.85, top=0.85)  # 调整布局
    plt.show()

# 初始化数据监控器
data_monitor = DataMonitor()