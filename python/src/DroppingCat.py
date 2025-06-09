import pybullet as p
import pybullet_data as pd
import time
import threading
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os
import noise
import subprocess
from Recorder import VideoRecorder
from Graph import Graph_thread,data_monitor
from logger import Log
def game_init():
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # 显式启用GUI控件
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setPhysicsEngineParameter(enableConeFriction=0)
    #p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(1)
    p.setTimeStep(1./240)

def load_urdf():
    plane = p.loadURDF("plane.urdf")
    urdfFlags = p.URDF_USE_SELF_COLLISION
    cat = p.loadURDF(
        "laikago/laikago.urdf",
        [0, 0, 0.5],
        [0, 0.5, 0.5, 0],
        flags=urdfFlags,
        useFixedBase=False
    )

    color_rgba = [1.0, 1.0, 0.0, 1.0]
    for i in range(16):
        p.changeVisualShape(cat, i, rgbaColor=color_rgba)
    return  plane,cat
# 获取模型的所有质量信息
def get_model_mass_info(model_id):
    num_joints = p.getNumJoints(model_id)  # 获取关节数量
    mass_info = []  # 存储所有质量信息

    # 获取基座的质量信息
    base_mass, link_local_inertia_diag,*_ = p.getDynamicsInfo(model_id, -1)#主身体为编号-1，返回的第一个数据就是质量
    p.changeDynamics(
        bodyUniqueId=cat,
        linkIndex=-1,
        mass=3,  # 设置质量 (kg)
    )
    mass_info.append(("Base", 3))

    # 获取各关节的质量信息
    for joint_index in range(num_joints):
        joint_name = p.getJointInfo(model_id, joint_index)[1].decode('utf-8')  # 关节名称
        #print(joint_name,joint_index)
        joint_mass,link_local_inertia_diag, *_ = p.getDynamicsInfo(model_id, joint_index)
        # 获取关节在世界坐标系中的位置
        link_state = p.getLinkState(model_id, joint_index)
        joint_pos = link_state[0]  # 世界坐标系中的位置
        i=joint_index
        to_mass=0
        if i%3==0:
            to_mass=0.3
        elif i%3==1:
            to_mass = 0.2
        elif i % 3 == 2:
            to_mass = 0.1
        if i==6:
            to_mass=0.4
        if i==9:
            to_mass=0.4
        p.changeDynamics(
                bodyUniqueId=cat,
                linkIndex=joint_index,
                mass=to_mass,  # 设置质量 (kg)
            )
        mass_info.append((joint_name,to_mass))

    return mass_info
#计算整体质心位置
def compute_com(body_id):
    total_mass = 0.0
    weighted_pos_sum = np.zeros(3)

    # 基座
    mass, _, local_com_pos, *_ = p.getDynamicsInfo(body_id, -1)
    if mass > 0:
        base_pos, base_orn = p.getBasePositionAndOrientation(body_id)
        world_com_base = np.array(p.multiplyTransforms(base_pos, base_orn, local_com_pos, [0, 0, 0, 1])[0])
        weighted_pos_sum += world_com_base * mass
        total_mass += mass

    # 各链接
    for link_idx in range(p.getNumJoints(body_id)):
        mass, _, local_com_pos, *_ = p.getDynamicsInfo(body_id, link_idx)
        if mass <= 0:
            continue

        link_state = p.getLinkState(body_id, link_idx, computeForwardKinematics=True)
        world_com = np.array(p.multiplyTransforms(link_state[0], link_state[1], local_com_pos, [0, 0, 0, 1])[0])
        weighted_pos_sum += world_com * mass
        total_mass += mass

    return weighted_pos_sum / total_mass if total_mass > 0 else np.zeros(3)
#计算整体相对于质心的转动惯量
def compute_inertia_about_point(body_id, ref_point):
    ref_point = np.array(ref_point)
    total_inertia = np.zeros((3, 3))  # 3x3惯性张量

    def add_inertia_contribution(pos, orn, local_inertia, mass):
        # 转换惯性张量到世界坐标系
        rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        world_inertia = rot_matrix @ np.diag(local_inertia) @ rot_matrix.T

        # 平行轴定理
        r = pos - ref_point
        r_skew = np.array([[0, -r[2], r[1]],
                           [r[2], 0, -r[0]],
                           [-r[1], r[0], 0]])
        return world_inertia + mass * (r.T @ r * np.eye(3) - r[:, None] @ r[None, :])

    # 基座贡献
    mass, _, local_com_pos, inertia_diag, *_ = p.getDynamicsInfo(body_id, -1)
    if mass > 0:
        base_pos, base_orn = p.getBasePositionAndOrientation(body_id)
        world_com = np.array(p.multiplyTransforms(base_pos, base_orn, local_com_pos, [0, 0, 0, 1])[0])
        total_inertia += add_inertia_contribution(world_com, base_orn, inertia_diag, mass)

    # 各链接贡献
    for link_idx in range(p.getNumJoints(body_id)):
        mass, _, local_com_pos, inertia_diag, *_ = p.getDynamicsInfo(body_id, link_idx)
        if mass <= 0:
            continue

        link_state = p.getLinkState(body_id, link_idx, computeForwardKinematics=True)
        world_com = np.array(p.multiplyTransforms(link_state[0], link_state[1], local_com_pos, [0, 0, 0, 1])[0])
        total_inertia += add_inertia_contribution(world_com, link_state[1], inertia_diag, mass)

    return total_inertia  # 返回3x3惯性张量

# 初始化
game_init()
logger=Log()
plane,cat = load_urdf()
# 计算总质量
mass_info = get_model_mass_info(cat)
for name, mass in mass_info:
    data_monitor.mass+=mass
# 启动绘图线程
matplotlib.use('TkAgg')
plotter = threading.Thread(target=Graph_thread, daemon=True)
plotter.start()
# 调试参数
jointIds = []
paramIds = []
limits=[]
for j in range(p.getNumJoints(cat)):
    info = p.getJointInfo(cat, j)
    p.changeDynamics(cat, j, linearDamping=0, angularDamping=0.02)
    if info[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]:
        jointIds.append(j)
        paramIds.append(p.addUserDebugParameter(
            info[1].decode(), info[8], info[9],
            p.getJointState(cat, j)[0]
        ))
        limits.append([info[8],info[9]])
paramIds.append(p.addUserDebugParameter(
            "RESET_Height", 30, 400,
            100
        ))
limits.append([30,400])
paramIds.append(p.addUserDebugParameter(
            "Max_Velocity", 30, 150,
            60
        ))
limits.append([30,150])
paramIds.append(p.addUserDebugParameter(
            "Quit", 1,0,0
        ))
paramIds.append(p.addUserDebugParameter(
            "Auto", 0,3,0
        ))
paramIds.append(p.addUserDebugParameter(
            "Begin_Collision", 0,3,0
        ))
# 设置检测参数
RESET_HEIGHT = 100.0  # 重置高度
GROUND_THRESHOLD = 5  # 接近地面的阈值
VELOCITY_THRESHOLD = 60  # 速度阈值，当速度接近地面时重置
#摄像机相关
_,init_base_orientation=p.getBasePositionAndOrientation(cat)
offset=0.5
aabb_min, aabb_max = p.getAABB(cat)
height = aabb_max[2] - aabb_min[2]  # 模型的高度
distance=3

num_links = p.getNumJoints(cat)

yaw=45 #水平旋转
pitch=345 #上下旋转，正值表示上仰
#游戏进程相关
start_time = time.time()
pause=True
simulation_paused_time = 0  # 记录暂停时的总时间偏移量
pause_start_time = 0  # 记录暂停开始的系统时间
#初始化tmp数组
base_position, base_orientation = p.getBasePositionAndOrientation(cat)
base_velocity, base_angular = p.getBaseVelocity(cat)
tmp=[base_position, base_orientation,base_velocity, base_angular]
# 初始化视频录制线程
Recording_file = "droppingCat_recording"
counter = 1
file_dir=None
# 在 Recorder 文件夹下寻找可用的子目录名
while True:
    dir = os.path.join("Recorder", f"{Recording_file}_{counter}")
    if not os.path.exists(dir):  # 如果目录不存在，则退出循环
        file_dir=dir
        break
    counter += 1
#os.makedirs(file_dir)  # 自动创建目录
r_width, r_height = 640, 480  # 输出视频的分辨率
#fps = 30  # 每秒帧数
#video_recorder = VideoRecorder(r_width, r_height,file_dir,Recording_file,p)
#video_recorder.start()
#录像相关
last_shot=0
is_auto=False
#投影面积相关
ray_col_num=50
ray_width=5#投影区域为5*5
ray_delta=ray_width/ray_col_num
rayFromPositions = []
rayToPositions = []
for ray_x in range(-ray_col_num // 2, ray_col_num // 2):
    for ray_y in range(-ray_col_num // 2, ray_col_num // 2):
        x =  ray_x * ray_delta
        y = ray_y * ray_delta
        rayFromPositions.append((x, y, RESET_HEIGHT))
        rayToPositions.append((x, y, 0))

min_Sxy=0.01
max_Sxy=0.5
Sxy=0.05

Allow_Collsion=False

frames=0
# 计算整体质心
com = compute_com(cat)
last_com=com
time_step = 0.1  # 时间步长（控制运动平滑程度）
time_counter = 0  # 用于记录时间
tmp_s=0#用于打印报告
#报告相关
m_total_impulse=0
m_total_torque = np.zeros(3)
m_v=0
m_w=0
m_drag=0
m_x=0
m_y=0
last=0
ds=1
#游戏主循环
while True:
    # 获取机器人基座状态
    base_position, base_orientation = p.getBasePositionAndOrientation(cat)
    base_velocity, base_angular = p.getBaseVelocity(cat)

    current_time = time.time() - start_time
    #计算转速大小
    angular_speed = np.linalg.norm(base_angular)
    #处理质心和惯性张量
    if np.linalg.norm(com - last_com) > 0.05:  # 仅当质心移动>5cm时重新计算
        last_com = com.copy()
    inertia_at_com = compute_inertia_about_point(cat, com)
    #使用射线检测估计投影面积
    if frames%2==0:
        results = p.rayTestBatch(rayFromPositions, rayToPositions)
        Sxy = sum(1 for r in results if r[0] == cat) * ray_delta * ray_delta
        min_Sxy=min(min_Sxy,Sxy)
        max_Sxy=max(max_Sxy,Sxy)
    #计算空气阻力
    if max_Sxy>min_Sxy:
        Cd=0.3+0.9*(Sxy-min_Sxy)/(max_Sxy-min_Sxy)
    else:
        Cd=0.7
    base_velocity_magnitude = np.linalg.norm(base_velocity)
    if base_velocity_magnitude > 1e-6:
        velocity_dir = base_velocity / base_velocity_magnitude  # 单位方向向量
        drag_magnitude = 0.5 * 1.225 * Sxy * Cd * np.dot(base_velocity_magnitude, base_velocity_magnitude)
        drag = -drag_magnitude * velocity_dir  # 阻力方向与速度相反
        # 应用空气阻力
        if not pause:
            p.applyExternalForce(
                objectUniqueId=cat,
                linkIndex=-1,  # 基座
                forceObj=drag.tolist(),  # 转换为列表传入PyBullet
                posObj=p.getBasePositionAndOrientation(cat)[0],
                flags=p.WORLD_FRAME
            )
    else:
        # 如果速度为零，不施加阻力
        drag = np.zeros(3)
    #评估冲击力和力矩
    total_impulse = 0.0
    total_torque = np.zeros(3)
    contacts = p.getContactPoints(bodyA=cat,bodyB=plane)
    base_pos = np.array(base_position)
    if contacts:
        contact_pos = np.array([c[5] for c in contacts])  # 所有接触点位置
        normals = np.array([c[7] for c in contacts])  # 所有法向量
        impulses = np.array([c[9] for c in contacts])  # 所有冲量

        force_vecs = normals * impulses[:, None]  # 向量化计算
        r_vectors = contact_pos - base_pos
        torques = np.cross(r_vectors, force_vecs)

        total_impulse = np.sum(impulses)
        total_torque = np.sum(torques, axis=0)
        if Allow_Collsion and not pause and base_position[2]<1:
            if tmp_s==0:
                last=current_time
            if total_impulse>=m_total_impulse:
                m_total_impulse=total_impulse
                m_total_torque=total_torque
            m_v=max(base_velocity_magnitude,m_v)
            m_w=max(angular_speed,m_w)
            m_drag=max(m_drag,abs(drag[2]))
            if (m_x**2+m_y**2)<(base_position[0]**2+base_position[1]**2):
                m_x=base_position[0]
                m_y=base_position[1]
            tmp_s+=1
            if tmp_s>5:
                print("ds:",ds)
                m_time=current_time-simulation_paused_time
                ds=current_time-last
                total_force=total_impulse/ds
                m_Sxy=Sxy
                m_h=RESET_HEIGHT
                m_m=data_monitor.mass
                logger.save(m_total_impulse,m_total_torque,m_v,m_w,m_drag,m_time,m_Sxy,m_h,m_m,m_x,m_y,total_force)
                time.sleep(30)
                break
    # 获取键盘事件
    events = p.getKeyboardEvents()
    if ord('d') in events and events[ord('d')] & p.KEY_IS_DOWN:
        yaw += 5  # 增加5度
    if ord('a') in events and events[ord('a')] & p.KEY_IS_DOWN:
        yaw -= 5  # 减少5度
    if ord('q') in events and events[ord('q')] & p.KEY_IS_DOWN:
        pitch += 5  # 增加5度
    if ord('e') in events and events[ord('e')] & p.KEY_IS_DOWN:
        pitch -= 5  # 减少5度
    if ord('f') in events and events[ord('f')] & p.KEY_IS_DOWN:
        distance += 0.3  # 增加距离
    if ord('j') in events and events[ord('j')] & p.KEY_IS_DOWN:
        distance = max(distance - 0.3, 1)  # 减少距离，最小值为2
    if ord('b') in events and events[ord('z')] & p.KEY_IS_DOWN:
        offset+=0.2
    if ord('n') in events and events[ord('c')] & p.KEY_IS_DOWN:
        offset-=0.2
    if ord('r') in events and events[ord('r')] & p.KEY_WAS_TRIGGERED:
        offset=0.5
        distance = 3
        yaw = 45
        pitch = 345
    if ord(' ') in events and events[ord(' ')] & p.KEY_IS_DOWN:
        if pause==False:
            pause_start_time =current_time   # 记录暂停开始时间
            tmp=[base_position, base_orientation,base_velocity, base_angular]
            p.resetBaseVelocity(cat, (0,0,0), init_base_orientation)
            p.setGravity(0, 0, 0)
        else:
            simulation_paused_time +=(current_time  - pause_start_time)
            p.setGravity(0, 0, -9.8)
            base_velocity, base_angular=tmp[2],tmp[3]
            p.resetBaseVelocity(cat, tmp[2], tmp[3])
        pause=not pause
        data_monitor.paused=pause
    # 重置摄像机视角
    p.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=(base_position[0], base_position[1], base_position[2] + height + offset)
    )
    #if not pause and current_time - simulation_paused_time - last_shot >= 1:  # 每秒截图一次
    #    last_shot += 1
    #    video_recorder.add_frame({"distance":distance,"yaw":yaw,"pitch":pitch,"base_position":base_position,"height":height,"offset":offset,"last_shot":last_shot})

    # 获取旋转方向（角速度方向）
    angular_velocity = np.array(base_angular)
    if angular_speed  > 1e-8:  # 避免零向量归一化
        rotation_direction = angular_velocity / np.linalg.norm(angular_velocity)  # 单位方向向量
         # 转动惯量
        rotational_inertia = np.einsum('i,ij,j', rotation_direction, inertia_at_com, rotation_direction)
    else:
        rotational_inertia = 0  # 如果角速度为零，转动惯量设为零

    # 更新监控数据
    #print("jjj",inertia_at_com[0][0]/1000,"   ",inertia_at_com[1][1]/1000)
    data_monitor.update(current_time-simulation_paused_time, base_position[2], -base_velocity[2],
                        rotational_inertia/1000,[inertia_at_com[0][0]/1000,inertia_at_com[1][1]/1000,inertia_at_com[2][2]/1000] ,
                        com[0], com[1],
                        abs(drag[2]),angular_speed
                        )

    # 检测是否需要重置
    if base_position[2] <= GROUND_THRESHOLD and not Allow_Collsion:
        # 重置位置和速度
        p.resetBasePositionAndOrientation(cat, [base_position[0], base_position[1], RESET_HEIGHT],
                                          base_orientation)
        p.resetBaseVelocity(cat, (0, 0, 0), init_base_orientation)
    if -base_velocity[2] >= VELOCITY_THRESHOLD:
        print("程序结束，因为超速了。。。")
        time.sleep(10)
        break

    # 应用滑动条控制关节
    for i in range(len(jointIds)):
        targetpos = p.readUserDebugParameter(paramIds[i])
        if is_auto:
            ma = limits[i][1]
            mi = limits[i][0]
            leng = ma - mi

            # 使用Perlin噪声生成平滑随机值
            random_value = noise.pnoise1(time_counter)
            random_value = random_value * leng / 2  # 缩放到指定范围

            targetpos += random_value
            targetpos = max(mi, targetpos)
            targetpos = min(ma, targetpos)

            time_counter += time_step#当time_counter值不变的时候，返回的随机值不会变化

        p.setJointMotorControl2(
            bodyIndex=cat,
            jointIndex=jointIds[i],
            controlMode=p.POSITION_CONTROL,
            targetPosition=targetpos,
            force=7
        )
    now_reset_height=p.readUserDebugParameter(paramIds[len(paramIds)-5])
    RESET_HEIGHT=now_reset_height
    now_max_v = p.readUserDebugParameter(paramIds[len(paramIds) - 4])
    VELOCITY_THRESHOLD=now_max_v
    if p.readUserDebugParameter(paramIds[len(paramIds) - 3]):
        break
    Auto=p.readUserDebugParameter(paramIds[len(paramIds) - 2])
    if Auto<1:
        is_auto=False
    elif Auto>2:
        is_auto = True
    Allow= p.readUserDebugParameter(paramIds[len(paramIds) - 1])
    if Allow<1:
        Allow_Collsion=False
    elif Allow>2:
        Allow_Collsion = True

    frames+=1
    # 每次循环的时间间隔
    time.sleep(1. / 240.)

#获取当前脚本所在目录
current_dir = os.getcwd()
# 构建批处理文件的完整路径
bat_file = os.path.join(current_dir, "save.bat")
# 异步启动批处理文件，不等待其完成
process = subprocess.Popen(
    [bat_file],
    cwd=current_dir,
    creationflags=subprocess.CREATE_NO_WINDOW
)

print("累计暂停时长",simulation_paused_time)
print("累计运行时长:",current_time-simulation_paused_time)
# 停止视频录制线程
#video_recorder.stop()
#video_recorder.join()
p.disconnect()
