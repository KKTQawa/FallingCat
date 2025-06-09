import os
import pybullet
import numpy as np
class Log:
    def __init__(self):
        self.init_log_dir()

    def init_log_dir(self):
        output_dir = os.path.expanduser("../report")
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        log_name = "test_cat_report"
        log_count = 0
        while True:
            self.log_dir = os.path.join(output_dir, f"{log_name}_{log_count}.txt")
            if not os.path.exists(self.log_dir):
                break
            log_count += 1

    def save(self,m_total_impulse,m_total_torque,m_v,m_w,m_drag,m_time,m_Sxy,m_h,m_m,m_x,m_y,m_f):
        scores=["猫表现良好","猫乳臭未干","猫表现一般","测试无效！"]
        res=["猫粉身碎骨","猫凶多吉少","猫并无大碍，可以挑战更高难度！"]
        sco=0
        m_scores=scores[0]
        m_res=res[0]
        m_tmp=m_v*m_w*m_f
        sco+=m_h*m_drag*1000000000/np.linalg.norm(m_total_torque)/m_tmp
        if m_f<1300:
            m_res=res[2]
        elif m_f<2000:
            m_res=res[1]
        else:
            m_res = res[0]
        with open(self.log_dir, 'w', encoding='utf-8') as f:
            content = f"""落体猫运动---下落报告：
        总评：{sco}
        单次下落高度：{m_h}m
        单次下落时长：{m_time}s
        猫初始朝向：面朝y轴正方向
        猫的质量：{m_m}kg
        猫的体型：长约0.3m，宽约0.2m，高约0.15m
        下落过程xy轴最大偏移值(区分正负)({m_x},{m_y})
        下落最大线速度：{m_v}m/s
        下落最大角速度：{m_w}rad/s
        下落竖直方向最大线性阻力：{m_drag}N
        下落过程最大投影面积：{m_Sxy}平方米
        落地瞬时最大冲量：{m_total_impulse}N*s
        落地瞬时冲击力：{m_f}N
        落地瞬时最大力矩：({m_total_torque[0]}, {m_total_torque[1]}, {m_total_torque[2]})
        
        {m_res}
        
        注意：实际生活中猫在接触地面后还会自行调整以增加落地接触时间减小接触力。此处只是展示落地瞬间猫受到的冲击力。如果冲击力为0，说明测试无效！
        """
            f.write(content)
        print(f"报告已保存至：{self.log_dir}")


