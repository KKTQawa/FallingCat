import threading
import numpy as np
import cv2
import os
import queue
# 视频录制线程类
class VideoRecorder(threading.Thread):
    def __init__(self, width, height, dir, filename,p):
        super().__init__()
        self.p=p
        self.width = width
        self.height = height
        self.dir = dir
        self.filename = filename
        self.frame_queue = queue.Queue(maxsize=30)  # 设置适当的队列大小
        self.running = False
        self.daemon = True  # 设置为守护线程，主程序退出时自动结束
        self.counter = 0  # 添加计数器

    def run(self):
        self.running = True  # 必须先设置运行标志

        while self.running or not self.frame_queue.empty():  # 运行中或还有待处理帧
            try:
                # 从队列获取原始数据
                data = self.frame_queue.get(timeout=0.1)

                # 计算视图矩阵
                view_matrix = self.p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=(data['base_position'][0],
                                          data['base_position'][1],
                                          data['base_position'][2] + data['height'] + data['offset']),
                    distance=data['distance'],
                    yaw=data['yaw'],
                    pitch=data['pitch'],
                    roll=0,
                    upAxisIndex=2,
                )

                # 计算投影矩阵
                projection_matrix = self.p.computeProjectionMatrixFOV(
                    fov=60, aspect=self.width / self.height, nearVal=0.1, farVal=100.0
                )

                # 获取图像
                _, _, rgb, _, _ = self.p.getCameraImage(
                    self.width, self.height,
                    viewMatrix=view_matrix,
                    projectionMatrix=projection_matrix
                )

                # 转换为OpenCV格式并保存
                rgb_array = np.reshape(rgb, (self.height, self.width, 4)).astype(np.uint8)
                bgr_array = cv2.cvtColor(rgb_array[:, :, :3], cv2.COLOR_RGB2BGR)

                # 生成文件名并保存
                filename = os.path.join(self.dir, f"{self.filename}_{self.counter}.png")
                cv2.imwrite(filename, bgr_array)
                self.counter += 1


            except queue.Empty:
                if not self.running:  # 如果已经停止且队列为空
                    break
                continue
        print(f"截图已保存到: {self.dir}")

    def start(self):
        self.running = True
        super().start()  # 调用父类的start方法

    def add_frame(self, data):
        """添加帧数据到队列"""
        if self.running and not self.frame_queue.full():
            self.frame_queue.put(data)

    def stop(self):
        self.running = False