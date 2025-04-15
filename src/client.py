#!~/anaconda3/envs/my_jupyter/bin/python python3

# 需要在anaconda3环境里面跑这个脚本

import os
import socket
from ultralytics import YOLO
import cv2

# 配置参数
HOST = '127.0.0.1' # IP 地址
PORT = 8080        # 进程端口号
TEST_DIR = 'data_sets/test/images'   # 测试集图片存放路径
MODEL_PATH = 'data_sets/bestW.pt'  # 训练好的权重文件路径

def send_result_via_socket(result):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT))
            s.sendall(result.encode('utf-8'))
            print(f"Sent result: {result}")
        except Exception as e:
            print(f"Socket error: {e}")

def main():
    model = YOLO(MODEL_PATH)
    # 获取图片路径列表
    test_images = [
        os.path.join(TEST_DIR, img) 
        for img in os.listdir(TEST_DIR) 
        if img.endswith(('.png', '.jpg', '.jpeg'))
        ]

    # 用cv2读取图片，遍历图片,交给yolo识别
    for img_path in test_images:
        # 读取图片
        img = cv2.imread(img_path)
        if img is None:
            print(f"Failed to load image: {img_path}")
            continue

        # 使用 bestW.pt 模型进行预测
        results = model(task='dectet',mode='predict',source=img)

        # 解析结果
        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())
            bbox = box.xyxy.tolist()[0]  # 边界框坐标 [x1, y1, x2, y2]
            detections.append({
                "class": cls_id,
                "confidence": conf,
                "bbox": bbox
            })

        # 构造识别结果字符串
        result_str = f"Image: {os.path.basename(img_path)}, Detections: {detections}"

        # 发送结果
        send_result_via_socket(result_str)

if __name__ == "__main__":
    main()