#!/usr/bin/env python3

import rospy
import socket
from std_msgs.msg import String

HOST = '127.0.0.1'  # IP 地址
PORT = 8080         # 进程端口号

def main():
    # 初始化 ROS 节点
    rospy.init_node('yolo_result_publisher', anonymous=True)
    pub = rospy.Publisher('/yolo_results', String, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # 初始化socket,并绑定特定的ip与端口号
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        print(f"Connected to {HOST}:{PORT}...")

        while not rospy.is_shutdown():
            c, addr = server_socket.accept()
            with c:
                data = c.recv(1024).decode('utf-8')  # 接收数据
                if data:
                    print(f"Received: {data}")
                    # 发布到 ROS 话题
                    pub.publish(data)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
