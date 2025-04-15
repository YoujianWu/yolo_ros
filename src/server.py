#!/usr/bin/env python3

import rospy
import socket
from std_msgs.msg import String

HOST = '127.0.0.1'  # IP 地址
PORT = 8080         # 进程端口号

def main():
    # 初始化 ROS 节点
    rospy.init_node('yolo_result_publisher')
    pub = rospy.Publisher('/yolo_results', String, queue_size=10)
    rate = rospy.Rate(100)

    # 初始化socket,并绑定特定的ip与端口号
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        print(f"Connected to {HOST}:{PORT}...")

        while not rospy.is_shutdown():
            c, addr = server_socket.accept()
            with c:
                # 因为只接受字节传输，要utf-8解码，否则会是字符串乱码
                data = c.recv(1024).decode('utf-8')  # 接收数据
                if data:
                    # print(f"Received: {data}")
                    pub.publish(data)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
