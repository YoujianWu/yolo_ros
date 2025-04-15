### 《Linux操作系统与应用》课程的作业

1、下载Anaconda：
wget https://repo.anaconda.com/archive/Anaconda3-2024.10-1-Linux-x86_64.sh

2、安装Anaconda

```shell
bash Anaconda3-2024.10-1-Linux-x86_64.sh
```

过程中一直yes到底，安装完后执行：

```shell
source ~/.bashrc
```

3、安装pytorch
CPU版本

```shell
pip install torch torchvision torchaudio
```

GPU版本（以CUDA11.8为例）

```shell
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

4、安装Yolo V8 （是ultralytics公司设计的算法）

```shell
pip install ultralytics
```

5、如果想自己重新训练一遍，那么在提供的图片集文件夹data_set中训练

```shell
yolo task=detect mode=train model=yolov8n.pt data=data.yaml epochs=100 imgsz=640 batch=4
```

6、训练完后在runs菜单中找到best.pt（默认名称，我自己修改为bestW.pt），然后复制出来到有测试图片traffic_light.jpg的目录下，鼠标右键弹出菜单进入终端，输入：

```shell
yolo task=detect mode=predict model=bestW.pt source=traffic_light.jpg
```

即可识别traffic_light.jpg图片的结果，结果放在runs目录中。
