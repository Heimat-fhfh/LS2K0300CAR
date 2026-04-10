# 第二十一届智能车竞赛龙芯组别源代码仓库

### 车模信息
编码器齿轮：30齿
电机齿轮：68齿

车轮直径：6.5cm

编码器读数单位：0.1 rps

实际速度 = 0.00901 X (编码器读数) (m/s)

### 上位机图像处理测试程序

该测试程序用于在服务器或上位机上离线测试图像处理相关功能，不部署到边缘设备。

#### 编译

在仓库根目录执行：

```sh
cmake -S . -B build_host -DUSE_EDGE_TOOLCHAIN=OFF
cmake --build build_host -j 2 --target image_web_test
```

#### 运行

在仓库根目录执行：

```sh
./build_host/image_web_test --dataset document/img/20260409_141917 --config config/config_0.json --port 8090
```

#### 访问

浏览器打开：

```sh
http://127.0.0.1:8090
```

如果你在远程服务器上访问，请将 `127.0.0.1` 替换成服务器的实际 IP 地址。