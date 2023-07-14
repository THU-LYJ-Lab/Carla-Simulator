# carla

[官方文档](http://carla.org/)

## Ubuntu配置

1.进入[Ubuntun官网](https://cn.ubuntu.com/download/desktop)，下载光盘映像文件.iso

2.下载[Win32DiskImager ](https://sourceforge.net/projects/win32diskimager/)，按照默认安装

3.准备一个空的U盘（有数据提前备份，然后删除）

4.打开Win32DiskImager，将下载的.iso文件写入U盘

  此时U盘处于不可用的状态，失去了储存数据的功能，待安装完成可以回复。（或者使用ventoy，可以同时储存数据，但可能会损坏u盘）

5.磁盘分区，给Ubuntu留至少130g的空间，当然越大越好，并且查看磁盘格式是GPT，还是MBR

6.在U盘插入的情况下，关机，进入BIOS（不同电脑主板可能会不一样，可以百度查询，acer，ASUS，等大部分电脑是f2），根据之前查找的磁盘格式，设置对应的启动模式，如果是GPT则为UEFI，MBR为Legacy。

7.关闭安全启动，选择boot，使用u盘进入

8.按照默认，或者特定需求安装

### Ubuntu 未发现WiFi适配器

检查是否有无线网卡



### Ubuntu 的重装

插入u盘，再次进入即可。



### BIOS下Secure Boot显示为灰色





## 配置carla

### dpkg: 处理软件包 xxx (--configure)时出错

```
\#先切换到root用户

sudo su         

 mv /var/lib/dpkg/info   /var/lib/dpkg/info_bak

mkdir /var/lib/dpkg/info

apt-get update && apt-get -f install 

mv /var/lib/dpkg/info/*    /var/lib/dpkg/info_bak/

rm -rf /var/lib/dpkg/info

mv /var/lib/dpkg/info_bak /var/lib/dpkg/info


```



### s段错误 (核心已转储)

在下载预编译文件时

```
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/Dev/CARLA_Latest.tar.gz
```

到99%时报错，s段错误 (核心已转储)，但是后续无影响。

原因未知



### fatal: unable to access 'https://github.com/xxx': Failed to connect to github.com port 443: Operation timed out

可能是没梯子的缘故，改用预编译文件

```
https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/Dev/CARLA_Latest.tar.gz
```



### File "generate_traffic.py", line 165, in main     world = client.get_world() RuntimeError: time-out of 10000ms while waiting for the simulator, make sure the simulator is ready and connected to 127.0.0.1:2000

原因：未打开CarlaUE4.sh，

解决：打开CarlaUE4.sh，再打开所需.py





### Spawn failed because of collision at spawn position

解决：

此消息只是一个警告，某些车辆或某些步行者（在本例中可能是步行者）尚未生成，因为随机位置太靠近另一个步行者之一。所以基本上不用管。



### Eeception "Disabling core dumps" was thrown while running "./CarlaUE4.sh"

解决：

This is not an error but a message that tells you that core dumps are disabled. This is expected and happens to everyone running Carla with default options.

这不是错误，而是一条消息，告诉您核心转储已禁用，不用管



### incompatible vulkan driver found

原因:nvidia显卡驱动出问题

解决：[(25条消息) Cannot find a compatible Vulkan driver (ICD). Please look at the Getting Started guide for additiona_DBzs的博客-CSDN博客](https://blog.csdn.net/weixin_43290709/article/details/121737146)

nvidia-smi 查看是否报错，若是重装驱动。

```
报错为
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver. Make 
sure that the latest NVIDIA driver is installed and running. This can also be 
happening if non-NVIDIA GPU is running as primary display, and NVIDIA GPU is in 
WDDM mode.

```

```
$ sudo apt-get --purge remove nvidia-*
$ sudo apt autoremove
$ sudo ubuntu-drivers install
```



## anoconda

1.官网下载https://www.anaconda.com/

2.运行.sh文件



```
conda create -n xxx python=x.x
conda install -n your_env_name [package]
```





```
sudo apt-get update &&
sudo apt-get install wget software-properties-common &&
sudo add-apt-repository ppa:ubuntu-toolchain-r/test &&
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add - &&
sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main" &&
sudo apt-get update


sudo apt-get install build-essential clang-8 lld-8 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git


sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180


conda create -n carla python=3.7
conda activate carla


pip install --user -Iv setuptools==47.3.1 &&
pip install --user distro &&
pip install --user wheel auditwheel


pip install --user pygame numpy


wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/Dev/CARLA_Latest.tar.gz


mkdir CARLA
tar -zxvf CARLA_Latest.tar.gz -C CARLA
cd CARLA
./ImportAssets.sh


cd ../../.. # 回到根目录
./CarlaUE4.sh


cd PythonAPI/examples
python -m pip install -r requirements.txt # Support for Python2 is provided



cd PythonAPI/carla/dist
ls # 查看python包
pip install carla-0.9.13-cp37-cp37m-manylinux_2_27_x86_64.whl

```

