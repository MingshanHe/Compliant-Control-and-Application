FROM consol/ubuntu-xfce-vnc

# 切换到root，root才有权限进行安装软件等操作
USER 0

# 编辑sources.list，使用国内软件源
# 根据自己需求安装一些linux工具，如ping、tftp
RUN cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
    sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y iputils-ping tftp lsb-core && \
    apt-get clean
    
# 安装ROS及其编译工具，配置ROS环境
# 安装turtlesim功能包
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get -y update && \
    apt-get -y install ros-kinetic-desktop-full --allow-unauthenticated && \
    apt-get -y install ros-kinetic-turtlesim --allow-unauthenticated && \
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
    apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential libblas-dev liblapack-dev --allow-unauthenticated && \
    apt-get clean
    
# 在生成镜像之前切换回default
#USER 1000
# 添加docker用户并默认是使用docker用户密码也是docker
RUN apt-get update &&\
	apt-get -y install sudo
RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo
USER docker
CMD /bin/bash
