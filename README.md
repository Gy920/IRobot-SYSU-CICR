# IRobot-CICR-2023 比赛提交代码

## 参赛情况

### 1. 比赛详情

[第一届"逸仙勇士杯"协同机器人国际邀请赛](https://github.com/SYSU-STAR/CICRSIM?tab=readme-ov-file)

### 2. 参赛人员

 [Guiyong Zheng](https://github.com/Gy920), [Bihao Zhang](https://github.com/BH0924), [Weilv Chi](https://github.com/shitoujie), [Yuhao Lu](https://github.com/Rok2002)

### 3. 获奖情况

仿真赛成功提交晋级后，参与实机赛获得二等奖（2023.08）

## 提交说明

- 运行系统为Ubuntu20.04
- ROS 环境为 ROS1 noetic

###  1. Dependencies

- **CICRSIM**

  - Install

    ```shell
    sudo apt-get install ros-noetic-mavlink python3-wstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev ros-noetic-mavros python3-pygame
    ```

  - **具体配置请参考src/CICRSIM/README.md**

- **FUEL**

  - Install

    ```shell
    sudo apt-get install libarmadillo-dev  libdw-dev
    ```

  - build nlopt 

    ```shell
    wget https://github.com/stevengj/nlopt/archive/v2.7.1.tar.gz -O nlopt-2.7.1.tar.gz
    tar -zxvf  nlopt-2.7.1.tar.gz
    cd nlopt-2.7.1 
    mkdir build && cd build
    cmake ..
    make -j9
    sudo make install 
    ```

### 2. Build

  ``` shell
  catkin build
  ```

- 由于不同包之间的msg存在依赖关系，可能第一次build会失败，再build一次就可以了


###  3. Run

- 一键开启

  ``` shell
  cd <fuel/workspace>
	sh ./script/test.sh
  ```


- 在出现的rviz中使用2D Nav goal 进行裁判系统和探索的触发
