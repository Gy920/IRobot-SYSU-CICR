# Irobot提交说明

- 运行系统为Ubuntu20.04
- ROS 环境为 ROS1 noetic

###  1. Dependencies

- CICRSIM

  - Install

    ```shell
    sudo apt-get install ros-noetic-mavlink python3-wstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev ros-noetic-mavros python3-pygame
    ```

  - 具体配置请参考src/CICRSIM/README.md

- FUEL

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

- 命令行1: 开启仿真器和裁判系统
  ``` shell
	sh ./test.sh
  ```

- 命令行2: 开启探索节点

  ```shell
  source devel/setup.bash && roslaunch exploration_manager  auto_exploration.launch
  ```
  
- 命令行3: 开启检测检点

  ```shell
  source devel/setup.bash;roslaunch apriltag_ros  continuous_detection.launch
  ```

- 命令行4: 开启rviz可视化

  ```shell
  source devel/setup.bash && roslaunch exploration_manager  rviz.launch
  ```


- 在出现的rviz中使用2D Nav goal 进行裁判系统和探索的触发
