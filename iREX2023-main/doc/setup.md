### pymodbus

pymodbus を pip で最新にすると robotiq の modbus で response がない現象が起きる

```
NG : pymodbus 3.5.4
$ python3 -m pip install -U pymodbus

OK : pymodbus 2.1.0
$ sudo apt install python3-pymodbus
```

### ROBOTIQ 2F-140
[robotiq](https://github.com/ros-industrial/robotiq.git)

noetic で使用する為に robotiq.patch を適用する。

### DynPick
[dynpick_driver](https://github.com/tork-a/dynpick_driver.git)

noetic で使用する為に dynpic_driver.patch を適用する。

### Ethernet/IP
[EIPScanner](https://github.com/nimbuscontrols/EIPScanner.git)

バイスで Ethernet/IP を使用している為、インストールしている必要がある。
$HOME/.local/EIPScanner にインストールされている前提としている。

```
$ git clone https://github.com/nimbuscontrols/EIPScanner.git
$ cd EIPScanner/
$ mkdir build
$ cd build/
$ cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local/EIPScanner ..
$ make
$ make install
```

### IAI PCON-CB

* パラメータ

No.84 フィルードバス動作モード : 3 - フル直値モード  
No.90 フィールドバス入出力フォーマット : 0 - 入替えは行わない  


### setup

```
$ mkdir -p ~/workspace/iREX/src
$ cd ~/workspace/iREX/src

$ git clone https://github.com/robocip/iREX2023.git

ROBOTIQ
$ git clone -b kinetic-devel https://github.com/ros-industrial/robotiq.git
$ cd robotiq/
$ $ patch -Np1 -i ../doc/robotiq.patch

DynPick
$ git clone https://github.com/tork-a/dynpick_driver.git
$ cd  dynpick_driver/
$ patch -Np1 -i dynpick_driver.patch

$ cd ~/workspace/iREX/
$ catkin config --skiplist robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_gripper_articulated_msgs robotiq_3f_gripper_control robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization robotiq_3f_rviz
$ catkin build

```

