### CRX-5iA を使う為の準備

#### driver

```
$ roslaunch fanuc_ros_driver robocip_test.launch 
```

#### RViz

```
$ roslaunch fanuc_ros_driver test_crx5ia_rviz.launch
```

RViz のウィンドウで [ Add ] ボタンをクリックして [ MotionPlanning ] を追加する。
ロボットの非常停止ボタンを解除して、アラームを解除すると、RViz 上に表示されているロボットと実機のロボットは同じ姿勢になる。


### sequence_all.py を使う為の準備

#### 使用している USB ポートの確認

```
$ ls /dev/ttyUSB*
/dev/ttyUSB0  /dev/ttyUSB1  /dev/ttyUSB2  /dev/ttyUSB3

$ utils/usb_port.sh 
/dev/ttyUSB0 DynPick
/dev/ttyUSB1 ROBOTIQ 2F-140
/dev/ttyUSB2 Digital scale
/dev/ttyUSB3 DPA-06
```

#### ロボットのハンド

DynPick と ROBOTIQ 2F-140 が使用している USB のポートは utils/usb_port.sh で得た結果を使用する。

```
$ roslaunch robot_hand robot_hand.launch robotiq_device:=/dev/ttyUSB1 dynpick_device:=/dev/ttyUSB0
```

ハンドの操作を行う。  
rostopic コマンド入力する代わりに rqt で gui による操作も可。  

```
activation
$ rostopic pub -1 /robot_hand_control std_msgs/String "data: 'initialize'"

open
$ rostopic pub -1 /robot_hand_control std_msgs/String "data: 'o'"

close (distance : mm)
$ rostopic pub -1 /robot_hand_control std_msgs/String "data: '80'"

close (force : N)
$ rostopic pub -1 /robot_hand_control std_msgs/String "data: 'c0'"

```

#### ロボットのアーム

```
$ roslaunch robot_arm robot_arm.launch
```

#### 電子秤

電子秤が使用している USB のポートは utils/usb_port.sh で得た結果を使用する。

```
$ roslaunch digital_scale digital_scale.launch device:=/dev/ttyUSB2
```

#### バイス

```
$ roslaunch vice vice.launch
```

バイスの操作を行う場合は、ロボットの非常停止ボタンは解除しておくこと。
非常停止ボタンが押されている状態は、バイスも非常停止中である。

x, y それぞれの閉じ幅を指示して操作を行う。  
rostopic コマンド入力する代わりに rqt で gui による操作も可。  

```
x=0mm, y=0mm (open)
$ rosservice call /vice_control/controller "position_x: 0 
position_y: 0" 

x=15mm, y=10mm
$ rosservice call /vice_control/controller "position_x: 1500
position_y: 1000" 
```

午後の紅茶  
x=2300, y=2300  

森永チョコチップクッキー  
x=2300, y=1200  

ロッテコアラのマーチ  
x=2600, y=1400  

日清カップヌードル  
x=1500, y=1500  

明治おいしい牛乳 450ml, 900ml
x=1500, y=1500

オレオ
x=2800, y=1200

ドレッシング
x=2300, y=1500

### sequence_all.py の実行

```
$ rosrun sequence_and_interface sequence_all.py
```

