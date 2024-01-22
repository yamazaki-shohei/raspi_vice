* 起動
```
$ roslaunch vice vice.launch
```

* 25mm 閉じる
```
$ rosservice call /vice_control/controller "position_x: 2500
position_y: 2500"
```

* 原点復帰（開く）
```
$ rosservice call /vice_control/controller "position_x: 0
position_y: 0"
```
