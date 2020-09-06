# TUKUBA_2021

## way point specification
つくばチャレンジの選択課題Cの「複数のチェックポイント（前日に番号を通知）をすべて通過し、かつ通りがかった経路封鎖看板（通知なし）を認識して迂回すること」に対するパッケージ

### 実行手順
環境はturtlebot3を使用した. 地図は事前に作成したと仮定した場合の実行手順. 地図作成は以下のURLを参考に行う  
http://joe.ash.jp/program/ros/tutorial/tutorial_slam.htm
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/maps/world_map.yaml
```
```
rosrun waypoint_specification waypoint_specification_node
```

