# turtle_sample2

## 概要

## デバッグコマンド

## フォルダ構成

## パッケージ作成
> ros2 pkg create <パッケージ名> --build-type <ビルドタイプ> --dependencies <依存パッケージ>

例：
> ros2 pkg create turtle_publish --build-type ament_cmake --dependencies rclcpp std_msgs geometry_msgs

## ビルド
> cd ~/<ワークスペース>  
> colcon build

例：
> cd ~/turtle_ws  
> colcon build

## 実行
> ros2 run <パッケージ名> <ノード名>

例：
> ros2 run turtle_publish turtle_publish

## パッケージ確認順
1. turtle_publish
2. turtle_subscribe
3. turtle_client
4. turtle_launch