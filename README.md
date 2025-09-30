# turtle_sample2

## 概要
ROS2学習のためのturtlesimを用いたサンプルプログラム．
turtle_practiceパッケージを作成して，パッケージ確認順に沿って編集していくことでTopic(publish,subscrive)，Service(client), Parameter, launchの順で学べる

# 座学

## ROS2概要
### Topic
![TOPIC](https://docs.ros.org/en/foxy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

### Service
![SERVICE](https://docs.ros.org/en/foxy/_images/Service-MultipleServiceClient.gif)

### Parameter

### Action
![ACTION](https://docs.ros.org/en/foxy/_images/Action-SingleActionClient.gif)

## パッケージ作成までのコマンド
```sh
# ワークスペース作成
mkdir -p ~/<ワークスペース名>/src
cd <ワークスペース名>
colcon build
source install/setup.bash

# パッケージ作成
cd ~/<パッケージを配置するフォルダ>   
ros2 pkg create <パッケージ名> --build-type <ビルドタイプ> --dependencies <依存パッケージ>
  # ビルドタイプはament_cmake
  # 依存パッケージはrclcpp, std_msgs, ...

# ビルド
cd ~/<ワークスペース名>  
colcon build  
source install/setup.bash

# 実行
ros2 run <パッケージ名> <ノード名>
```

## デバッグコマンド
### Node関係
```sh
# 起動しているNodeの確認
ros2 node list

# Nodeの詳細確認
ros2 node info
```

### Topic関係
```sh
# 存在するTopicの確認
ros2 topic list

# Topicの通信状況の確認
ros2 topic echo <topic名>

# Topicの詳細確認
ros2 topic info <topic名>

# Topicへ送信
ros2 topic pub <topic名> <topic通信の型> <通信データ>
```

### Service関係
```sh
# 存在するServiceの確認
ros2 service list

# Service通信の型の確認
ros2 service type

# Serviceへ送信
ros2 service call <service名> <service通信の型> <通信データ>
```

### Parameter関係
```sh
# Parameterの一覧確認
ros2 param list

# Parameterのセット
ros2 param set <node名> <parameter名> <設定値>

# Parameterの設定値確認
ros2 param get <node名> <parameter名>

# Parameterの保存
ros2 param dump <node名> > <保存ファイルへのパス>　   

# 例
ros2 param dump /turtle_control > ~/turtle_ws/wrc/turtle_sample2/turtle_control/config/params.yaml
```

## 参考
- https://docs.ros.org/en/foxy/Tutorials.html
- https://docs.ros.org/en/foxy/index.html

# 実践編
## フォルダ構成
```sh
~/turtle_ws     
　　/build　　　　# システムトラブル時は削除      
　　/include　　　# システムトラブル時は削除      
　　/src    
　　　　/<パッケージ名>　　# パッケージは必ず/##_ws/src下に配置
　　　　　　/config　　　# parameterファイル置き場   
　　　　　　/launch　　　# launchファイル置き場　   
　　　　　　/src　　　　　#ソースファイル置き場　   
　　　　　　　　/<パッケージ名>.cpp 
        /sample2
            /turtle_publish  # 今回のパッケージ(パッケージはワークスペース名/src配下ならディレクト内でも良い)
                /src
                    /turtle_publish.cpp
                /CMakeLists.txt
                /package.xml
　　　　/CMakeLists.txt     
　　　　/package.xml
```

## パッケージ確認順
1. turtle_publish
2. turtle_subscribe
3. turtle_client
4. turtle_control

## サンプルコードの実行方法
```sh
# ワークスペースの作成
mkdir -p ~/turtle_ws/src     
cd turtle_ws     
colcon build  
source install/setup.bash

# パッケージのクローン
cd turtle_ws/src
git clone https://github.com/Kousuke-Okabe/turtle_sample2.git

# ビルド
cd ~/turtle_ws
colcon build

# 実行
shell-1> source install/setup.bash
shell-1> ros2 run turtle_publish turtle_publish

shell-2> source install/setup.bash
shell-2> ros2 run turtlesim turtlesim_node
```
