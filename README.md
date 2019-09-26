# proto2

## Overview
プロト２号は、二足歩行ロボット格闘技大会ROBO-ONEおよびROBO-ONE auto用に製作されたロボットです。ゲームコントローラによる手動制御、物体認識を使った自律制御が可能です。

## System Requirements
- Raspberry Pi 3b+
- Raspbain Stretch
- Docker CE 18.09.0

## Installation
- 以下のコマンドでワークスペースを作成し、このリポジトリをクローンします。
  ```sh
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/Spritaro/proto2.git
  ```
- Dockerイメージを作成します。Raspberry Pi 3b+では半日〜１日ほどかかります。
  ```sh
  $ cd ~/catkin_ws/src/proto2/proto2_dockerfiles
  $ docker build -t ros-cpp-tf .
  ```
- Dockerコンテナを起動した後、ワークスペースをビルドします。
  ```sh
  $ cd ~/catkin_ws/src/proto2/proto2_dockerfiles
  $ ./run.sh
  $ cd ~/catkin_ws
  $ catkin_make
  ```

## Usage
- 手動制御モード
  - 起動方法
    ```sh
    $ roslaunch proto2_bringup bringup_vr.launch
    ```

- ROBO-ONE auto規格審査モード
  - 起動方法
    ```sh
    $ roslaunch proto2_bringup bringup_preliminary.launch
    ```

- ROBO-ONE auto試合モード
  - 起動方法
    ```sh
    $ roslaunch proto2_bringup bringup_battle.launch
    ```
