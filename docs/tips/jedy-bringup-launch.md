# Jedyを起動するjedy_bringup.launch.pyの構成

このlaunchファイルを見てみよう． 付録の「rosemacsの利用」を行うと，
ROSのファイルの参照が行いやすい． jedy_bringup.launch.pyを，
「rosemacsの利用」を読み emacs上で開いてみよう．
エディタとしてemacsを使わない場合は，

<div class="screen">

``` bash
$ roscd jedy_bringup/launch
```

</div>

のようにして該当ディレクトリに移動しつつ，各自の好きなエディタでファイ
ルを開いてもよい．

jedy_bringup.launch.pyを見てみると，

<div class="screen">

      kxr_controller.launch
      ->サーボ基板との通信，ロボットの身体情報定義ファイルを読み込み，他のノードが使えるようにするノード群の立ち上げ
      diff_drive_controller.launch
      ->台車を動かし，情報取得のためのコントローラー立ち上げ
      mpu6886_imu_publisher.launch
      -> Imuをpublishするノードの立ち上げ

</div>

などの複数のノードが立ち上がる． 特にkxr_controller.launchは
UART通信を行う/dev/ttyAML1のデバイスファイルへ
指令値をwriteしセンサ値をreadするノードが起動している．
また，ROS 2のPython launchファイルでは`DeclareLaunchArgument`と`LaunchConfiguration`を使ってlaunchファイルの引数を指定する．
kxr_controller.launchなどのファイルはセンサや台車が異なるJedyでも
共通した部分を記述しており， jedy_bringup.launch.pyでは`IncludeLaunchDescription`を使ってそれらをincludeし
引数を渡すことで各ロボットに対応させている．

