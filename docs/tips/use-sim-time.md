# use_sim_timeパラメータとは

## 概要

`use_sim_time`はROSのパラメータの一つでありシミュレーション環境を使用する際の時刻管理を制御する重要な設定である．このパラメータを理解することでシミュレーション環境と実機環境の違いを適切に扱えるようになる．

ROS 1とROS 2で設定方法が異なるため，それぞれについて説明する．

## use_sim_timeの役割

### 実機環境での時刻管理

通常実機でロボットを動かす場合ROSノードは**実際の時計（システムクロック）** を使用する．

- `rospy.Time.now()`や`ros::Time::now()`は現在のシステム時刻を返す
- センサデータのタイムスタンプは実際の時刻が記録される
- タイムアウト処理や周期的な処理は実時間で動作する

### シミュレーション環境での時刻管理

一方Gazeboなどのシミュレータを使用する場合**シミュレーション時刻**を使用する必要がある．

シミュレーション時刻とは：
- シミュレータ内部で管理される仮想的な時刻
- シミュレーションの実行速度に応じて進む
- 一時停止すれば時刻も停止する
- 高速実行すれば時刻も速く進む

**use_sim_timeパラメータの機能：**

`/use_sim_time`を`true`に設定するとROSノードは：
- システムクロックではなく`/clock` topicから時刻を取得する
- Gazeboなどのシミュレータが`/clock` topicにシミュレーション時刻をpublishする
- すべてのROSノードがシミュレーション時刻を使用するようになる

## 設定方法

### ROS 1での設定

#### Gazebo使用時の設定

Gazeboでシミュレーションを実行する場合は`use_sim_time`を`true`に設定する．

**方法1: rosparamコマンドで設定**

```{code-block} console
$ rosparam set /use_sim_time true
```

**方法2: launchファイルで設定**

```xml
<launch>
  <param name="/use_sim_time" value="true" />
  <!-- Gazeboや他のノードの起動 -->
</launch>
```

多くのシミュレーション用launchファイルでは自動的にこのパラメータが設定される．

#### 実機使用時の設定

**重要：実機を使用する場合は`use_sim_time`を`true`に設定してはいけない．**

実機で`use_sim_time=true`にするとロボットが正常に動作しなくなる．

**理由：**

ロボットの動作指令（例：`follow_joint_trajectory`）には**動作開始時刻**のタイムスタンプが含まれている．この時刻はコマンドを送信したノードが使用している時刻系に基づいて設定される．

- **use_sim_time=true**の場合：ノードは`/clock`トピックからシミュレーション時刻を取得する
- **実機のコントローラ**：`/clock`トピックを購読せずシステム時刻を使用している

この時刻の不一致により以下の問題が発生する：

1. 動作指令のタイムスタンプがシミュレーション時刻（例：123秒）
2. 実機コントローラはシステム時刻（例：1234567890秒）で判断
3. 「過去の指令」または「未来すぎる指令」として拒否される
4. ロボットが動かない

**正しい設定：**

実機では`use_sim_time`を設定しない（デフォルトで`false`）：

```{code-block} console
# 設定を確認
$ rosparam get /use_sim_time
# ERROR: Parameter [/use_sim_time] is not set  ← これが正常

# もし誤ってtrueになっていたら削除またはfalseに設定
$ rosparam delete /use_sim_time
# または
$ rosparam set /use_sim_time false
```

### ROS 2での設定

ROS 2では`use_sim_time`パラメータの扱いが異なる．ROS 1のようなグローバルパラメータではなく**各ノードごとにパラメータを設定する**必要がある．

#### Gazebo使用時の設定

**方法1: ros2 runコマンドで設定**

```{code-block} console
$ ros2 run <package_name> <node_name> --ros-args -p use_sim_time:=true
```

**方法2: launchファイルで設定**

ROS 2のlaunchファイル（Python形式）で設定する場合：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='your_node',
            name='your_node',
            parameters=[{'use_sim_time': True}]
        ),
    ])
```

**方法3: すべてのノードに一括設定**

launchファイル内のすべてのノードに対して`use_sim_time`を設定する場合：

```python
from launch import LaunchDescription
from launch.actions import SetParameter
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        Node(
            package='your_package',
            executable='your_node1',
        ),
        Node(
            package='your_package',
            executable='your_node2',
        ),
    ])
```

#### 実機使用時の設定

**重要：実機を使用する場合は`use_sim_time=true`を設定してはいけない．**

実機で`use_sim_time=true`にするとロボットが正常に動作しなくなる．

**理由：**

ROS 2でもROS 1と同様にロボットの動作指令（例：`follow_joint_trajectory`）には**動作開始時刻**のタイムスタンプが含まれており，この時刻はコマンドを送信したノードが使用している時刻系に基づいて設定される．

- **use_sim_time=true**の場合：ノードは`/clock`トピックからシミュレーション時刻を取得する
- **実機のコントローラ**：通常は`/clock`トピックを購読せずシステム時刻を使用している

この時刻の不一致により：

1. 動作指令のタイムスタンプがシミュレーション時刻（例：123秒）
2. 実機コントローラはシステム時刻（例：1234567890秒）で判断
3. 「過去の指令」または「未来すぎる指令」として拒否される
4. ロボットが動かない

**正しい設定：**

実機では`use_sim_time`を設定しない（デフォルトで`false`）：

```{code-block} console
# ノードを起動（use_sim_timeパラメータを指定しない）
$ ros2 run <package_name> <node_name>

# もし誤ってtrueになっているノードがあれば確認
$ ros2 param get /node_name use_sim_time
# 出力: Boolean value is: False  ← これが正常
```

#### ROS 2での確認方法

特定のノードの`use_sim_time`設定を確認するには：

```{code-block} console
$ ros2 param get /node_name use_sim_time
```

例：
```{code-block} console
$ ros2 param get /robot_state_publisher use_sim_time
Boolean value is: True
```

## use_sim_timeの確認方法（ROS 1）

ROS 1で現在の設定を確認するには以下のコマンドを使用する：

```{code-block} console
$ rosparam get /use_sim_time
```

出力：
- `true`: シミュレーション時刻を使用
- `false`または`ERROR: Parameter [/use_sim_time] is not set`: システム時刻を使用

## なぜuse_sim_timeが必要なのか

### 1. 時刻の一貫性

シミュレーション環境では複数のROSノードが協調動作する際すべてのノードが同じ時刻を参照する必要がある．

**問題の例（use_sim_timeがfalseの場合）：**
- Gazeboがシミュレーション時刻でセンサデータのタイムスタンプを設定
- ROSノードがシステム時刻でデータを処理
- 時刻の不整合により以下の問題が発生：
  - TF（座標変換）のタイムスタンプエラー
  - センサフュージョンの失敗
  - タイムアウト処理の誤動作

**解決（use_sim_timeがtrueの場合）：**
- すべてのノードが`/clock` topicからシミュレーション時刻を取得
- Gazeboとすべてのノードが同じ時刻を共有
- 正しく動作する

### 2. シミュレーションの再現性

シミュレーション時刻を使用することでシミュレーションの結果を再現できる．

- システム時刻を使うと実行するたびに異なる時刻が記録される
- シミュレーション時刻を使うと同じ条件で実行すれば同じ時刻進行になる
- デバッグやテストが容易になる

### 3. シミュレーション速度の制御

シミュレーション時刻により実時間より速くまたは遅くシミュレーションを実行できる．

- 高速実行：長時間のシミュレーションを短時間で完了
- スロー実行：詳細な観察やデバッグ
- 一時停止：状態の確認や分析

## 実際の使用例

### ROS 1: EusLispでシミュレーションを使用する場合

EusLispはROS 1のみをサポートしているためGazeboシミュレーションを使用する際には必ず`use_sim_time`を設定する必要がある．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ rosparam set /use_sim_time true
$ roseus your-script.l
```

**設定しないとどうなるか：**
- TFのエラーが発生する
- センサデータのタイムスタンプが不整合になる
- ロボットモデルの表示が正しく行われない

### ROS 1: 実機とシミュレーションの切り替え

同じプログラムで実機とシミュレーションを切り替える場合**use_sim_timeの設定を必ず切り替える**必要がある．

**シミュレーション：**
```{code-block} console
$ rosparam set /use_sim_time true
$ roslaunch your_package gazebo.launch
$ roseus your-script.l
```

**実機：**
```{code-block} console
# use_sim_timeを削除またはfalseに設定（重要！）
$ rosparam delete /use_sim_time
# または
$ rosparam set /use_sim_time false

$ roslaunch your_package real_robot.launch
$ roseus your-script.l
```

<span style="color:red">**注意：シミュレーションから実機に切り替える際にuse_sim_timeをtrueのままにするとロボットが動作しない．** </span>`follow_joint_trajectory`などの動作指令の時刻が不一致になるためである．

### ROS 2: Gazeboシミュレーションでの使用

ROS 2でGazeboシミュレーションを使用する場合launchファイルで`use_sim_time`を設定する：

```python
from launch import LaunchDescription
from launch.actions import SetParameter, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Gazeboのlaunchファイルをインクルード
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    return LaunchDescription([
        # すべてのノードに対してuse_sim_timeをtrueに設定
        SetParameter(name='use_sim_time', value=True),

        gazebo_launch,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
        ),

        Node(
            package='your_package',
            executable='your_controller',
            name='your_controller',
        ),
    ])
```

**コマンドラインから個別に起動する場合：**

```{code-block} console
# Gazeboを起動
$ ros2 launch gazebo_ros gazebo.launch.py

# ノードをuse_sim_time=trueで起動
$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true
$ ros2 run your_package your_controller --ros-args -p use_sim_time:=true
```

## `/clock` topicについて

`use_sim_time`が`true`の場合ROSノードは`/clock` topicから時刻を取得する．このtopicはROS 1とROS 2の両方で同じ役割を持つ．

### ROS 1での/clock topic

**clockトピックの確認：**

```{code-block} console
$ rostopic echo /clock
```

出力例：
```
clock:
  secs: 123
  nsecs: 456000000
---
clock:
  secs: 124
  nsecs: 456000000
---
```

**clockトピックの型：**
```{code-block} console
$ rostopic type /clock
rosgraph_msgs/Clock
```

### ROS 2での/clock topic

**clockトピックの確認：**

```{code-block} console
$ ros2 topic echo /clock
```

出力例：
```
clock:
  sec: 123
  nanosec: 456000000
---
clock:
  sec: 124
  nanosec: 456000000
---
```

**clockトピックの型：**
```{code-block} console
$ ros2 topic info /clock
Type: rosgraph_msgs/msg/Clock
Publisher count: 1
Subscription count: 5
```

Gazeboは自動的に`/clock` topicをpublishするが他のシミュレータを使用する場合は手動で設定が必要な場合がある．

## トラブルシューティング

### TFのタイムスタンプエラー

**エラーメッセージ例：**
```
Lookup would require extrapolation into the past
```

**原因：**
- `use_sim_time`が正しく設定されていない
- Gazeboが`/clock`をpublishしていない

**解決方法：**
```{code-block} console
$ rosparam set /use_sim_time true
# Gazeboを再起動
```

### 時刻が進まない

**症状：**
- `rospy.Time.now()`が更新されない
- タイムアウト処理が動作しない

**原因：**
- `use_sim_time`が`true`だが`/clock`がpublishされていない

**解決方法：**
```{code-block} console
# clockがpublishされているか確認
$ rostopic hz /clock

# publishされていなければGazeboを起動
$ roslaunch gazebo_ros empty_world.launch

# またはuse_sim_timeをfalseに
$ rosparam set /use_sim_time false
```

### 実機でロボットが動作しない

**症状：**
- 実機を使っているのにロボットが動かない
- `follow_joint_trajectory`の動作指令を送っても反応しない
- TFエラーが出る（`Lookup would require extrapolation into the past`など）
- センサデータが古いと警告が出る

**原因：**
- `use_sim_time`が`true`のままになっている
- シミュレーションから実機に切り替えた際に設定を変更し忘れている

**なぜ動かないのか：**

動作指令（`follow_joint_trajectory`）の時刻とコントローラの時刻が一致しないため：

1. **コマンド送信ノード**（`use_sim_time=true`）：`/clock`トピックから時刻を取得（例：123秒）
2. **動作指令のタイムスタンプ**：123秒で開始するように設定される
3. **実機コントローラ**：システム時刻を使用（例：1234567890秒）
4. **判定**：「123秒はもう過ぎた（過去の指令）」として拒否される
5. **結果**：ロボットが動かない

**解決方法：**

ROS 1の場合：
```{code-block} console
# use_sim_timeを確認
$ rosparam get /use_sim_time
true  # ← trueになっている場合は削除またはfalseに設定

# 削除する
$ rosparam delete /use_sim_time

# または falseに設定
$ rosparam set /use_sim_time false

# すべてのノードを再起動
```

ROS 2の場合：
```{code-block} console
# 各ノードのuse_sim_timeを確認
$ ros2 param get /controller_manager use_sim_time
Boolean value is: True  # ← trueになっている場合は修正が必要

# launchファイルからSetParameter(name='use_sim_time', value=True)を削除
# または各ノードのparametersからuse_sim_timeを削除して再起動
```

## まとめ

### ROS 1での基本ルール

1. **Gazeboなどのシミュレータを使用する場合：**
   - `rosparam set /use_sim_time true`を実行
   - シミュレータ起動後にROSノードを起動

2. **実機を使用する場合：**
   - **絶対に`use_sim_time=true`を設定してはいけない**
   - デフォルトのまま（未設定）またはfalseに設定
   - システム時刻を使用
   - **理由**：follow_joint_trajectoryなどの動作指令の時刻が不一致になりロボットが動かなくなる

3. **EusLispでシミュレーションを使用する場合：**
   - 必ず`rosparam set /use_sim_time true`を実行してからEusLispスクリプトを起動
   - 設定を忘れるとTFエラーなどが発生する

4. **切り替え時の注意（重要）：**
   - シミュレーションから実機に切り替える際は**必ず`use_sim_time`を削除またはfalseに戻す**
   - 設定を変更し忘れるとロボットが動作しない
   - すべてのノードを再起動して設定を反映させる

### ROS 2での基本ルール

1. **Gazeboなどのシミュレータを使用する場合：**
   - launchファイルで`SetParameter(name='use_sim_time', value=True)`を設定
   - またはコマンドラインで`--ros-args -p use_sim_time:=true`を指定
   - **各ノードごとに設定が必要**

2. **実機を使用する場合：**
   - **絶対に`use_sim_time=true`を設定してはいけない**
   - デフォルトのまま（未設定）にする
   - システム時刻を使用
   - **理由**：follow_joint_trajectoryなどの動作指令の時刻が不一致になりロボットが動かなくなる

3. **切り替え時の注意（重要）：**
   - シミュレーションから実機に切り替える際は**必ずlaunchファイルから`use_sim_time`設定を削除**
   - 設定を変更し忘れるとロボットが動作しない
   - すべてのノードを再起動して設定を反映させる

### ROS 1とROS 2の違い

| 項目 | ROS 1 | ROS 2 |
|:-----|:------|:------|
| パラメータの種類 | グローバルパラメータ | ノード単位のパラメータ |
| 設定方法 | `rosparam set /use_sim_time true` | `--ros-args -p use_sim_time:=true` |
| 設定範囲 | すべてのノードに自動適用 | 各ノードごとに設定が必要 |
| 確認方法 | `rosparam get /use_sim_time` | `ros2 param get /node_name use_sim_time` |

`use_sim_time`を正しく設定することでシミュレーション環境と実機環境の両方でスムーズにロボットプログラミングができるようになる．

## 参考資料

### ROS 1

- [ROS Wiki - Clock](http://wiki.ros.org/Clock)
- [ROS Wiki - Gazebo Simulation Time](http://wiki.ros.org/gazebo/Tutorials/Using%20roslaunch%20to%20start%20Gazebo%2C%20world%20files%20and%20URDF%20models#Using_ROS_simulation_time_published_on_.2BAC8-clock)
- [ROS Answers - use_sim_time](https://answers.ros.org/questions/tags:use_sim_time)

### ROS 2

- [ROS 2 Design - Clock and Time](https://design.ros2.org/articles/clock_and_time.html)
- [ROS 2 Documentation - Parameters](https://docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html)
- [Gazebo ROS 2 Integration](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)
