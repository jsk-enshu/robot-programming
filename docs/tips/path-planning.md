# 経路計画

地図の上で「どこを通って目標まで行くか」を決めるのが経路計画である．
ROS 2 Jazzyでは[Nav2](https://docs.nav2.org/)がこれを行う．
Gazeboの演習室で地図を作り，Nav2で目標まで走らせる手順は{doc}`slam`にある．
このページでは，そのときNav2の中で何が起きているかを説明する．

## Nav2の中の流れ

RVizの「Nav2 Goal」やPython（`nav2_goal`）で目標を送ると，次の順に処理される．

1. **自己位置推定（AMCL）**：LiDARの点を地図に合わせ，ロボットが地図のどこにいるかを推定する．RVizの緑の矢印の群れ（`/particle_cloud`）が推定の候補で，ロボットが動くと実際の位置に集まっていく．
2. **コストマップ**：地図とLiDARから，「ぶつかる」「近い」「通れる」を数値にした格子（0.05 m角）を作る．
3. **経路計画（プランナ）**：全体のコストマップの上で，今の位置から目標までの経路（`/plan`）を探す．
4. **経路追従（コントローラ）**：経路に沿うように，台車の速度（`/cmd_vel`）を1秒に何回も計算して送る．
5. **復帰動作**：進めなくなったら，その場で回る，少し下がる，待つ，などを試す．

どの順で何を呼ぶかは，ビヘイビアツリー（Behavior Tree）というXMLで書かれている（`bt_navigator`）．

## コストマップ

コストマップは2つある．RVizの左の「Displays」で表示を切り替えられる．

| | RVizの表示 | 大きさ | 使うもの |
| --- | --- | --- | --- |
| 全体（global） | Global Planner → Global Costmap | 地図全体 | 地図（StaticLayer），LiDAR（ObstacleLayer），膨張（InflationLayer） |
| 局所（local） | Controller → Local Costmap | ロボットの周り3 m×3 m | LiDAR（ObstacleLayer），膨張（InflationLayer） |

膨張（inflation）は，障害物の周りのコストを距離に応じて下げながら広げるものである．
Jedyの設定（`jedy_bringup/config/nav2_params.yaml`）では`inflation_radius: 0.35`（m）なので，障害物から0.35 m以内は「なるべく通らない」場所になる．
ロボットの形（footprint）は腕をたたんだ台車を囲む長方形（前0.20 m，後ろ0.085 m，左右0.135 m）で，これが障害物に重なる位置は通れない．

## プランナとコントローラ

| 役割 | Jedyの設定 | 中身 |
| --- | --- | --- |
| プランナ | `nav2_navfn_planner::NavfnPlanner`（`use_astar: false`） | コストマップの上でダイクストラ法により最小コストの経路を探す．`use_astar: true`でA\*になる |
| スムーザ | `nav2_smoother::SimpleSmoother` | プランナの経路の角を滑らかにする |
| コントローラ | `nav2_mppi_controller::MPPIController`（`motion_model: "Omni"`） | 速度の候補をたくさん乱数で作って少し先まで動かしてみて，経路に近く障害物から遠いものを選ぶ（MPPI）．メカナムの台車なので横移動（`vy`）も使う |
| ゴールの判定 | `nav2_controller::SimpleGoalChecker` | 位置0.15 m，向き0.2 radまで近づいたら到着 |

速さの上限は前後・左右0.156 m/s，旋回1.66 rad/sで，実機の台車に合わせてある（{doc}`slam` の「Nav2の起動」）．

## 設定を変えて試す

設定ファイルを自分の場所にコピーして書き換え，`params_file:=`で渡す．

```{code-block} console
$ cp $(ros2 pkg prefix jedy_bringup)/share/jedy_bringup/config/nav2_params.yaml ~/enshu/my_nav2_params.yaml
$ ros2 launch jedy_bringup jedy_nav2.launch.py params_file:=$HOME/enshu/my_nav2_params.yaml rviz:=true
```

たとえば次を変えて，RVizに出るコストマップや経路（Global Planner → Path），走り方がどう変わるかを見てみよう．

- `inflation_radius`（global_costmapとlocal_costmapの両方）：障害物の周りの「なるべく通らない」範囲の広さ．RVizのコストマップの色の付いた範囲が変わる．
- `use_astar`：`true`にするとダイクストラ法の代わりにA\*で探す．経路の形や探す時間を比べる．
- `xy_goal_tolerance`，`yaw_goal_tolerance`（general_goal_checker）：どこまで近づいたら到着とするか．

`inflation_radius: 0.6`と`use_astar: true`を両方入れても，`nav2_goal 4.4,-3.6,90`は変えないときと同じく約53秒（ロボットの時計）で着いた．

:::{warning}
`vy_max`（横移動の上限）を0にすると，この設定のままでは`Failed to make progress`を繰り返して目標に着かなかった（`motion_model`を`"DiffDrive"`にしても同じ）．
横移動を止めるには，コントローラの設定をもっと変える必要がある．
:::

各パラメータの意味は[Nav2の設定ガイド](https://docs.nav2.org/configuration/index.html)にある．

:::{note}
このページは以前，ROS 1（TurtleBotの`move_base`と`amcl`）の手順だった．ROS 2 Jazzyでは`move_base`はNav2の`planner_server`と`controller_server`などに分かれ，RVizの「2D Nav Goal」は「Nav2 Goal」になった．
:::
