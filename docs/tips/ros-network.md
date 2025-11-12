# ROS/ROS 2ネットワーク設定

このページでは，ROS 1とROS 2におけるネットワーク設定の方法について説明する．

## ROS 2ネットワーク設定

### ⚠️ **重要: ネットワーク設定**

**<span style="color:red">演習中や同一ネットワークで複数人がROS 2を起動すると，通信が相互に干渉してしまう．</span>**

**<span style="color:red">必ず以下の環境変数を設定すること:</span>**

```{code-block} console
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

**bashrcに追加して永続化する場合:**

```{code-block} console
$ echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc
$ source ~/.bashrc
```

この設定により，ROS 2の通信がローカルホストのみに制限され，他の受講者との通信干渉を防ぐことができる．

> **注意:** ROS 2 Jazzy以前では`ROS_LOCALHOST_ONLY=1`が使用されていたが，Jazzyで非推奨となり削除された．代わりに`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`を使用すること．詳細は[Improved Dynamic Discovery](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)を参照．

### 設定の確認

環境変数が正しく設定されているか確認する:

```{code-block} console
$ echo $ROS_AUTOMATIC_DISCOVERY_RANGE
```

`LOCALHOST`と表示されれば設定完了である．

### 複数PCで通信したい場合（ROS 2）

複数のPC間でROS 2通信を行いたい場合は，以下のいずれかの設定を使用する:

**サブネット内での通信:**

```{code-block} console
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

**ローカルネットワーク全体での通信:**

```{code-block} console
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
```

ただし，演習環境では基本的に`LOCALHOST`を使用すること．

### ROS_DOMAIN_IDを使った通信の分離

ROS 2では`ROS_DOMAIN_ID`環境変数を使用して，異なるドメイン間で通信を分離することができる．同じ`ROS_DOMAIN_ID`を持つノード同士のみが通信可能であり，異なるドメインIDのノードは互いに通信できない．

#### ROS_DOMAIN_IDの特徴

- **デフォルト値**：0
- **設定可能範囲**：0〜101（または0〜232，DDS実装による）
- **用途**：複数のロボットシステムやユーザーが同一ネットワーク上で独立して動作する場合に使用

#### 実機のROS_DOMAIN_IDの確認

本演習で使用する`Jedy`ロボットには，あらかじめ固有の`ROS_DOMAIN_ID`が設定されている．このIDは**ロボット背面の`Atom S3`ディスプレイに表示**されている．

実機と通信するには，自分のPCでも同じ`ROS_DOMAIN_ID`を設定する必要がある．

#### ROS_DOMAIN_IDの設定方法

**一時的な設定（現在のターミナルのみ）:**

```{code-block} console
$ export ROS_DOMAIN_ID=<Atom S3に表示されているID>
```

例えば，`Atom S3`に`42`と表示されている場合：

```{code-block} console
$ export ROS_DOMAIN_ID=42
```

**bashrcに追加して永続化する場合:**

```{code-block} console
$ echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
$ source ~/.bashrc
```

#### 設定の確認

環境変数が正しく設定されているか確認する：

```{code-block} console
$ echo $ROS_DOMAIN_ID
```

`Atom S3`に表示されているIDと同じ値が表示されれば設定完了である．

#### 注意事項

- **演習開始時の確認**：演習を開始する際は，必ず`Atom S3`でIDを確認し，自分のPCに設定すること
- **ロボット切り替え時**：別の`Jedy`ロボットを使用する場合は，新しいロボットの`ROS_DOMAIN_ID`を再設定する必要がある
- **ROS_AUTOMATIC_DISCOVERY_RANGEとの併用**：`ROS_DOMAIN_ID`を設定しても，`ROS_AUTOMATIC_DISCOVERY_RANGE`の設定は維持すること．演習環境では`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`と`ROS_DOMAIN_ID`の両方を設定する

#### トラブルシューティング

**ROS 2のトピックが見えない場合**の一般的な確認項目を以下に示す．

##### 基本的な確認項目

1. **`ROS_DOMAIN_ID`が実機と一致しているか確認：**

   ```{code-block} console
   $ echo $ROS_DOMAIN_ID
   ```

2. **ネットワーク接続を確認：**

   ```{code-block} console
   $ ping <ロボットPCのIPアドレス>
   ```

3. **`ROS_AUTOMATIC_DISCOVERY_RANGE`を確認：**

   ```{code-block} console
   $ echo $ROS_AUTOMATIC_DISCOVERY_RANGE
   ```

4. **ROS 2デーモンの再起動：**

   ROS 2のデーモンが古い情報を保持している可能性がある．一度デーモンを停止させてみる．

   ```{code-block} console
   $ ros2 daemon stop
   $ ros2 daemon start
   ```

   その後，`ros2 topic list`などを実行して確認する．

##### 詳細なトラブルシューティング

**ROS_DOMAIN_IDとサブネットが同じでもトピックが見えない場合**，以下の原因が考えられる．ROS 2はDDS（Data Distribution Service）というミドルウェアで通信しており，ネットワーク設定以外にもDDSレベルでの設定が影響する．

###### 1. DDSミドルウェアの実装が異なる

ROS 2は複数のDDS実装（Fast DDS，Cyclone DDSなど）を利用できる．通信するノード同士で，使用しているDDS実装が異なる（または互換性がない）と通信できない．

**確認方法：**

両方のマシン（またはターミナル）で，環境変数`RMW_IMPLEMENTATION`を確認する．

```{code-block} console
$ echo $RMW_IMPLEMENTATION
```

**対策：**

両方で同じ実装（例：`rmw_fastrtps_cpp`や`rmw_cyclonedds_cpp`）に設定するか，環境変数を未設定（デフォルトのFast DDSが使われる）にする．

###### 2. ファイアウォールによるブロック

DDSは通信の検出（Discovery）やデータ交換にUDPポート（場合によってはTCPも）を使用する．OSのファイアウォールがこれらのパケットをブロックしている可能性がある．

**確認方法：**

一時的に両方のマシンのファイアウォールを無効にして，トピックが見えるようになるか試す．

```{code-block} console
$ sudo ufw disable
```

**対策：**

もしファイアウォールが原因なら，DDSが使用するポート（特定のポート範囲またはプロセス）を許可するルールを追加する．

###### 3. 使用するネットワークインターフェースの不一致

マシンに複数のネットワークインターフェース（Wi-Fi，有線LAN，VPN，Dockerの仮想ブリッジなど）があると，DDSが意図しないインターフェース（例：サブネットが異なるインターフェース）を使おうとすることがある．

**確認方法：**

`ifconfig`や`ip a`コマンドで，意図したサブネットのIPアドレスが割り当てられているインターフェース名を確認する．

```{code-block} console
$ ip a
```

**対策：**

DDSの設定ファイル（例：Fast DDSなら`DEFAULT_FASTRTPS_PROFILES.xml`，Cyclone DDSなら`cyclonedds.xml`）で，使用するネットワークインターフェースを明示的に指定する．

###### 4. DDSのXML設定ファイル

特定のDDS設定（`FASTRTPS_DEFAULT_PROFILES_FILE`などで読み込むXML）が，通信の検出範囲を制限している（例：localhostのみに設定されている）場合がある．

**確認方法：**

環境変数`FASTRTPS_DEFAULT_PROFILES_FILE`や`CYCLONEDDS_URI`が設定されているか確認する．

```{code-block} console
$ echo $FASTRTPS_DEFAULT_PROFILES_FILE
$ echo $CYCLONEDDS_URI
```

**対策：**

設定ファイルの内容を確認し，必要に応じて修正または環境変数を解除する．

##### 切り分けのためのステップ

まずは以下の基本的な点を確認する：

1. **pingが通るか？**

   お互いのIPアドレスに対して`ping`を実行し，基本的なネットワーク疎通があることを確認する．

   ```{code-block} console
   $ ping <相手のIPアドレス>
   ```

2. **ROS_DOMAIN_IDの値を再確認**

   `echo $ROS_DOMAIN_ID`を実行し，両方で全く同じ値（数字）が設定されているか，あるいは両方とも未設定（デフォルトの0が使われる）になっているかを確認する．

   ```{code-block} console
   $ echo $ROS_DOMAIN_ID
   ```

3. **ROS 2デーモンの再起動**

   上記「基本的な確認項目」の4を参照．

## ROS 1ネットワーク設定

ROS 1では，複数のPC間で通信を行うために以下の環境変数を設定する必要がある．

### 基本的な環境変数

#### ROS_MASTER_URI

ROSマスターが動作しているPCのアドレスとポート番号を指定する．

```{code-block} console
$ export ROS_MASTER_URI=http://192.168.1.100:11311
```

- デフォルトは`http://localhost:11311`
- 同一PC内でのみ通信する場合はデフォルトのまま

#### ROS_IP

自分のPCのIPアドレスを指定する．

```{code-block} console
$ export ROS_IP=192.168.1.101
```

#### ROS_HOSTNAME

自分のPCのホスト名を指定する（`ROS_IP`の代わりに使用可能）．

```{code-block} console
$ export ROS_HOSTNAME=my-robot
```

> **注意:** `ROS_IP`と`ROS_HOSTNAME`は同時に設定しないこと．どちらか一方のみを使用する．

### 設定例

**マスターPC（192.168.1.100）の場合:**

```{code-block} console
$ export ROS_MASTER_URI=http://192.168.1.100:11311
$ export ROS_IP=192.168.1.100
```

**クライアントPC（192.168.1.101）の場合:**

```{code-block} console
$ export ROS_MASTER_URI=http://192.168.1.100:11311
$ export ROS_IP=192.168.1.101
```

### bashrcへの追加

永続化する場合は，`~/.bashrc`に追加する:

```{code-block} console
$ echo 'export ROS_MASTER_URI=http://192.168.1.100:11311' >> ~/.bashrc
$ echo 'export ROS_IP=192.168.1.101' >> ~/.bashrc
$ source ~/.bashrc
```

### IPアドレスの確認方法

自分のPCのIPアドレスを確認するには以下のコマンドを使用する:

```{code-block} console
$ hostname -I
```

または

```{code-block} console
$ ip addr show
```

### トラブルシューティング

#### ノードが見えない場合

1. **ネットワーク接続の確認:**

```{code-block} console
$ ping 192.168.1.100
```

2. **環境変数の確認:**

```{code-block} console
$ echo $ROS_MASTER_URI
$ echo $ROS_IP
```

3. **ファイアウォールの確認:**

ファイアウォールがROSの通信をブロックしている可能性がある．一時的に無効化して確認する:

```{code-block} console
$ sudo ufw disable
```

4. **マスターへの接続確認:**

```{code-block} console
$ rosnode list
```

マスターに接続できていれば，ノード一覧が表示される．

5. **`rossetip`コマンドが見つからない場合:**

もし`rossetip`コマンドが見つからない場合は，以下のコマンドで`ros-one-jsk-tools`をインストールし作業環境を再設定する．

```{code-block} console
$ sudo apt install ros-one-jsk-tools
$ source ~/ros_ws/devel/setup.bash
```

これにより`rossetip`コマンドが使用可能となるため，再度`rossetip`と`rossetmaster`を実行した上で`rostopic list`で通信が行われるかを確認する．

`rostopic list`をしても期待するトピックが見つからない場合は，ネットワークが`Jedy`と同じネットワークに接続されているかを確認し，IPアドレスの打ち間違いなどがないかを確認する．

> **注意:** `rossetip`の詳細については，[複数PCの接続設定](multi-pc.md)を参照のこと．

### 参考資料

詳細は公式ドキュメントを参照:
- [ROS Network Setup](https://wiki.ros.org/ROS/NetworkSetup)
- [ROS 2 Improved Dynamic Discovery](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)
