# ROS 2ネットワーク設定

このページでは，ROS 2（Jazzy）のネットワーク設定の方法について説明する．

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

> **注意:** ROS 2 Jazzy以前では`ROS_LOCALHOST_ONLY=1`が使用されていたが，Jazzyで非推奨になった（まだ効くが，将来のバージョンで削除される）．代わりに`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`を使用すること．詳細は[Improved Dynamic Discovery](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)を参照．

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

#### 実機と通信するとき

2026 年版の演習では，ロボット(実機)は自分の PC に USB でつなぎ，ROS 2 のノードはすべて自分の PC の中で動く．
そのため上の `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` だけでよく，`ROS_DOMAIN_ID` を設定する必要はない
(通信が PC の外に出ないので，他の人のロボットやシミュレータと混ざらない)．

ロボットと PC が**ネットワーク越しに** ROS 2 で通信する構成(ロボットに別の PC が載っている場合など)では，
`LOCALHOST` では相手が見えない．その場合は `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` にし，ロボット側と PC 側で
同じ `ROS_DOMAIN_ID` を設定する:

```{code-block} console
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
$ export ROS_DOMAIN_ID=42      # ロボット側と同じ値
$ echo $ROS_DOMAIN_ID          # 確認
```

同じネットワークで複数のロボットを使うときは，ロボットごとに違う `ROS_DOMAIN_ID` にする．

2025 年度版の資料 ({doc}`/robot-programming-1-2025`) では，ロボット背面の `Atom S3` に表示された `ROS_DOMAIN_ID` を PC 側にも設定していた．

#### トラブルシューティング

**ROS 2のトピックが見えない場合**の一般的な確認項目を以下に示す．

##### 基本的な確認項目

1. **すべてのターミナルで設定が同じか確認：**

   ノードを起動したターミナルと`ros2 topic list`を実行したターミナルで，次の2つが同じ値（または両方とも未設定）になっているかを確かめる．
   `~/.bashrc`に書く前に開いたターミナルには設定が入っていない．

   ```{code-block} console
   $ echo $ROS_AUTOMATIC_DISCOVERY_RANGE
   $ echo $ROS_DOMAIN_ID
   ```

2. **（ネットワーク越しに通信する構成のとき）相手のPCとのネットワーク接続を確認：**

   ```{code-block} console
   $ ping <相手のPCのIPアドレス>
   ```

3. **（ネットワーク越しに通信する構成のとき）`ROS_AUTOMATIC_DISCOVERY_RANGE`が`LOCALHOST`になっていないか確認：**

   `LOCALHOST`のままでは相手のPCのノードは見えない．上の「実機と通信するとき」のとおり`SUBNET`にする．

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

## 参考資料

- [ROS 2 Improved Dynamic Discovery](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)
- [ROS_DOMAIN_ID](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)
