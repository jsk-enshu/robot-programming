# 近藤サーボ（KRS）の設定

## 概要

近藤科学の**KRS（KONDO ROBOT SERVO）シリーズ**は，ICS（Interactive Communication System）という通信規格で，シリアル通信で角度を指令したり設定を変えたりできるサーボモータである．

2026 年度の演習では，{doc}`../robot-programming-3-2026` の発展課題の四脚ロボット JEFY で KRS-3302 ICS を 8 個使う．
サーボは近藤科学のコントロールボード **RCB-4mini** にまとめてつなぎ，Arduino UNO R4 WiFi は RCB-4mini に指令を送る．
サーボに ID を振るなどの設定は，PC に **Dual USB アダプター HS** でサーボを 1 個ずつつなぎ，`rcb4` パッケージの `ics-manager` で行う．

組み立て・配線・書き込み・校正の手順は [近藤科学の KRS サーボで組む（RCB-4mini 版）](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs.html) にまとまっている．このページは，その中のサーボの設定と，KRS サーボそのものについての補足である．

### ICS（Interactive Communication System）について

ICS は，サーボとコントロールボードの間の双方向の通信規格である．
コントロールボードからサーボを動かすときの通信にも，PC からサーボの設定を変えるときの通信にも使う．
1 本の線に複数のサーボをつなぎ（マルチドロップ接続），ID でどのサーボかを区別する．

**ICS3.5/3.6 の主な特徴：**

- 通信速度は 115200 bps，625000 bps，1.25 Mbps から選べる
- スピードやストレッチのほか，温度制限や電流制限などの特性を，動作中に変更できる
- マルチドロップ接続で最大 32 個つなげる（コントロールボードによって最大数が変わる）
- ICS3.6 は ICS3.5 の上位互換で，現在値を取得するコマンドが追加されている

**ICS3.5/3.6 の主な機能一覧：**

| 機能名 | 説明 |
|:-------|:-----|
| スピード | サーボの最大出力（デューティー比）を設定．軸の回転速度が変化する |
| ストレッチ | 軸の保持特性を設定 |
| パンチ | サーボの初動を調整 |
| デッドバンド | サーボのニュートラル帯域（不感帯）を調整 |
| プロテクション | サーボがロックしたときに脱力するまでの時間を設定 |
| リミッタ | 正転・逆転の移動範囲を制限 |
| 温度制限 | 設定した温度を超えるとサーボが脱力状態になる |
| 電流制限 | 設定した電流を超えるとサーボが脱力状態になる |
| リバース | サーボが指定した方向とは逆方向に移動する |
| スレーブ | サーボが返事を返さなくなる |
| 回転モード | サーボの軸が指定の方向に回転し続ける |
| レスポンス | 出力軸の動作開始時の立ち上がり特性を設定 |
| ダンピング | 出力軸の動作停止時のブレーキ特性を設定 |
| ユーザーオフセット | 出力軸の初期位置をユーザーが任意に設定可能 |
| シリアル専用 | シリアル信号での制御専用のモジュールとして設定 |
| 現在値（ICS3.6のみ） | サーボの現在値を取得 |

## 必要な機器

| 機器名 | 説明 |
|:-------|:-----|
| KRS サーボ | ICS 対応のサーボ．JEFY は KRS-3302 ICS を 8 個使う |
| Dual USB アダプター HS（近藤科学） | PC とサーボを ICS でつなぐ．ID を振るときに使う |
| RCB-4mini（近藤科学） | サーボをまとめて動かすコントロールボード |
| 電源 | ROBO パワーセル E タイプ（7.2 V）と充電器，または近藤科学の AC アダプター |

## rcb4 のインストール

`ics-manager` などのコマンドは，Python のパッケージ [rcb4](https://github.com/iory/rcb4) に入っている．
`uv tool install` で入れる（`uv` が無ければ {doc}`../environment-setup` の「初回環境構築」にある `curl -LsSf https://astral.sh/uv/install.sh | sh` で入れておく）．

```{code-block} console
$ uv tool install rcb4
...
Installed 5 executables: armh7-tools, ics-manager, rcb4-rom, rcb4-shell, rcb4-write-firmware
```

コマンドは `~/.local/bin` に入る．新しくするときは `uv tool upgrade rcb4`，消すときは `uv tool uninstall rcb4` である．
仮想環境を作ったり `pip install` したりする必要は無い．

入るコマンドは次のとおりである（それぞれ `--help` で使い方が出る）．

| コマンド | 中身 |
|:---------|:-----|
| `ics-manager` | Dual USB アダプターにつないだサーボ 1 個の ID・角度・スピード・ストレッチなどを見て，変える |
| `rcb4-shell` | RCB-4 / RCB-4mini につなぎ，それを `ri` として Python の対話環境を開く |
| `rcb4-rom` | RCB-4 / RCB-4mini の ROM を表示（`show`）・保存（`backup`）・書き戻し（`restore`）する |
| `rcb4-write-firmware` | ファームウェア（kondoh7）を書き込む．JEFY では使わない |
| `armh7-tools` | ARMH7 ボードとウォームギアモジュール用．JEFY では使わない |

### Linux で Dual USB アダプターを使えるようにする

Linux では，Dual USB アダプターを USB シリアルとして見えるようにする設定が要る．
次の udev ルールを入れておくと，アダプターを挿したときに自動で設定され，`sudo` なしで使える（1 回だけ）．

```{code-block} console
$ curl -fsSL https://raw.githubusercontent.com/iory/rcb4/main/rcb4/assets/system/99-rcb4-udev.rules | sudo tee /etc/udev/rules.d/99-rcb4-udev.rules
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

入れていないと，`ics-manager` が `Connection is not open.` と，その場で設定するための次のコマンドを表示する．

```{code-block} console
$ sudo su
# modprobe ftdi-sio
# echo 165C 0008 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
# exit
```

(ubuntu-ics-manager)=
## ics-manager でサーボの ID を振る

KRS サーボは出荷時に ID が全て**0**になっている．
同じ RCB-4mini につなぐサーボには，それぞれ違う ID を振っておく必要がある．
演習で配るサーボは既に ID が振られていることがあるので，まず今の ID を確かめること．

JEFY の ID は次のとおりである（[組み立てのページ](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs.html)）．

| | 右前 | 左前 | 右後 | 左後 |
|:--|:--:|:--:|:--:|:--:|
| 股（胴体側） | 1 | 3 | 5 | 7 |
| 膝 | 2 | 4 | 6 | 8 |

**手順：**

1. Dual USB アダプターのスライドスイッチを**ICS 側**にする．USB につないだとき，LED が赤なら ICS モード，緑ならシリアルモードである．
2. Dual USB アダプターにサーボを**1 個だけ**つないでから，アダプターを PC の USB ポートに挿す．
3. `ics-manager` を起動する．

```{code-block} console
$ ics-manager
```

```{figure} ../fig/krs-dual-usb-connection.jpg
:name: krs-dual-usb-connection
:align: center

Dual USBアダプターとサーボの接続例（スライドスイッチをICS側に設定）
```

4. 画面にサーボの状態が出る．`Baud Rate` が `1.25Mbps` になっていることを確かめる．
5. `Current Servo ID` が選ばれている（`>>` が付いている）状態で，←→キーで新しい ID を選ぶ（緑色で表示される）．
6. **Enter キー**を押すと，その ID が書き込まれる．`Current Servo ID` が新しい ID に変わったことを確かめる．
7. `q` で終わる．サーボを付け替えて，全部のサーボについて繰り返す．

**ics-manager の画面の例：**

```
--- Servo Status ---
>>  Current Servo ID: 0 -> Next Servo ID: [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 ]
    Angle: -120
    Speed: 127
    Stretch: 127
    Baud Rate: 1.25Mbps
    Rotation Mode: Disabled
    Slave Mode: Disabled
    Reverse Mode: Disabled
    Serial Mode: Disabled
    Free: Enabled
----------------------
```

**キー操作：**

| キー | 動き |
|:-----|:-----|
| ↑ / ↓ | 変える項目（`Current Servo ID`，`Angle`，`Speed`，`Stretch`）を選ぶ |
| ← / → | 選んだ項目の値を変える（`Current Servo ID` では書き込む ID を選ぶ．`Angle` ではサーボが動く） |
| Enter | `Current Servo ID` を選んでいるとき，緑の ID を書き込む |
| `z` | サーボを真ん中（0 度）に戻す |
| `f` | フリー（脱力）にする |
| `r` | 回転モード（車輪のように回り続ける）を切り替える |
| `v` | リバース（回る向きを逆にする）を切り替える |
| `d` | EEPROM の設定を既定値に戻す（`y` で確定．ID・通信速度・サーボ固有の校正値はそのまま） |
| `q` | 終わる |

```{warning}
`ics-manager` は，サーボの通信速度が `--baudrate`（既定 1250000，つまり 1.25 Mbps）と違うと，
`--baudrate` の値をサーボの EEPROM に書き込む（画面に `Servo baud rate was ...: wrote ... to its EEPROM` と出る）．
JEFY のサーボは 1.25 Mbps のままでよいので，`--baudrate` は付けずに起動する．
別の通信速度で使うサーボのときだけ，`--baudrate 115200` などを付ける．
```

### Windows の場合

Windows では，近藤科学の ICS3.5/3.6 Manager ソフトウェア[^ics-manager]と，アダプターのドライバ（KO Driver）を使う．
サーボを 1 個だけつないで ID を検索し，新しい ID に変えて保存する．

## サーボの角度

KRS サーボの位置は，ICS の位置の値**3500〜11500**で指定する．

- **3500**: 最小（約 -135 度）
- **7500**: 真ん中（0 度）
- **11500**: 最大（約 +135 度）

位置の値と角度の関係は次の式で表される（真ん中から ±135 度，合わせて 270 度）．

$$
\text{角度[度]} = \frac{(\text{位置} - 7500) \times 270}{8000}
$$

KRS-3302 が角度を測れるのも，この ±135 度の範囲だけである．
`ics-manager` の `Angle` は，この角度（度）を表示している．

### サーボの零点（0度位置）の確認

KRS サーボの軸には小さいドット（マーク）が打ってある．サーボが 0 度（位置の値 7500）のとき，このドットがサーボケースの真ん中に来る．
JEFY を組み立てるときは，`ics-manager` の `z` でサーボを真ん中に合わせてからホーンと脚を付ける．

```{figure} ../fig/krs-zero-point.jpg
:name: krs-zero-point
:align: center

KRSサーボの零点マーク（軸のドットがサーボの真ん中に来ると0度）
```

## RCB-4mini へのつなぎ方

- サーボは RCB-4mini の **SIO** ポートにつなぐ．
- RCB-4mini の **COM** ポートを Arduino UNO R4 WiFi の **D8 / D9** につなぐ（D0 / D1 は空けておく）．
- RCB-4mini の COM ポートは信号が反転した特殊なシリアル（1.25 Mbps）なので，Arduino の `Serial1` では話せない．
  そのための書き込み用のコードは [build-your-own-arduino-rtos](https://github.com/iory/build-your-own-arduino-rtos) にある．

詳しい配線と書き込みの手順は {doc}`../robot-programming-3-2026` の課題1と，[組み立てのページ](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs.html) を見よ．

## トラブルシューティング

### ics-manager が `Connection is not open.` のまま進まない

- Dual USB アダプターのスライドスイッチが ICS 側か（LED が赤か）．
- サーボを 1 個だけつないでいるか．複数つなぐと答えが混ざる．
- サーボに電源が入っているか．コネクタの向きが正しく，しっかり挿さっているか．
- Linux では，上の udev ルールを入れたか（または画面に出る `modprobe` のコマンドを実行したか）．
- 他のプログラム（別の `ics-manager` など）が同じアダプターを開いていないか．

### サーボが RCB-4mini から動かない

1. 電源：バッテリーが充電されているか（6 V を切っていないか），電源スイッチが ON か．
2. 配線：サーボのコネクタの向きと挿し込み，RCB-4mini と Arduino の配線（D8 / D9）．
3. ID：プログラムが使う ID とサーボの ID が一致しているか．同じ ID のサーボが 2 個ないか．
4. 通信速度：`ics-manager` の `Baud Rate` が `1.25Mbps` か．

## 参考資料

[^ics-manager]: 近藤科学 - ICS3.5/3.6 Manager software: <https://kondo-robot.com/faq/ics35mag>

- [近藤科学の KRS サーボで組む（RCB-4mini 版）](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs.html)
- [部品表（BOM）— KRS 版 8 自由度四脚ロボット](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs_bom.html)
- [近藤科学 - ICS3.5/3.6の説明](https://kondo-robot.com/faq/ics3-5-explain)
- [近藤科学 - KRSサーボシリーズ](https://kondo-robot.com/product-category/servomotor/krs)
- [rcb4パッケージ（GitHubリポジトリ）](https://github.com/iory/rcb4)
