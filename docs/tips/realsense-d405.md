# Intel RealSense Depth Camera D405のセットアップと活用

## Intel RealSense D405について

Intel RealSense Depth Camera D405は，ステレオビジョン方式を採用した深度カメラである．特に近距離での深度測定に優れており，**約7cmから測定可能**という特徴がある．

[公式製品ページ](https://www.intel.co.jp/content/www/jp/ja/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html)

### 主な特徴

- **近距離測定**: 7cm〜50cmの範囲で高精度な深度測定が可能
- **ステレオビジョン方式**: 2つのカメラを使った三角測量による距離測定
- **小型・軽量**: ロボットの手先やヘッド部に搭載可能なコンパクト設計
- **USB接続**: PCに接続するだけで使用可能

## ロボティクスでの活用例

### 大型ロボットでのマニピュレーション

大型のロボットマニピュレータの手先（エンドエフェクタ）にD405を取り付けることで，以下のような利点がある：

- **把持対象物の深度情報取得**: ロボットハンドに近い物体の正確な3D位置を把握
- **マニピュレーション精度の向上**: 対象物までの距離を正確に測定し，安全で確実な把持を実現
- **近接作業の支援**: 細かい組み立て作業や精密な位置決めに必要な視覚情報を提供

### 小型ロボット（Jedy）での活用

Jedyのような小型の移動ロボットに搭載した場合：

- **自己の手先（マニピュレータ）の視認**: カメラの視野内にロボット自身のアームや手先が映り，作業状態をリアルタイムで確認可能
- **近接物体の認識**: ロボットの周辺にある小さな物体や障害物を検出
- **テーブル上の作業**: 机上の物体をピックアップする際の位置推定に活用

## ステレオカメラの測距原理

### ステレオビジョンとは

ステレオカメラは，人間の両眼視と同じ原理で距離を測定する：

1. **視差（Disparity）の利用**: 2つのカメラで同じ対象を異なる角度から撮影すると，画像上での対象物の位置がずれる．このずれ（視差）が距離に対応する．

2. **三角測量**:
   - 2つのカメラ間の距離（ベースライン）は既知
   - 各カメラの画像上での対象物の位置から視差を計算
   - 視差と幾何学的な関係から，対象物までの距離を算出

3. **計算式**:
   ```
   距離（Z）= （ベースライン × 焦点距離）/ 視差
   ```

   視差が大きい（対象物が近い）ほど距離は短く，視差が小さい（対象物が遠い）ほど距離は長くなる．

### 測距のプロセス

1. **キャリブレーション**: 2つのカメラの位置関係と内部パラメータを事前に校正
2. **ステレオマッチング**: 左右の画像で同じ特徴点（対応点）を探索
3. **視差マップの生成**: 画像の各ピクセルについて視差を計算
4. **深度マップの生成**: 視差から実際の距離（深度）に変換

### 近距離測定の利点

D405が7cmという近距離から測定できる理由：

- **広角レンズの採用**: 視野角が広く，近い物体も画角に収まる
- **短いベースライン**: カメラ間の距離を最適化し，近距離での視差計算精度を向上
- **高解像度センサー**: 細かい視差の変化を捉えることができる

## Kinectとの違い

### Intel RealSense D405（ステレオビジョン方式）

- **原理**: 2つのカメラによる三角測量
- **光源**: 不要（自然光や室内照明で動作）
- **測定範囲**: 7cm〜50cm（近距離特化）
- **屋外使用**: 可能（太陽光下でも動作）
- **利点**: パッシブ方式のため省電力，他のセンサーとの干渉がない

### Kinect（赤外線構造化光方式）

- **原理**: 赤外線パターンを投射し，その歪みから距離を測定
- **光源**: 赤外線プロジェクタが必要
- **測定範囲**: 0.5m〜4.5m（中距離向け）
- **屋外使用**: 困難（太陽光の赤外線が干渉）
- **利点**: 単眼でも動作，テクスチャのない面でも測定可能

### 使い分け

- **近距離マニピュレーション**: RealSense D405
- **室内ナビゲーション・全身トラッキング**: Kinect
- **屋外での深度測定**: RealSense D405（ステレオ方式）

## インストール方法

Ubuntu環境でIntel RealSense D405を使用するには，librealsense2をインストールする必要がある．以下では，パッケージからのインストールとソースからのインストールの2つの方法を説明する．

### パッケージからのインストール（推奨）

最も簡単な方法は，Intel公式のAPTリポジトリからパッケージをインストールすることである：

```{code-block} console
# 1. GPGキー用のディレクトリを作成
$ sudo mkdir -p /etc/apt/keyrings

# 2. Intel RealSenseのGPGキーをダウンロード
$ curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# 3. APTリポジトリを追加
$ echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list

# 4. パッケージリストを更新
$ sudo apt-get update

# 5. librealsense2-utilsをインストール
$ sudo apt-get install librealsense2-utils librealsense2-dev
```

### ソースからのインストール

最新の機能やバグ修正が必要な場合，または特定のビルドオプションを指定したい場合は，ソースからビルドすることができる：

```{code-block} console
# 1. ソースコード用のディレクトリを作成
$ mkdir -p ~/src/github.com/IntelRealSense
$ cd ~/src/github.com/IntelRealSense

# 2. librealsenseリポジトリをクローン
$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd librealsense/

# 3. ビルドに必要な依存パッケージをインストール
$ sudo apt install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev cmake

# 4. udevルールをセットアップ
$ ./scripts/setup_udev_rules.sh

# 5. ビルドディレクトリを作成してビルド
$ mkdir build && cd build
$ cmake ../ -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false
$ sudo make clean && make && sudo make install
```

**ビルドオプションの説明：**

- `-DCMAKE_BUILD_TYPE=release`: リリースビルド（最適化有効）
- `-DBUILD_EXAMPLES=false`: サンプルプログラムをビルドしない
- `-DBUILD_GRAPHICAL_EXAMPLES=false`: GUIサンプルをビルドしない

GUIサンプル（`realsense-viewer`など）が必要な場合は，`-DBUILD_GRAPHICAL_EXAMPLES=true`に変更する．

### インストール確認

カメラを接続して，以下のコマンドで動作確認する：

```{code-block} console
# RealSenseビューアを起動
realsense-viewer
```

このビューアで，カラー画像と深度画像がリアルタイムで表示されれば，正常にインストールされている．

## ROSでの使用

ROSからRealSenseカメラを使用する場合は，以下のパッケージをインストールする：

```{code-block} console
sudo apt-get install ros-${ROS_DISTRO}-realsense2-camera
```

起動方法：

```{code-block} console
roslaunch realsense2_camera rs_camera.launch
```

これにより，以下のトピックが配信される：

- `/camera/color/image_raw`: カラー画像
- `/camera/depth/image_rect_raw`: 深度画像
- `/camera/aligned_depth_to_color/image_raw`: カラー画像に位置合わせされた深度画像
- `/camera/color/camera_info`: カメラキャリブレーション情報

## 参考情報

- [Intel RealSense 公式ドキュメント](https://www.intelrealsense.com/developers/)
- [librealsense GitHub](https://github.com/IntelRealSense/librealsense)
- [ROS RealSense wrapper](https://github.com/IntelRealSense/realsense-ros)
