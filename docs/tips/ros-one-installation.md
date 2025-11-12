# ROS-O (ROS One) のインストールと使用方法

## ROS-Oについて

ROS-O (ROS One) は，Ubuntu 20.04以降のバージョンで動作するROS 1パッケージを提供するコミュニティリポジトリである．OSRF（Open Source Robotics Foundation）が公式にサポートしていた最後のバージョンはUbuntu 20.04であったが，ROS-Oはそれ以降のUbuntuリリースでもROS 1を利用可能にしている．

### 特徴

- **Ubuntu 24.04 (Noble Numbat) 対応**: 最新のUbuntu環境でROS 1を使用可能
- **GitHub Actionsによる自動ビルド**: 継続的にパッケージが更新される
- **コミュニティベース**: 必要なパッケージのリクエストを受け付けている

## 学科PCでの使用方法

**学科のPCには既にROS-Oがインストールされている**ため，以下のコマンドを実行するだけで使用できる：

```{code-block} console
source /opt/ros/one/setup.bash
```

このコマンドをターミナルで実行すると，ROS-Oの環境が有効になり，`roscore`や`roslaunch`などのROSコマンドが使用可能になる．

### 自動的に有効化する方法

毎回sourceコマンドを実行するのが面倒な場合は，`~/.bashrc`に以下の行を追加する：

```{code-block} console
echo "source /opt/ros/one/setup.bash" >> ~/.bashrc
```

この設定により，新しいターミナルを開くたびに自動的にROS-O環境が有効化される．

## 個人PCへのインストール方法

個人のPCにROS-Oをインストールする場合は，以下の手順に従う．

### 1. ROS-Oのaptリポジトリを設定

```{code-block} console
# curlをインストール（まだインストールされていない場合）
sudo apt install curl

# GPGキーをダウンロード
sudo curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg

# aptリポジトリを追加
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros1.list

# デバッグパッケージ用リポジトリ（コメントアウトされた状態で追加）
echo "# deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main-dbg" | sudo tee -a /etc/apt/sources.list.d/ros1.list
```

### 2. rosdepのインストールと設定

rosdepは，ROSパッケージの依存関係を管理するツールである．

```{code-block} console
# パッケージリストを更新
sudo apt update

# rosdepをインストール
# 注意: python3-rosdep2はインストールしないこと（古いバージョンである）
sudo apt install python3-rosdep

# rosdepを初期化
sudo rosdep init
```

### 3. ROS-O用のカスタムrosdepマッピングを定義

```{code-block} console
# ROS-O用のパッケージマッピングを追加
echo "yaml https://ros.packages.techfak.net/ros-one.yaml one" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list

# rosdepデータベースを更新
rosdep update
```

### 4. ROSパッケージのインストール

必要なROSパッケージをインストールする．例えば，デスクトップ版をインストールする場合：

```{code-block} console
sudo apt install ros-one-desktop
```

その他のパッケージも同様に`ros-one-`というプレフィックスを付けてインストールできる．

### 5. 環境設定

インストール後，以下のコマンドでROS環境を有効化する：

```{code-block} console
source /opt/ros/one/setup.bash
```

毎回sourceする手間を省くため，`~/.bashrc`に追加することを推奨する：

```{code-block} console
echo "source /opt/ros/one/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 最新ビルドの使用（オプション）

最新のビルドを使用したい場合は，`-testing`リポジトリを利用できる．手順1でリポジトリを追加する際に，`$(lsb_release -cs)`を`$(lsb_release -cs)-testing`に置き換える．

**注意**: testing版は安定性が保証されていないため，本番環境での使用は推奨されない．

## ビルドスケジュールと同期

- **Nightly Build**: パッケージは毎晩自動的にビルドされ，`-testing`リポジトリにリリースされる
- **メインリポジトリへの同期**: 約2ヶ月ごとに不定期でメインリポジトリに同期される

## 参考情報

- [ROS-O 公式リポジトリ](https://ros.packages.techfak.net/dists/noble/)
- [ROS Builder Action (GitHub)](https://github.com/ubi-agni/ros-builder-action)
- [ROS-O Organization (GitHub)](https://github.com/ros-o/ros-o)

パッケージの追加リクエストは，[ros-builder-action](https://github.com/ubi-agni/ros-builder-action)の`ros-one.repos`ファイルにPRを送ることで可能である．
