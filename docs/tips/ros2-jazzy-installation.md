# ROS 2 Jazzy Jalisco のインストール方法

## ROS 2 Jazzyについて

ROS 2 Jazzy Jaliscoは，Ubuntu 24.04 (Noble Numbat)向けに提供されているROS 2の公式ディストリビューションである．対象プラットフォームは[REP 2000](https://www.ros.org/reps/rep-2000.html)で定義されている．

**本講義資料を使用する環境では，ROS 2 Jazzyは既にインストールされていると思われる．**したがって，以下のインストール手順は個人のPCにセットアップする場合や，環境を再構築する場合に参照すること．

## 対応プラットフォーム

- Ubuntu Noble (24.04): amd64, arm64

## システムのセットアップ

### ロケールの設定

UTF-8をサポートするロケールが必要である．最小限の環境（Dockerコンテナなど）では，ロケールがPOSIXのような最小限のものになっている可能性がある．

```{code-block} console
# 現在のロケールを確認
locale

# localesパッケージをインストール
sudo apt update && sudo apt install locales

# ロケールを生成
sudo locale-gen en_US en_US.UTF-8

# ロケールを更新
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 環境変数を設定
export LANG=en_US.UTF-8

# 設定を確認
locale
```

### 必要なリポジトリの有効化

ROS 2のaptリポジトリをシステムに追加する必要がある．

まず，Ubuntu Universeリポジトリが有効になっていることを確認する：

```{code-block} console
sudo apt install software-properties-common
sudo add-apt-repository universe
```

次に，ROS 2リポジトリの設定を行う．`ros2-apt-source`パッケージをインストールすると，ROS 2リポジトリが自動的に設定される：

```{code-block} console
# curlとapt-updateをインストール
sudo apt update && sudo apt install curl -y

# 最新のros2-apt-sourceのバージョンを取得
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

# ros2-apt-sourceパッケージをダウンロード
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

# パッケージをインストール
sudo dpkg -i /tmp/ros2-apt-source.deb
```

このパッケージの新しいバージョンがROS リポジトリにリリースされると，リポジトリ設定が自動的に更新される．

### 開発ツールのインストール（オプション）

ROSパッケージをビルドする場合や開発を行う場合は，開発ツールもインストールする：

```{code-block} console
sudo apt update && sudo apt install ros-dev-tools
```

## ROS 2のインストール

### リポジトリキャッシュの更新

リポジトリを設定した後，aptリポジトリキャッシュを更新する：

```{code-block} console
sudo apt update
```

ROS 2パッケージは頻繁に更新されるUbuntuシステム上でビルドされているため，新しいパッケージをインストールする前にシステムを最新の状態にすることが常に推奨される：

```{code-block} console
sudo apt upgrade
```

### Desktop Install（推奨）

ROS，RViz，デモ，チュートリアルを含む完全なデスクトップ環境をインストールする：

```{code-block} console
sudo apt install ros-jazzy-desktop
```

### ROS-Base Install（最小構成）

通信ライブラリ，メッセージパッケージ，コマンドラインツールのみをインストールする．GUIツールは含まれない：

```{code-block} console
sudo apt install ros-jazzy-ros-base
```

## 環境設定

ROS 2を使用するには，セットアップスクリプトをsourceする必要がある：

```{code-block} console
source /opt/ros/jazzy/setup.bash
```

毎回sourceする手間を省くため，`~/.bashrc`に追加することを推奨する：

```{code-block} console
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## インストール確認

ROS 2が正しくインストールされているか確認するには，以下のコマンドを実行する：

```{code-block} console
# ROS 2のバージョン確認
ros2 --version

# 利用可能なパッケージの確認
ros2 pkg list
```

## 参考情報

- [ROS 2 Jazzy 公式インストールガイド](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Status Page](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [REP 2000: ROS 2 Releases and Target Platforms](https://www.ros.org/reps/rep-2000.html)
