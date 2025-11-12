# apportエラーレポートの無効化

## apportとは

apportは，Ubuntuに標準搭載されているクラッシュレポートシステムである．アプリケーションが予期せず終了した際に，下図のようなエラーダイアログを表示し，開発者への問題報告を促す機能を提供する．

:::{figure} ../fig/apport.png
:align: center
:name: fig:apport-dialog

Ubuntu 24.04でapportが表示するエラーダイアログ
:::

## 演習中にapportを無効化する理由

ロボットプログラミング演習では，RVizやGazeboなどのROSツールを頻繁に起動・終了する．これらのツールは時折予期せず終了することがあり，その度にapportのダイアログが表示されると作業の妨げになる．

演習中は以下の理由からapportを無効化することを推奨する：

- **作業効率の向上**：エラーダイアログが表示されなくなり，作業に集中できる
- **不要な中断の削減**：既知の問題や一時的なクラッシュに対してダイアログが表示されなくなる
- **デバッグの簡素化**：ターミナルのエラーメッセージに集中できる

## 無効化の方法

以下のコマンドでapportを無効化できる：

```{code-block} console
$ sudo sed -i 's/enabled=1/enabled=0/g' /etc/default/apport
```

このコマンドは，`/etc/default/apport`ファイル内の`enabled=1`を`enabled=0`に書き換えることで，apportを無効化する．

### 設定の確認

設定が正しく変更されたか確認する：

```{code-block} console
$ cat /etc/default/apport
enabled=0
```

`enabled=0`と表示されれば，apportが無効化されている．

### 即座に反映させる

設定を即座に反映させるには，apportサービスを停止する：

```{code-block} console
$ sudo systemctl stop apport.service
$ sudo systemctl disable apport.service
```

## 再度有効化する方法

演習終了後にapportを再度有効化したい場合は，以下のコマンドを実行する：

```{code-block} console
$ sudo sed -i 's/enabled=0/enabled=1/g' /etc/default/apport
$ sudo systemctl enable apport.service
$ sudo systemctl start apport.service
```

## まとめ

apportは通常のシステム運用では有用なツールであるが，ロボットプログラミング演習のような集中的な開発作業では，頻繁なダイアログ表示が作業の妨げになることがある．演習中は無効化し，演習終了後に再度有効化することで，快適な開発環境を維持できる．
