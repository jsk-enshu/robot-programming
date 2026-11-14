# Google AI Studio APIキーの発行

## 機械情報工学科3年生の場合

機械情報工学科の3年生は，「HRI研究法入門」で配布されたAPI_KEYを使用すればよい．

## APIキーの新規発行

Google AI StudioでAPIキーを新規に発行する場合は，以下の手順で行う．
Googleアカウントがあればよく，お支払い情報（クレジットカード）の設定は要らない．
以下の画面は2026年11月のもので，キーやアカウントの情報は灰色で隠している．

1. [Google AI Studio API Keys](https://aistudio.google.com/api-keys) にアクセスする．

2. 右上の「API キーを作成」ボタンをクリックする．

:::{figure} ../fig/google-ai-studio-api-keys.png
:align: center
:name: fig:google-ai-studio-api-keys

Google AI StudioのAPIキーの一覧．右上が「API キーを作成」，各キーの右のアイコンの左端がキーのコピー
:::

3. 「新しいキーを作成する」が開くので，「キー名の設定」にキー名を入力する．
   - 任意のキー名（例：`jsk-enshu`）を入力する．最初から「Gemini API Key」が入っているので，消してから入力する

4. 「インポートしたプロジェクトを選択」でプロジェクトを選ぶ．
   - プロジェクトが無い場合は，ここを開いて「プロジェクトを作成」をクリックし，適当な名前を入力してプロジェクトを作成する

:::{figure} ../fig/google-ai-studio-key-project.png
:align: center
:name: fig:google-ai-studio-key-project

プロジェクトの選択．プロジェクトが無ければ「プロジェクトを作成」
:::

5. 「キーを作成」ボタンをクリックする．

:::{figure} ../fig/google-ai-studio-key.png
:align: center
:name: fig:google-ai-studio-key

Google AI Studioのキー作成ダイアログ
:::

6. 「API キーの詳細」が開くので，右下の「鍵をコピー」ボタンをクリックして，APIキーをコピーする．
   - キーは `AQ.` で始まる文字列である
   - 後からコピーするときは，一覧でそのキーの行の右にあるアイコンの左端（四角が2つ重なったもの）をクリックする

:::{figure} ../fig/google-ai-studio-key-details.png
:align: center
:name: fig:google-ai-studio-key-details

作成したキーの詳細．「鍵をコピー」でキーをコピーする
:::

7. コピーしたAPIキーを安全な場所に保存する．
   - キーは他人に見せない．GitHubなどに上げるファイルや，画面写真に入れないこと

一覧の「請求階層」が「無料枠」なら，お金はかからない．
「お支払い情報を設定」は押さなくてよい．

## 演習 (2026) の VLM で使う

ロボットプログラミング演習 1 日目の `ri.ask`，`ri.point`，`vlm_navigate` は，AI TA の代わりに自分のキーでも使える．
AI TA に登録したくない人，カメラ画像を AI TA 経由で送ることに同意しない人は，こちらを使ってよい．
プログラムを動かすターミナルで次を実行する (毎回打たないなら `~/.bashrc` に書く)．

```{code-block} console
$ export GOOGLE_API_KEY=発行したキー
```

設定すると，VLM には AI TA のキーは使わず，Google AI Studio に直接送る．
使えるモデルや無料枠の制限は {ref}`vlm-google-ai-studio` を見ること．
使った回数と上限は，Google AI Studio の左の「レート制限」で見られる．

:::{figure} ../fig/google-ai-studio-rate-limit.png
:align: center
:name: fig:google-ai-studio-rate-limit

「レート制限」の画面 (無料枠)．`ri.ask` の Gemini 3.8 Flash と，`ri.point` の Gemini Robotics ER 2 Preview は，どちらも 1 分 5 回 (RPM)・1 日 20 回 (RPD) まで
:::
