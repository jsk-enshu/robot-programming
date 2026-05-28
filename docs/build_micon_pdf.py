#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys
from pathlib import Path

# 共通関数を import
from build_html import replace_full_width_period, restore_files, setup_venv
from build_pdf import compile_with_lualatex, patch_latex_for_lualatex

# マイコンプログラミング演習で扱うTips・付録（_toc.ymlの該当パートと同じ並び）
MICON_TIPS = [
    ("tips/arduino-ide", "Arduino IDEインストール"),
    ("tips/arduino-device-detection", "Arduinoが認識されないときの確認方法"),
    ("tips/arduino-serial-monitor", "Arduino IDEのシリアルモニタ"),
    ("tips/arduino-builtin-led", "ArduinoのLED_BUILTIN"),
]

# マイコン演習サブセット用の_toc.yml（rootをmicon-programmingにする）
MICON_TOC = (
    "format: jb-book\n"
    "root: micon-programming\n"
    "parts:\n"
    "  - caption: Tips・付録（マイコンプログラミング演習）\n"
    "    numbered: false\n"
    "    chapters:\n"
    + "".join(
        f"    - file: {f}\n      title: {t}\n" for f, t in MICON_TIPS
    )
)

# タイトル文言と担当者名簿はmicon-programming.mdを正本として抽出する（情報の重複を避ける）。
# 年度や改行位置などPDFカバー固有の体裁だけをここで持つ
SOURCE_MD = "micon-programming.md"
YEAR = "2026"

OUTPUT_PDF_NAME = f"マイコンプログラミング演習{YEAR}.pdf"


def extract_title(md_text: str) -> str:
    """micon-programming.mdのH1見出し（# ...）からタイトル文言を取得する"""
    for line in md_text.splitlines():
        if line.startswith("# "):
            return line[2:].strip()
    raise ValueError(f"{SOURCE_MD} にH1見出し（# ...）が見つかりません")


def extract_author_groups(md_text: str) -> list[list[str]]:
    """「担当教員・TA」admonitionから所属グループ（先頭=見出し）を抽出する

    **見出し** を新しいグループの先頭、続く ``- 項目`` をそのメンバーとして読む
    """
    groups: list[list[str]] = []
    current = None
    in_block = False
    for line in md_text.splitlines():
        stripped = line.strip()
        if not in_block:
            if "{admonition}" in stripped and "担当教員" in stripped:
                in_block = True
            continue
        if stripped.startswith("```"):  # admonitionの閉じフェンスで終了
            break
        if stripped.startswith("**") and stripped.endswith("**"):
            current = [stripped.strip("*").strip()]
            groups.append(current)
        elif stripped.startswith("- ") and current is not None:
            current.append(stripped[2:].strip())
    if not groups:
        raise ValueError(f"{SOURCE_MD} に「担当教員・TA」の名簿が見つかりません")
    return groups


def build_pdf_title(md_text: str) -> str:
    """H1文言の前に年度を付け、半角スペース位置（「夏学期演習」の後）でLaTeX改行する"""
    base = extract_title(md_text)
    return f"{YEAR}年度 " + base.replace(" ", r" \\ ")


def build_pdf_author(md_text: str) -> str:
    r"""名簿をPDFカバー用の \author 文字列に組み立てる

    \author内はtabular環境なので \\ で改行。長い行が溢れないよう\largeに落とし、
    所属グループ間は \\[0.8em] で余白を空ける。内側tabular{l}で各行を左揃えにし、
    ブロック全体は外側tabular{c}で中央配置になる
    """
    groups = extract_author_groups(md_text)
    body = r" \\[0.8em] ".join(
        r" \\ ".join(r"\large " + line for line in group)
        for group in groups
    )
    return r"\begin{tabular}{@{}l@{}} " + body + r" \end{tabular}"


def patch_config_for_micon(
    config_text: str, pdf_title: str, pdf_author: str
) -> str:
    """_config.ymlのLaTeXマスタードキュメント・タイトル・著者を差し替える

    Args:
        config_text: 元の_config.ymlの内容
        pdf_title: PDFカバーのタイトル（LaTeX文字列）
        pdf_author: PDFカバーの著者（LaTeX文字列）

    Returns:
        str: マイコン演習サブセット用に書き換えた内容
    """
    # latex_documentsの開始ドキュメントをindexからmicon-programmingに変更
    patched = config_text.replace(
        '      - - index\n        - "lecture.tex"',
        '      - - micon-programming\n        - "lecture.tex"',
    )
    # PDFカバーの\titleは_config.yml先頭のtitle:から設定されるため、ここを差し替える。
    # シングルクォートでLaTeXの\\（改行）をそのまま保持する
    patched = patched.replace(
        "title: 知能ロボット行動プログラミング演習 2025",
        f"title: '{pdf_title}'",
    )
    # \author（latex_documentsの著者欄）を所属＋担当者リストに差し替える。
    # YAMLシングルクォートはバックスラッシュをそのまま保持するのでLaTeXの\\が壊れない
    patched = patched.replace(
        '        - "JSK Robotics Laboratory"',
        f"        - '{pdf_author}'",
    )
    return patched


def main() -> None:
    """メイン処理"""
    print("=== マイコンプログラミング演習 PDF Build Script ===")

    # プロジェクトルートに移動
    project_root = Path(__file__).parent
    os.chdir(project_root)

    # venvディレクトリのパス
    venv_dir = project_root / ".venv"

    # _toc.yml / _config.yml の元の内容を保持し、finallyで必ず復元する
    toc_path = project_root / "_toc.yml"
    config_path = project_root / "_config.yml"
    original_toc = toc_path.read_text(encoding="utf-8")
    original_config = config_path.read_text(encoding="utf-8")

    # タイトル・著者はmicon-programming.mdから抽出する（全角ピリオド置換の前に読む）
    source_md_text = (project_root / SOURCE_MD).read_text(encoding="utf-8")
    pdf_title = build_pdf_title(source_md_text)
    pdf_author = build_pdf_author(source_md_text)

    backup_files: list[Path] = []

    try:
        # ステップ1: venvのセットアップ
        print("Step 1: Setting up virtual environment...")
        python_path, jupyter_book_path = setup_venv(venv_dir)

        # ステップ2: 全角ピリオドを半角ピリオドに置換（ビルド対象ファイルのみ）
        print("Step 2: Replacing full-width period with half-width period...")
        target_files = [project_root / "micon-programming.md"]
        target_files.extend(project_root / f"{f}.md" for f, _ in MICON_TIPS)
        backup_files = replace_full_width_period(target_files)

        # ステップ3: マイコン演習サブセット用の_toc.yml / _config.ymlに差し替え
        print("Step 3: Switching to micon-only _toc.yml and _config.yml...")
        toc_path.write_text(MICON_TOC, encoding="utf-8")
        config_path.write_text(
            patch_config_for_micon(original_config, pdf_title, pdf_author),
            encoding="utf-8",
        )

        # ステップ4: LaTeXファイルを生成
        print("Step 4: Generating LaTeX files...")
        subprocess.run(
            [str(jupyter_book_path), "build", ".", "--builder", "latex"],
            check=True,
        )

        # ステップ5: _toc.yml / _config.yml を元に戻す
        print("Step 5: Restoring original _toc.yml and _config.yml...")
        toc_path.write_text(original_toc, encoding="utf-8")
        config_path.write_text(original_config, encoding="utf-8")

        # ステップ6: 元のMarkdownファイルに戻す
        print("Step 6: Restoring original Markdown files...")
        restore_files(backup_files)
        backup_files = []

        # ステップ7: lecture.texをパッチ
        print("Step 7: Patching lecture.tex for LuaLaTeX...")
        latex_dir = project_root / "_build" / "latex"
        lecture_tex = latex_dir / "lecture.tex"
        patch_latex_for_lualatex(lecture_tex)

        # ステップ8-10: LuaLaTeXでコンパイル（3回）
        compile_with_lualatex(latex_dir, passes=3)

        # ステップ11: PDFをプロジェクトルートにコピー
        print("Step 11: Copying PDF to project root...")
        src_pdf = latex_dir / "lecture.pdf"
        dst_pdf = project_root / OUTPUT_PDF_NAME

        if src_pdf.exists():
            shutil.copy2(src_pdf, dst_pdf)

            print()
            print("=== マイコンプログラミング演習 PDF Build Complete ===")
            print(f"Output: {dst_pdf.name}")

            size_bytes = dst_pdf.stat().st_size
            size_mb = size_bytes / (1024 * 1024)
            print(f"File size: {size_mb:.2f} MB ({size_bytes} bytes)")
        else:
            print("Error: lecture.pdf was not generated", file=sys.stderr)
            sys.exit(1)

    except subprocess.CalledProcessError as e:
        print(f"Error during build: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        # 例外が発生しても_toc.yml / _config.yml / Markdownを必ず元に戻す
        toc_path.write_text(original_toc, encoding="utf-8")
        config_path.write_text(original_config, encoding="utf-8")
        if backup_files:
            print("Restoring original Markdown files...")
            restore_files(backup_files)


if __name__ == "__main__":
    main()
