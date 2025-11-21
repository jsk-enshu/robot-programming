#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys
from pathlib import Path

# build_html.py から共通関数を import
from build_html import replace_full_width_period, restore_files, setup_venv


def patch_latex_for_lualatex(latex_file: Path) -> None:
    """LaTeXファイルをLuaLaTeX用にパッチ適用

    Args:
        latex_file: lecture.texのパス
    """
    content = latex_file.read_text(encoding="utf-8")

    # ucharclassesをコメントアウト
    content = content.replace(
        r"\usepackage[Latin,Greek]{ucharclasses}",
        r"% \usepackage[Latin,Greek]{ucharclasses} % Commented out for LuaLaTeX"
    )

    latex_file.write_text(content, encoding="utf-8")


def compile_with_lualatex(latex_dir: Path, passes: int = 3) -> None:
    """LuaLaTeXでコンパイル

    Args:
        latex_dir: LaTeXファイルのあるディレクトリ
        passes: コンパイル回数（参照を解決するため）
    """
    for i in range(1, passes + 1):
        print(f"Step {i + 5}: Compiling with LuaLaTeX (pass {i}/{passes})...")

        # 最終パス以外は出力を抑制
        if i < passes:
            result = subprocess.run(
                ["lualatex", "-interaction=nonstopmode", "lecture.tex"],
                cwd=latex_dir,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True
            )
        else:
            result = subprocess.run(
                ["lualatex", "-interaction=nonstopmode", "lecture.tex"],
                cwd=latex_dir,
                stderr=subprocess.PIPE,
                text=True
            )

        if result.returncode != 0:
            print(f"Warning: LuaLaTeX returned non-zero exit code on pass {i}")
            if result.stderr:
                print(f"Error output: {result.stderr}")


def main() -> None:
    """メイン処理"""
    print("=== Jupyter Book PDF Build Script ===")

    # プロジェクトルートに移動
    project_root = Path(__file__).parent
    os.chdir(project_root)

    # venvディレクトリのパス
    venv_dir = project_root / ".venv"

    try:
        # ステップ1: venvのセットアップ
        print("Step 1: Setting up virtual environment...")
        python_path, jupyter_book_path = setup_venv(venv_dir)

        # ステップ2: 全角ピリオドを半角ピリオドに置換
        print("Step 2: Replacing full-width period with half-width period...")
        target_files = []

        # ルートディレクトリのMarkdownファイル
        for pattern in ["*.md", "appendix-*.md", "robot-programming-*.md"]:
            target_files.extend(project_root.glob(pattern))

        # tipsディレクトリのMarkdownファイル
        tips_dir = project_root / "tips"
        if tips_dir.exists():
            target_files.extend(tips_dir.glob("*.md"))

        backup_files = replace_full_width_period(target_files)

        # ステップ3: LaTeXファイルを生成
        print("Step 3: Generating LaTeX files...")
        subprocess.run(
            [str(jupyter_book_path), "build", ".", "--builder", "latex"],
            check=True
        )

        # ステップ4: 元のファイルに戻す
        print("Step 4: Restoring original files...")
        restore_files(backup_files)

        # ステップ5: lecture.texをパッチ
        print("Step 5: Patching lecture.tex for LuaLaTeX...")
        latex_dir = project_root / "_build" / "latex"
        lecture_tex = latex_dir / "lecture.tex"
        patch_latex_for_lualatex(lecture_tex)

        # ステップ6-8: LuaLaTeXでコンパイル（3回）
        compile_with_lualatex(latex_dir, passes=3)

        # ステップ9: PDFをプロジェクトルートにコピー
        print("Step 9: Copying PDF to project root...")
        src_pdf = latex_dir / "lecture.pdf"
        dst_pdf = project_root / "知能ロボット行動プログラミング演習2025.pdf"

        if src_pdf.exists():
            shutil.copy2(src_pdf, dst_pdf)

            print()
            print("=== PDF Build Complete ===")
            print(f"Output: {dst_pdf.name}")

            # ファイルサイズを表示
            size_bytes = dst_pdf.stat().st_size
            size_mb = size_bytes / (1024 * 1024)
            print(f"File size: {size_mb:.2f} MB ({size_bytes} bytes)")
        else:
            print("Error: lecture.pdf was not generated", file=sys.stderr)
            sys.exit(1)

    except subprocess.CalledProcessError as e:
        print(f"Error during build: {e}", file=sys.stderr)
        # エラーが発生してもバックアップを復元
        if 'backup_files' in locals():
            print("Restoring original files due to error...")
            restore_files(backup_files)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        # エラーが発生してもバックアップを復元
        if 'backup_files' in locals():
            print("Restoring original files due to error...")
            restore_files(backup_files)
        sys.exit(1)


if __name__ == "__main__":
    main()
