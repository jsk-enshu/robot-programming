#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys
from pathlib import Path


def setup_venv(venv_dir: Path) -> tuple[Path, Path]:
    """venvのセットアップと必要なパッケージのインストール

    Returns:
        tuple[Path, Path]: (python_path, pip_path)
    """
    if sys.platform == "win32":
        python_path = venv_dir / "Scripts" / "python.exe"
        pip_path = venv_dir / "Scripts" / "pip.exe"
        jupyter_book_path = venv_dir / "Scripts" / "jupyter-book.exe"
    else:
        python_path = venv_dir / "bin" / "python"
        pip_path = venv_dir / "bin" / "pip"
        jupyter_book_path = venv_dir / "bin" / "jupyter-book"

    # venvが存在しない場合は作成
    if not venv_dir.exists():
        print(f"Creating virtual environment: {venv_dir}")
        subprocess.run([sys.executable, "-m", "venv", str(venv_dir)], check=True)
        print("Virtual environment created successfully")

    # jupyter-bookがインストールされているか確認
    if not jupyter_book_path.exists():
        print("Installing jupyter-book and required extensions...")
        subprocess.run([str(pip_path), "install", "--upgrade", "pip"], check=True)
        subprocess.run(
            [
                str(pip_path),
                "install",
                "jupyter-book<2.0",  # Use v1系 to avoid v2.0 API changes
                "sphinx-copybutton",
                "sphinx-exercise",
                "sphinxcontrib-images",
            ],
            check=True,
        )
        print("jupyter-book and extensions installed successfully")

    return python_path, jupyter_book_path


def replace_full_width_period(files: list[Path]) -> list[Path]:
    """全角ピリオドを半角ピリオドに置換

    Args:
        files: 処理対象のファイルリスト

    Returns:
        list[Path]: バックアップファイルのリスト
    """
    backup_files = []

    for file_path in files:
        if not file_path.exists():
            continue

        # バックアップファイルを作成
        backup_path = Path(str(file_path) + ".bak")
        shutil.copy2(file_path, backup_path)
        backup_files.append(backup_path)

        # 全角ピリオドを半角ピリオドに置換
        content = file_path.read_text(encoding="utf-8")
        content = content.replace("。", "．")
        content = content.replace("、", "，")
        file_path.write_text(content, encoding="utf-8")

        print(f"  Replaced period in {file_path}")

    return backup_files


def restore_files(backup_files: list[Path]) -> None:
    """バックアップファイルから元のファイルを復元

    Args:
        backup_files: バックアップファイルのリスト
    """
    for backup_path in backup_files:
        if not backup_path.exists():
            continue

        original_path = Path(str(backup_path).replace(".bak", ""))
        shutil.move(str(backup_path), str(original_path))
        print(f"  Restored {original_path}")


def copy_images(build_dir: Path) -> None:
    """画像ファイルをビルドディレクトリにコピー

    Args:
        build_dir: ビルド出力ディレクトリ
    """
    fig_src = Path("fig")
    fig_dst = build_dir / "html" / "fig"

    # 出力ディレクトリを作成
    fig_dst.mkdir(parents=True, exist_ok=True)

    # 画像・動画ファイルをコピー
    file_count = 0
    for ext in ["*.jpg", "*.png", "*.mp4"]:
        for file in fig_src.glob(ext):
            shutil.copy2(file, fig_dst / file.name)
            file_count += 1

    print(f"  Total files copied: {file_count}")


def main() -> None:
    """メイン処理"""
    print("=== Jupyter Book HTML Build Script ===")

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

        # ステップ3: HTMLビルド
        print("Step 3: Building HTML...")
        subprocess.run([str(jupyter_book_path), "build", "."], check=True)

        # ステップ4: 元のファイルに戻す
        print("Step 4: Restoring original files...")
        restore_files(backup_files)

        # ステップ5: 画像をコピー
        print("Step 5: Copying images...")
        build_dir = project_root / "_build"
        copy_images(build_dir)

        # ステップ6: 確認
        print("Step 6: Verifying...")

        print()
        print("=== HTML Build Complete ===")
        print("Open _build/html/index.html in your browser")
        print("Or run: open _build/html/index.html")

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
