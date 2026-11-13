#!/usr/bin/env python3
"""Download VOICEVOX's voices for `speaker --engine voicevox` (ずんだもん etc.).

    ros2 run jedy_speech setup_voicevox            # 0.vvm: ずんだもん, 四国めたん, ...
    ros2 run jedy_speech setup_voicevox --all      # every voice model (~1 GB)

Runs VOICEVOX's own downloader (pinned version, checked by hash). It shows
the voice model and ONNX Runtime terms and asks YOU to accept them: the
models are not redistributed with this package, each user agrees for
themselves. Every voice must be credited ("VOICEVOX:ずんだもん") wherever its
speech is used; read the terms it shows.
"""

import argparse
import hashlib
import os
import subprocess
import urllib.request

from jedy_speech import VOICEVOX_DIR

DOWNLOADER_URL = ("https://github.com/VOICEVOX/voicevox_core/releases/download/"
                  "0.17.1/download-linux-x64")
DOWNLOADER_SHA256 = "4ba6dc22bfacab5dd1a7cc0dc074ca714d920526ce90813aec06dc86531ddd8f"


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--all', action='store_true', help='all voice models, not just 0.vvm')
    p.add_argument('--output', default=VOICEVOX_DIR)
    args = p.parse_args()

    cache = os.path.join(os.path.expanduser('~'), '.cache', 'jedy_speech')
    os.makedirs(cache, exist_ok=True)
    exe = os.path.join(cache, 'voicevox-download-linux-x64-0.17.1')
    if not os.path.exists(exe):
        print('downloading', DOWNLOADER_URL)
        urllib.request.urlretrieve(DOWNLOADER_URL, exe + '.part')
        digest = hashlib.sha256(open(exe + '.part', 'rb').read()).hexdigest()
        if digest != DOWNLOADER_SHA256:
            os.remove(exe + '.part')
            raise SystemExit('downloader hash mismatch: {}'.format(digest))
        os.rename(exe + '.part', exe)
        os.chmod(exe, 0o755)
    cmd = [exe, '--only', 'onnxruntime', 'models', 'dict', '-o', args.output]
    if not args.all:
        cmd += ['--models-pattern', '0.vvm']
    print('running VOICEVOX downloader -> {} (read the terms it shows)'.format(args.output))
    raise SystemExit(subprocess.call(cmd))


if __name__ == '__main__':
    main()
