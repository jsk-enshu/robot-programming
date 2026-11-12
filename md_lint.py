#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["cmarkgfm"]
# ///

from __future__ import annotations

import argparse
import re
import sys
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cmarkgfm


FENCE_RE = re.compile(r"^(?P<indent> {0,3})(?P<fence>`{3,}|~{3,})(?P<info>.*)$")
HEADING_RE = re.compile(r"^ {0,3}#{1,6}\s")
LIST_ITEM_RE = re.compile(r"^(?P<indent> *)(?P<marker>[-*+]|\d+[.)])\s+\S")
FULLWIDTH_FENCE_RE = re.compile(r"^\s*｀｀｀+")
CJK_RE = re.compile(r"[぀-ヿ一-鿿]")
OUTSIDE_EMPH_RE = re.compile(
    r"(?P<open>\*\*|__)(?P<lb>[「『（【《])(?P<body>[^*_\n]+?)(?P<rb>[」』）】》])(?P=open)"
)
CODE_SPAN_RE = re.compile(r"(?P<ticks>`+)(?P<body>.+?)(?P=ticks)", re.DOTALL)
DELIM_RUN_RE = re.compile(r"\*+|_+")
OPENERS = "「『（【《［〈［(["
EXCLUDED_DIRS = ("_build", ".venv", ".git", "node_modules")


@dataclass
class Issue:
    line: int
    code: str
    message: str
    fixable: bool = False
    col: int = 0
    warning: bool = False


@dataclass
class Block:
    start: int
    end: int
    char: str
    length: int
    info: str


def scan_fences(lines: Sequence[str]) -> Tuple[List[Block], List[int]]:
    blocks: List[Block] = []
    fence_lines: List[int] = []
    open_block: Optional[Block] = None

    for i, line in enumerate(lines):
        m = FENCE_RE.match(line)
        if m is None:
            continue
        fence = m.group("fence")
        char, length = fence[0], len(fence)
        info = m.group("info").strip()

        if open_block is None:
            if info and (char in info):
                continue
            open_block = Block(start=i, end=-1, char=char, length=length, info=info)
            fence_lines.append(i)
        elif char == open_block.char and length >= open_block.length and not info:
            open_block.end = i
            blocks.append(open_block)
            fence_lines.append(i)
            open_block = None

    if open_block is not None:
        blocks.append(open_block)
        fence_lines.append(open_block.start)
    return blocks, fence_lines


def in_code(index: int, blocks: Sequence[Block], total: int) -> bool:
    for b in blocks:
        end = b.end if b.end >= 0 else total - 1
        if b.start <= index <= end:
            return True
    return False


def paragraphs(lines: Sequence[str], blocks: Sequence[Block]) -> List[Tuple[int, int]]:
    spans: List[Tuple[int, int]] = []
    start = None
    skip_to = 0
    if lines and lines[0].strip() == "---":
        for j in range(1, len(lines)):
            if lines[j].strip() in ("---", "..."):
                skip_to = j + 1
                break
    for i in range(skip_to, len(lines)):
        blank = not lines[i].strip()
        if in_code(i, blocks, len(lines)) or blank:
            if start is not None:
                spans.append((start, i - 1))
                start = None
            continue
        if start is None:
            start = i
    if start is not None:
        spans.append((start, len(lines) - 1))
    return spans


def emphasis_breaks(text: str) -> bool:
    html = cmarkgfm.github_flavored_markdown_to_html(text)
    html = re.sub(r"<a [^>]*>(?:https?://|www\.)[^<]*</a>", "", html)
    text_only = re.sub(r"<[^>]*>", "", html)
    return "**" in text_only or "__" in text_only


def check_fences(lines: Sequence[str], blocks: Sequence[Block]) -> List[Issue]:
    issues: List[Issue] = []
    for b in blocks:
        if b.end < 0:
            issues.append(
                Issue(
                    b.start + 1,
                    "FENCE-UNCLOSED",
                    "フェンスが閉じられないままファイルが終わっています。"
                    "以降の本文がすべてコードブロックに飲み込まれます。",
                )
            )
            body = lines[b.start + 1 :]
        else:
            body = lines[b.start + 1 : b.end]

        headings = [j for j, ln in enumerate(body) if HEADING_RE.match(ln)]
        if headings and not b.info:
            issues.append(
                Issue(
                    b.start + 1 + headings[0] + 1,
                    "FENCE-SWALLOW",
                    "コードブロックの中に見出し行があります。内側のフェンスが外側を"
                    "早く閉じ、本文が飲み込まれた可能性が高い。外側を ```` (4本以上) "
                    "にしてください。",
                )
            )
    return issues


def check_list_fences(lines: Sequence[str], blocks: Sequence[Block]) -> List[Issue]:
    issues = []
    starts = {b.start for b in blocks}
    for i, line in enumerate(lines):
        if i == 0 or i not in starts:
            continue
        prev = lines[i - 1]
        if LIST_ITEM_RE.match(prev) and not line.startswith((" ", "\t")):
            issues.append(
                Issue(
                    i + 1,
                    "FENCE-IN-LIST",
                    "箇条書きの直後にインデントなしのフェンスがあり、リストが分断され"
                    "ます。フェンスをリスト記号の幅だけ字下げしてください。",
                    fixable=True,
                )
            )
    return issues


def check_fullwidth(lines: Sequence[str]) -> List[Issue]:
    return [
        Issue(i + 1, "FENCE-FULLWIDTH", "全角バッククォート (｀) はフェンスになりません。", True)
        for i, line in enumerate(lines)
        if FULLWIDTH_FENCE_RE.match(line)
    ]


def mask_code_spans(text: str) -> str:
    return CODE_SPAN_RE.sub(
        lambda m: m.group("ticks")
        + re.sub(r"[^\n]", "x", m.group("body"))
        + m.group("ticks"),
        text,
    )


def _is_punct(ch: str) -> bool:
    return bool(ch) and unicodedata.category(ch)[0] in ("P", "S")


def dead_delimiters(chunk: str) -> List[int]:
    dead: List[int] = []
    stranded: List[int] = []
    for m in DELIM_RUN_RE.finditer(chunk):
        before = chunk[m.start() - 1] if m.start() else ""
        after = chunk[m.end()] if m.end() < len(chunk) else ""
        before_ws = (not before) or before.isspace()
        after_ws = (not after) or after.isspace()
        left = (not after_ws) and (not _is_punct(after) or before_ws or _is_punct(before))
        right = (not before_ws) and (not _is_punct(before) or after_ws or _is_punct(after))
        if not left and not right:
            dead.append(m.start())
        elif not left and after in OPENERS and not before_ws:
            stranded.append(m.start())
    return dead or stranded


def check_emphasis(lines: Sequence[str], blocks: Sequence[Block]) -> List[Issue]:
    issues = []
    for start, end in paragraphs(lines, blocks):
        probe = mask_code_spans("\n".join(ln.lstrip() for ln in lines[start : end + 1]))
        if ("**" not in probe and "__" not in probe) or not emphasis_breaks(probe):
            continue
        offsets = dead_delimiters(probe)
        anchor = start
        col = 0
        if offsets:
            anchor = start + probe.count("\n", 0, offsets[0])
            line_start = probe.rfind("\n", 0, offsets[0]) + 1
            col = offsets[0] - line_start + 1
        else:
            for j in range(start, end + 1):
                if "**" in lines[j] or "__" in lines[j]:
                    anchor = j
                    break
        issues.append(
            Issue(
                anchor + 1,
                "EMPH-DEAD",
                "この位置の ** が強調として機能していません "
                "(直前が文字で直後が括弧、直前が句読点で直後が文字、など)。括弧の内側に寄せてください: "
                "これは**「A」**です → これは「**A**」です。",
                fixable=bool(OUTSIDE_EMPH_RE.search(probe)),
                col=col,
            )
        )
    return issues


def check_indented_prose(lines: Sequence[str], blocks: Sequence[Block]) -> List[Issue]:
    issues = []
    for i, line in enumerate(lines):
        if in_code(i, blocks, len(lines)) or not line.startswith("    "):
            continue
        if i > 0 and lines[i - 1].strip():
            continue
        prev = next((lines[j] for j in range(i - 1, -1, -1) if lines[j].strip()), "")
        if LIST_ITEM_RE.match(prev) or LIST_ITEM_RE.match(line):
            continue
        body = line.strip()
        if CJK_RE.search(body) and not re.search(r"[{};=()<>]|^\w+\s+\w+\s*=", body):
            issues.append(
                Issue(
                    i + 1,
                    "INDENT-CODE",
                    "4スペース字下げされた日本語がコードブロック扱いになっています。",
                    warning=True,
                )
            )
    return issues


def analyse(text: str) -> List[Issue]:
    lines = text.split("\n")
    blocks, _ = scan_fences(lines)
    issues = (
        check_fences(lines, blocks)
        + check_list_fences(lines, blocks)
        + check_fullwidth(lines)
        + check_emphasis(lines, blocks)
        + check_indented_prose(lines, blocks)
    )
    return sorted(issues, key=lambda x: (x.line, x.code))


def fix_text(text: str) -> Tuple[str, int]:
    lines = text.split("\n")
    count = 0

    for i, line in enumerate(lines):
        if FULLWIDTH_FENCE_RE.match(line):
            lines[i] = line.replace("｀", "`")
            count += 1

    blocks, _ = scan_fences(lines)
    for start, end in paragraphs(lines, blocks):
        chunk = "\n".join(lines[start : end + 1])
        probe = mask_code_spans("\n".join(ln.lstrip() for ln in lines[start : end + 1]))
        if not OUTSIDE_EMPH_RE.search(probe) or not emphasis_breaks(probe):
            continue

        def rewrite(m: "re.Match") -> str:
            before = chunk[m.start() - 1] if m.start() else ""
            after = chunk[m.end()] if m.end() < len(chunk) else ""
            if not emphasis_breaks(before + m.group(0) + after):
                return m.group(0)
            return "{lb}{o}{body}{o}{rb}".format(
                lb=m.group("lb"), rb=m.group("rb"),
                o=m.group("open"), body=m.group("body"),
            )

        candidate = OUTSIDE_EMPH_RE.sub(rewrite, chunk)
        verify = mask_code_spans("\n".join(ln.lstrip() for ln in candidate.split("\n")))
        if candidate != chunk and not emphasis_breaks(verify):
            new_lines = candidate.split("\n")
            if len(new_lines) == end - start + 1:
                lines[start : end + 1] = new_lines
                count += 1

    blocks, _ = scan_fences(lines)
    for b in reversed(blocks):
        if b.start == 0 or b.end < 0:
            continue
        prev = LIST_ITEM_RE.match(lines[b.start - 1])
        if prev is None or lines[b.start].startswith((" ", "\t")):
            continue
        pad = " " * (len(prev.group("indent")) + len(prev.group("marker")) + 1)
        for j in range(b.start, b.end + 1):
            if lines[j].strip():
                lines[j] = pad + lines[j]
        count += 1

    blocks, _ = scan_fences(lines)
    out: List[str] = []
    starts = {b.start for b in blocks}
    ends = {b.end for b in blocks if b.end >= 0}
    for i, line in enumerate(lines):
        if i in starts and out and out[-1].strip() and not LIST_ITEM_RE.match(out[-1]):
            out.append("")
            count += 1
        out.append(line)
        if i in ends and i + 1 < len(lines) and lines[i + 1].strip():
            if not LIST_ITEM_RE.match(lines[i + 1]):
                out.append("")
                count += 1
    return "\n".join(out), count


def collect_files(paths: Sequence[Path]) -> List[Path]:
    files: List[Path] = []
    for p in paths:
        if p.is_dir():
            files.extend(
                f for f in sorted(p.rglob("*.md"))
                if not any(part in EXCLUDED_DIRS for part in f.relative_to(p).parts)
            )
        else:
            files.append(p)
    return files


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Markdown lint for Japanese documents")
    parser.add_argument("mode", choices=["check", "fix"])
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--format", choices=["text", "github"], default="text")
    args = parser.parse_args(argv)

    files = collect_files(args.paths)

    total = 0
    warnings = 0
    for path in files:
        text = path.read_text(encoding="utf-8")
        if args.mode == "fix":
            new, n = fix_text(text)
            if n:
                path.write_text(new, encoding="utf-8")
                print("fixed {} ({} 箇所)".format(path, n))
            remaining = [i for i in analyse(new) if not i.fixable]
            for issue in remaining:
                print("  {} {}:{}{}: [{}] {}".format(
                    "警告" if issue.warning else "要手動",
                    path, issue.line,
                    ":{}".format(issue.col) if issue.col else "",
                    issue.code, issue.message))
            total += sum(1 for i in remaining if not i.warning)
        else:
            for issue in analyse(text):
                if args.format == "github":
                    print("::{} file={},line={},col={}::[{}] {}".format(
                        "warning" if issue.warning else "error",
                        path, issue.line, issue.col or 1, issue.code, issue.message))
                else:
                    where = "{}:{}".format(path, issue.line)
                    if issue.col:
                        where += ":{}".format(issue.col)
                    print("{}: [{}] {}{}{}".format(
                        where, issue.code, issue.message,
                        " (自動修正可)" if issue.fixable else "",
                        " (警告)" if issue.warning else ""))
                if issue.warning:
                    warnings += 1
                else:
                    total += 1

    if args.mode == "check":
        print("\n{} ファイル中 エラー {} 件 / 警告 {} 件".format(len(files), total, warnings),
              file=sys.stderr)
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
