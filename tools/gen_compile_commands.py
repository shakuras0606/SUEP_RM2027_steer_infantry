#!/usr/bin/env python3
"""为这个 CubeMX Makefile 工程生成 compile_commands.json.

Why: VS Code 的 C/C++ 扩展(以及 clangd)只有在拿到"每个文件真实编译参数"之后,
才能正确解析条件编译、HAL 巨量头文件和宏,跨文件"转到定义"才会准。
本工程是 Makefile 构建,没有 CMake 那种自动导出的编译数据库,所以这里补一个。

Source of truth:
  1. 首选 `make -Bn`(dry-run),这样参数永远和真实构建一致,Makefile 改了也不用改脚本;
  2. 没有 make 时,退回直接解析 Makefile 里的 C_SOURCES / C_DEFS / C_INCLUDES / MCU / OPT。

用法:
    python tools/gen_compile_commands.py            # 生成 compile_commands.json
    python tools/gen_compile_commands.py --make X   # 指定 make 可执行文件
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "compile_commands.json"

# dry-run 里有、但语义分析不需要的开关
DROP_EXACT = {"-MMD", "-MP"}
DROP_PREFIX = ("-MF", "-MT", "-MQ", "-Wa,", "-o")


def run_make(make_cmd: str | None) -> list[str] | None:
    candidates = [make_cmd, os.environ.get("MAKE"), "mingw32-make", "make"]
    for cand in candidates:
        if not cand:
            continue
        exe = shutil.which(cand)
        if not exe:
            continue
        # .cmd/.bat 必须过 cmd.exe,否则 CreateProcess 起不来
        if exe.lower().endswith((".cmd", ".bat")):
            cmd = f'"{exe}" -Bn'
            proc = subprocess.run(cmd, shell=True, cwd=ROOT, capture_output=True, text=True, errors="replace")
        else:
            proc = subprocess.run([exe, "-Bn"], cwd=ROOT, capture_output=True, text=True, errors="replace")
        if proc.returncode == 0 and proc.stdout.strip():
            return proc.stdout.splitlines()
    return None


def parse_make_output(lines: list[str]) -> list[dict]:
    entries: list[dict] = []
    for raw in lines:
        line = raw.strip()
        if " -c " not in line or "gcc" not in line:
            continue
        try:
            tokens = shlex.split(line)
        except ValueError:
            continue
        compiler, rest = tokens[0], tokens[1:]
        kept: list[str] = []
        source = None
        skip_next = False
        for tok in rest:
            if skip_next:
                skip_next = False
                continue
            if tok in ("-o", "-MF", "-MT", "-MQ"):
                skip_next = True
                continue
            if tok.startswith(DROP_PREFIX):
                continue
            if tok in DROP_EXACT:
                continue
            if tok.endswith(".c"):
                source = tok
                break
            kept.append(tok)
        if not source or not (ROOT / source).is_file():
            continue
        entries.append({"directory": str(ROOT), "file": str(ROOT / source), "command": " ".join([compiler, *kept, source])})
    return entries


def expand_vars(variables: dict[str, str], value: str) -> str:
    for _ in range(10):  # $(MCU) -> $(CPU) $(FPU) ... 需要多轮
        new = re.sub(r"\$\(([\w-]+)\)", lambda m: variables.get(m.group(1), ""), value)
        if new == value:
            break
        value = new
    return value.strip()


def read_makefile_variables() -> dict[str, str]:
    text = (ROOT / "Makefile").read_text(encoding="utf-8", errors="replace")
    merged = re.sub(r"\\\s*\n", " ", text)  # 拼掉续行
    variables: dict[str, str] = {}
    for line in merged.splitlines():
        m = re.match(r"^([\w-]+)\s*(\+=|\?=|=)\s*(.*)$", line.strip())
        if not m:
            continue
        name, op, val = m.group(1), m.group(2), m.group(3).strip()
        if op == "+=":
            variables[name] = (variables.get(name, "") + " " + val).strip()
        elif op == "?=":
            variables.setdefault(name, val)
        else:
            variables[name] = val
    return {k: expand_vars(variables, v) for k, v in variables.items()}


def parse_makefile() -> list[dict]:
    v = read_makefile_variables()
    prefix = v.get("PREFIX", "arm-none-eabi-")
    flags = " ".join(
        part
        for part in (
            v.get("MCU") or " ".join(x for x in (v.get("CPU", ""), "-mthumb", v.get("FPU", ""), v.get("FLOAT-ABI", "")) if x),
            v.get("C_DEFS", ""),
            v.get("C_INCLUDES", ""),
            v.get("OPT", ""),
            "-Wall -fdata-sections -ffunction-sections",
            "-g -gdwarf-2" if v.get("DEBUG", "").strip() == "1" else "",
        )
        if part
    )
    entries = []
    for src in shlex.split(v.get("C_SOURCES", "")):
        if not src.endswith(".c") or not (ROOT / src).is_file():
            continue
        entries.append({"directory": str(ROOT), "file": str(ROOT / src), "command": f"{prefix}gcc -c {flags} {src}"})
    return entries


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--make", help="make 可执行文件 (默认自动查找)")
    args = ap.parse_args()

    lines = run_make(args.make)
    if lines:
        entries, origin = parse_make_output(lines), "make -Bn"
    else:
        entries, origin = parse_makefile(), "Makefile 解析"
    if not entries:
        print("[FAIL] no compile command captured", file=sys.stderr)
        return 1

    entries.sort(key=lambda e: e["file"])
    OUT.write_text(json.dumps(entries, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[OK] {len(entries)} entries -> {OUT.relative_to(ROOT)}  (source: {origin})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
