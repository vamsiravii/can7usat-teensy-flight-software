#!/usr/bin/env python3
"""Validate documentation, build entries and repository hygiene."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
POLLING = ROOT / "projects/ASR6601CB-EVAL/examples/lora/lora_myself_net"

PAIRS = [
    (ROOT / "readme.md", ROOT / "readme.zh.md", "中文-README-blue", "English-README-green"),
    (ROOT / "docs/CODE_ENTRY.md", ROOT / "docs/CODE_ENTRY.zh.md", "中文-文档-blue", "English-Docs-green"),
    (ROOT / "docs/ARCHITECTURE.md", ROOT / "docs/ARCHITECTURE.zh.md", "中文-文档-blue", "English-Docs-green"),
    (ROOT / "docs/VALIDATION.md", ROOT / "docs/VALIDATION.zh.md", "中文-文档-blue", "English-Docs-green"),
    (POLLING / "README.md", POLLING / "README.zh.md", "中文-文档-blue", "English-Docs-green"),
]

PRODUCT_APPS = [
    "projects/ASR6601CB-EVAL/examples/lora/lora_factory_test",
    "projects/ASR6601CB-EVAL/examples/lora/lora_myself_net",
    "projects/ASR6601CB-EVAL/examples/lora/lora_test",
    "projects/ASR6601CB-EVAL/examples/lora/pingpong",
    "projects/ASR6601CB-EVAL/examples/lora/Ra-08-kit-LoraTest",
    "projects/ASR6601CB-EVAL/examples/lorawan/class_a",
    "projects/ASR6601CB-EVAL/examples/lorawan/class_c",
    "projects/ASR6601CB-EVAL/examples/lorawan/lorawan_at",
]


def main() -> int:
    errors: list[str] = []

    for english, chinese, english_badge, chinese_badge in PAIRS:
        for path, badge in ((english, english_badge), (chinese, chinese_badge)):
            if not path.is_file():
                errors.append(f"missing documentation pair: {path.relative_to(ROOT)}")
                continue
            lines = path.read_text(encoding="utf-8").splitlines()
            if not lines or badge not in lines[0]:
                errors.append(f"incorrect language badge: {path.relative_to(ROOT)}")

    required = [
        ROOT / "build/envsetup.sh",
        ROOT / "build/make/common.mk",
        ROOT / "platform/system/startup_cm4.S",
        ROOT / "platform/system/printf-stdarg.c",
        ROOT / "tools/validation/build_product_examples.sh",
        ROOT / ".gitmodules",
    ]
    for app in PRODUCT_APPS:
        required.extend((ROOT / app / "Makefile", ROOT / app / "src/main.c"))
    for path in required:
        if not path.is_file():
            errors.append(f"missing required file: {path.relative_to(ROOT)}")

    gitmodules = (ROOT / ".gitmodules").read_text(encoding="utf-8")
    if "path = tools/toolchain" not in gitmodules or "https://e.coding.net/" not in gitmodules:
        errors.append("toolchain submodule declaration is missing or unexpected")

    if "#include <stdint-gcc.h>" in (ROOT / "platform/system/printf-stdarg.c").read_text(encoding="utf-8"):
        errors.append("obsolete stdint-gcc.h include is still present")

    markdown_link = re.compile(r"(?<!!)\[[^]]+\]\(([^)]+)\)")
    for english, chinese, *_ in PAIRS:
        for path in (english, chinese):
            if not path.is_file():
                continue
            for target in markdown_link.findall(path.read_text(encoding="utf-8")):
                target = target.split("#", 1)[0]
                if not target or "://" in target or target.startswith("mailto:"):
                    continue
                if not (path.parent / target).resolve().exists():
                    errors.append(f"broken local link in {path.relative_to(ROOT)}: {target}")

    tracked = subprocess.run(
        ["git", "ls-files", "-z"], cwd=ROOT, check=True, capture_output=True
    ).stdout.decode("utf-8").split("\0")
    for path in tracked:
        if "Zone.Identifier" in path:
            errors.append(f"Windows alternate-stream artifact: {path}")

    if errors:
        print("Repository validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print(
        f"Repository validation passed: {len(PAIRS)} bilingual pairs and "
        f"{len(PRODUCT_APPS)} product build entries checked."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
