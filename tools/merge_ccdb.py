#!/usr/bin/env python3
"""合并 .pio/ccdbs/*.json 为根目录 compile_commands.json (clangd 使用)。

用法: python3 tools/merge_ccdb.py
前置: 先按各环境执行 `pio run -t compiledb` 并将产物存为 .pio/ccdbs/<env>.json
      (见仓库根 README.md 的"编译数据库"章节)。

补偿 PIO `-t compiledb` 的已知缺陷:
  header-only 库 (如 lib/SemanticEnums, 无 .cpp) 不会被注入 -I:
  真实构建命令含 -Ilib/SemanticEnums, 但 ccdb 缺失, 导致 clangd 解析
  Kinematics.h 时报 'SemanticEnums.h' file not found. 此处对缺失条目统一补齐。
"""

import glob
import json
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLCHAIN = os.path.expanduser("~/.platformio/packages/toolchain-xtensa-esp32s3/bin")
# header-only 库: compiledb 漏注入的 include, 相对仓库根, 随条目的 directory 解析
HEADER_ONLY_LIBS = ("lib/RobotConfig", "lib/SemanticEnums")


def main() -> None:
    ccdbs = sorted(glob.glob(os.path.join(REPO, ".pio/ccdbs/*.json")))
    seen: set[str] = set()
    out: list[dict] = []
    for f in ccdbs:
        with open(f, encoding="utf-8") as fh:
            entries = json.load(fh)
        for e in entries:
            cmd = e["command"]
            # 相对编译器名补绝对路径前缀; 不用 sed 全局替换 (名字也存在于绝对路径内)
            if cmd.startswith("xtensa-esp32s3-elf-"):
                e["command"] = f"{TOOLCHAIN}/{cmd}"
            # 兜底: header-only 库缺失的 -I, 避免 clangd 误报
            for inc in HEADER_ONLY_LIBS:
                if f"-I{inc}" not in e["command"] and f"-I {inc}" not in e["command"]:
                    e["command"] += f" -I{inc}"
            # 公共 framework 源文件在各环境重复, 按 file 去重
            if e["file"] not in seen:
                seen.add(e["file"])
                out.append(e)
    with open(os.path.join(REPO, "compile_commands.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print(f"merged {len(out)} entries from {len(ccdbs)} envs")


if __name__ == "__main__":
    main()
