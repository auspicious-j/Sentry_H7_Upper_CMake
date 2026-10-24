#!/usr/bin/env bash
#
# [LINUX-PORT] 一键编译脚本
#
# 本机没有通过 apt 安装 cmake（无 sudo 权限），而是使用免安装版本：
#   /home/speros987/.local/tools/cmake-3.31.6-linux-x86_64/bin/cmake
# 如果系统里已经装了 cmake，脚本会自动优先使用系统的。
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CMAKE_BIN="${CMAKE_BIN:-}"
if [[ -z "$CMAKE_BIN" ]]; then
    if command -v cmake >/dev/null 2>&1; then
        CMAKE_BIN="$(command -v cmake)"
    else
        CMAKE_BIN="$HOME/.local/tools/cmake-3.31.6-linux-x86_64/bin/cmake"
    fi
fi

if [[ ! -x "$CMAKE_BIN" ]]; then
    echo "找不到可用的 cmake，请先安装或设置 CMAKE_BIN 环境变量" >&2
    exit 1
fi

BUILD_TYPE="${1:-Debug}"
BUILD_DIR="build/${BUILD_TYPE}"

echo "==> cmake: $CMAKE_BIN"
echo "==> 构建目录: $BUILD_DIR (${BUILD_TYPE})"

"$CMAKE_BIN" -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
"$CMAKE_BIN" --build "$BUILD_DIR" -j"$(nproc)"

echo
echo "==> 编译完成: $BUILD_DIR/sentry_upper_linux"
echo "==> 运行: ./$BUILD_DIR/sentry_upper_linux"
