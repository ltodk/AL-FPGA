#!/bin/bash
# 阶段A 加载握手复现 + 修复验证
#   用法: bash run_handshake.sh
set -u
export PATH="/d/FG_software/modelsim/win64:$PATH"
cd "$(dirname "$0")"

SRC="../src/user_source/hdl_source/SD"
PY="C:/Users/lxj/.workbuddy/binaries/python/versions/3.13.12/python.exe"

run_case () {
    local tag="$1"; shift
    local work="work_hs_$tag"
    rm -rf "$work"
    vlib "$work" >/dev/null 2>&1
    vlog -sv -quiet -work "$work" "$@" \
        "$SRC/media_frame_loader.v" "$SRC/media_sdram_write_cdc.v" \
        "$SRC/media_write_commit.v" tb_load_handshake.v >/dev/null 2>&1
    vsim -c -quiet -work "$work" tb_load_handshake -do "run -all; quit -f" > "hs_$tag.log" 2>&1
    echo "=== [$tag] ==="
    "$PY" -c "
import sys
s=open('hs_$tag.log','rb').read().decode('utf-8','replace')
for ln in s.splitlines():
    if any(k in ln for k in ['attempt','FAIL','PASS','RESULT','BUG','模式','Error','error']):
        print(ln[:150])
"
}

run_case old +define+OLD_MODE
run_case new
