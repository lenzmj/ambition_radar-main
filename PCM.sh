#!/usr/bin/env bash
# 反制前哨：延迟启动 ./build/app

set -euo pipefail
# -e: 命令失败即退出；-u: 未定义变量报错；-o pipefail: 管道中任一命令失败则整条失败

cd "$(dirname "$0")"
# 切换到脚本所在目录，保证 ./build/app 路径正确

echo "反制前哨于4:00,等待 3 分  秒后启动,预计4:10~3:50反制"
sleep 215
# 等待 3 分 35 秒（215 秒）

exec ./build/app
# 用 app 替换当前 shell 进程；
