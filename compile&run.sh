#!/bin/bash

echo "Compiling dexterous hand program..."

# 创建 build 目录（如果不存在）
mkdir -p build

# 编译
gcc -c dexterous_hand.c -o build/dexterous_hand.o
gcc -c main.c -o build/main.o
gcc build/dexterous_hand.o build/main.o -o build/dexterous_hand -lm

# 检查编译是否成功
if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi

echo "Compilation successful!"
echo "Running program..."
./build/dexterous_hand