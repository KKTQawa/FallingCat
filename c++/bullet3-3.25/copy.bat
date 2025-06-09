@echo off
setlocal enabledelayedexpansion

:: 设置源目录和目标目录
set "SOURCE_DIR=E:\third_party\bullet3-3.25\src"
set "TARGET_DIR=E:\third_party\bullet3-3.25\include"

:: 创建目标目录（如果不存在）
if not exist "%TARGET_DIR%" mkdir "%TARGET_DIR%"

:: 遍历src目录下的所有.h文件
for /r "%SOURCE_DIR%" %%f in (*.h) do (
    :: 获取文件的相对路径
    set "filepath=%%f"
    set "relpath=!filepath:%SOURCE_DIR%\=!"
    
    :: 创建目标子目录结构
    set "targetpath=%TARGET_DIR%\!relpath!"
    set "targetdir=%%~dpf"
    set "targetdir=%TARGET_DIR%\!targetdir:%SOURCE_DIR%\=!"
    
    if not exist "!targetdir!" (
        mkdir "!targetdir!"
    )
    
    :: 复制文件并显示进度
    echo 正在复制: !relpath!
    copy "%%f" "!targetpath!" >nul
)

echo 所有头文件已复制完成，保持原始目录结构
pause