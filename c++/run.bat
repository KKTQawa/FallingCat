@echo off

call src\mycat\Scripts\activate

echo 成功激活运行环境

echo 如运行错误，请保证python解释器已添加至环境变量！

echo 赶工作品，时间紧迫，有很多地方没有完善~~~

echo ------------------------------------《落体猫运动》--------------------------------------

echo 请查看说明文档进行按键操作！

echo ...
echo ...
echo ...

cd bullet3-3.25\execute

App_RobotSimulator_vs2010_x64_release.exe

pause

cd ../../
