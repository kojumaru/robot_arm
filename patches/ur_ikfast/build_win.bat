@echo off
REM ur_ikfast を Windows + scipy-openblas32 でビルドするスクリプト。
REM C:\tmp\ur_ikfast\ 直下に setup.py と ur3e\lapack_stubs.cpp を
REM patches/ur_ikfast/ からコピーした上でこのバッチを実行する。

call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
set DISTUTILS_USE_SDK=1
set MSSdk=1
cd /d C:\tmp\ur_ikfast
C:\Users\kojum\robot_arm\.venv_win\Scripts\pip.exe install --no-build-isolation --force-reinstall --no-deps .
