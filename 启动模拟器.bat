@echo off
chcp 65001 >nul
where python >nul 2>nul || (echo 未找到 Python，请先安装 Python 3.10 或更高版本。& pause&exit /b 1)
python -c "import serial" >nul 2>nul || (echo 正在安装串口依赖 pyserial...& python -m pip install -r requirements.txt)
python main.py
pause
