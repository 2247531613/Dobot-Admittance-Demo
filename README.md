# Dobot 导纳拖动 Demo

一个使用 Dobot TCP/IP 接口实现六维导纳拖动的最小 Python 工程。

## 目录

```text
Hardware/
  dobot_api.py              # Dobot TCP/IP API
  feedback_monitor.py       # 30004 实时反馈线程
AdmittanceDrag/
  AdmittanceController/
    admittance_controller.py
  Filters/
    Filters.py
  Demo/
    AdmittanceForceDrive.py
Dobot_TCPIP_Quick_Start.md
```

## 环境

- Python 3.10 或更高版本
- NumPy
- Requests

```powershell
py -m pip install -r requirements.txt
```

## 运行

先在 `AdmittanceDrag/Demo/AdmittanceForceDrive.py` 中确认机器人 IP 和导纳参数，然后运行：

```powershell
py .\AdmittanceDrag\Demo\AdmittanceForceDrive.py
```

程序连接：

- `29999`：控制命令和 `ServoP`
- `30004`：机器人状态与六维力实时反馈

## 安全说明

本工程会使能并驱动真实机器人。首次运行前必须确认急停、工具和负载配置、力反馈方向、工作空间以及现场人员安全。建议先按《Dobot二次开发快速指南》完成只读反馈验证，再进行低速导纳拖动测试。

