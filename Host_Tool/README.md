# FastBoot v1.0.22 主机工具

适用于 STC32G144K246 FastBoot 协议 `0x0206`，APP 与 Bootloader 均使用名义波特率 `2,000,000`。

## 安装与启动

在本目录打开 PowerShell：

```powershell
python -m pip install -r requirements.txt
python -m stcfastboot tui
```

也可以双击 `install_and_run_tui.bat` 首次安装并启动，之后双击 `run_tui.bat`。

## 常用命令

```powershell
python -m stcfastboot ports
python -m stcfastboot status --port COM7
python -m stcfastboot crc "D:\...\SEEKFREE.hex" --port COM7
python -m stcfastboot flash "D:\...\SEEKFREE.hex" --port COM7 --yes
```

强制全量重写：

```powershell
python -m stcfastboot flash "D:\...\SEEKFREE.hex" --port COM7 --yes --full-rewrite
```

默认使用页面 CRC32 增量升级，只擦除和写入变化页。升级过程中断后，重新使用同一 HEX 即可按页续传。冷恢复或 APP 无法启动时，拉低 P3.3 上电，并在 TUI 勾选冷恢复，或在 CLI 添加 `--cold-recovery`。

升级期间必须保持供电稳定，并关闭 STC-ISP、串口助手、VOFA+ 等占用串口的程序。
