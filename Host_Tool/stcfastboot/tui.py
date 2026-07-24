from __future__ import annotations

import traceback
from pathlib import Path
from threading import Event


def run() -> None:
    try:
        from textual import on, work
        from textual.app import App, ComposeResult
        from textual.containers import Horizontal, Vertical, VerticalScroll
        from textual.widgets import (
            Button,
            Checkbox,
            Footer,
            Header,
            Input,
            Label,
            ProgressBar,
            RichLog,
            Static,
        )
    except ImportError as exc:
        raise RuntimeError(
            "缺少 Textual。请在 Python_TUI 目录执行：python -m pip install -r requirements.txt"
        ) from exc

    from .engine import (
        FastBootEngine,
        FlashOptions,
        OperationCancelled,
        build_flash_plan,
        parse_intel_hex,
    )

    class FastBootTUI(App):
        TITLE = "STC32G144K246 FastBoot v1.0.22 Turbo — 2,000,000 baud"
        SUB_TITLE = "Page CRC Incremental · CRC32 · Recovery"
        CSS = """
        Screen {
            layout: vertical;
            padding: 0 1;
        }

        #main-scroll {
            height: 1fr;
        }

        #settings {
            height: auto;
            border: round $accent;
            padding: 1 1;
        }

        .field-row {
            height: 3;
            margin-bottom: 1;
        }

        .field-label {
            width: 14;
            height: 3;
            padding: 1 1 0 0;
        }

        Input {
            width: 1fr;
            height: 3;
        }

        .field-row Button {
            width: 14;
            min-width: 14;
            height: 3;
            margin-left: 1;
        }

        .check-row {
            height: 3;
        }

        #actions {
            height: 3;
            margin-top: 1;
        }

        #actions Button {
            width: 1fr;
            min-width: 15;
            height: 3;
            margin-right: 1;
        }

        #activity {
            height: 3;
            min-height: 3;
            border: round $secondary;
            padding: 0 1;
            margin-top: 1;
        }

        #progress {
            height: 1;
            margin: 1 1 0 1;
        }

        #log-title {
            height: 1;
            text-style: bold;
            margin-top: 1;
        }

        #log {
            height: 12;
            min-height: 8;
            border: round $accent;
            padding: 0 1;
            margin-bottom: 1;
        }
        """
        BINDINGS = [
            ("ctrl+q", "quit", "退出"),
            ("ctrl+x", "cancel", "取消"),
        ]

        _OPERATION_BUTTON_IDS = ("scan", "plan", "status", "crc", "flash")

        def __init__(self) -> None:
            super().__init__()
            self.cancel_event = Event()
            self._debug_path = Path.cwd() / "fastboot_tui_debug.log"

        def compose(self) -> ComposeResult:
            yield Header()
            with VerticalScroll(id="main-scroll"):
                with Vertical(id="settings"):
                    with Horizontal(classes="field-row"):
                        yield Label("串口", classes="field-label")
                        yield Input(value="COM7", id="port")
                        yield Button("扫描", id="scan")
                    with Horizontal(classes="field-row"):
                        yield Label("Intel HEX", classes="field-label")
                        yield Input(placeholder=r"D:\...\SEEKFREE.hex", id="hex")
                        yield Button("检查", id="plan")
                    yield Checkbox(
                        "冷恢复/Bootloader 直连（2000000）",
                        id="cold",
                        classes="check-row",
                    )
                    yield Checkbox(
                        "强制全量重写（默认关闭：自动只更新变化页）",
                        id="full",
                        classes="check-row",
                    )
                    yield Checkbox(
                        "已关闭电机并确认主板与电脑供电稳定",
                        id="safety",
                        classes="check-row",
                    )
                    with Horizontal(id="actions"):
                        yield Button("读取设备状态", id="status")
                        yield Button("校验当前 APP CRC32", id="crc")
                        yield Button("开始 Turbo 增量升级", id="flash", variant="primary")
                        yield Button("取消", id="cancel", variant="error")
                yield Static("就绪：请选择串口和 HEX。", id="activity")
                yield ProgressBar(total=100, id="progress")
                yield Label("运行日志", id="log-title")
                yield RichLog(id="log", highlight=True, markup=True, wrap=True)
            yield Footer()

        def on_mount(self) -> None:
            self.set_busy(False)
            self.log_line("[bold green]TUI 已启动，按钮事件系统就绪。[/bold green]")
            self.log_line("提示：状态、进度和日志均位于按钮下方；窗口较小时可滚动查看。")

        def log_line(self, message: str) -> None:
            self.query_one("#log", RichLog).write(message)

        def set_activity(self, message: str) -> None:
            self.query_one("#activity", Static).update(message)

        def set_port(self, value: str) -> None:
            self.query_one("#port", Input).value = value

        def set_progress(self, phase: str, done: int, total: int) -> None:
            percent = int(done * 100 / max(total, 1))
            self.query_one("#progress", ProgressBar).update(total=100, progress=percent)
            self.set_activity(f"阶段：{phase}    进度：{done}/{total} ({percent}%)")

        def reset_progress(self) -> None:
            self.query_one("#progress", ProgressBar).update(total=100, progress=0)

        def set_busy(self, busy: bool) -> None:
            for button_id in self._OPERATION_BUTTON_IDS:
                self.query_one(f"#{button_id}", Button).disabled = busy
            self.query_one("#cancel", Button).disabled = not busy

        def snapshot_port_mode(self) -> tuple[str, bool]:
            port = self.query_one("#port", Input).value.strip()
            if not port:
                raise ValueError("串口不能为空")
            cold = self.query_one("#cold", Checkbox).value
            return port, cold

        def snapshot_hex(self) -> Path:
            raw = self.query_one("#hex", Input).value.strip().strip('"')
            if not raw:
                raise ValueError("请先填写 Intel HEX 路径")
            return Path(raw)

        def make_engine(
            self, port: str, cold: bool, full_rewrite: bool = False
        ) -> FastBootEngine:
            options = FlashOptions(
                port=port, cold_recovery=cold,
                force_full_rewrite=full_rewrite,
            )
            return FastBootEngine(
                options,
                log=lambda text: self.call_from_thread(self.log_line, text),
                progress=lambda phase, done, total: self.call_from_thread(
                    self.set_progress, phase, done, total
                ),
                cancel_event=self.cancel_event,
            )

        def begin_operation(self, title: str) -> None:
            self.cancel_event.clear()
            self.reset_progress()
            self.set_busy(True)
            self.set_activity(title)
            self.log_line(f"[bold cyan]▶ {title}[/bold cyan]")

        def finish_operation(self, message: str) -> None:
            self.set_busy(False)
            self.set_activity(message)

        def report_worker_error(self, operation: str, exc: Exception) -> None:
            details = traceback.format_exc()
            try:
                self._debug_path.write_text(
                    f"操作：{operation}\n异常：{exc}\n\n{details}",
                    encoding="utf-8",
                )
            except OSError:
                pass

            if isinstance(exc, OperationCancelled):
                self.call_from_thread(
                    self.log_line, f"[bold yellow]{operation}已取消：{exc}[/bold yellow]"
                )
                self.call_from_thread(self.finish_operation, f"已取消：{operation}")
            else:
                self.call_from_thread(
                    self.log_line, f"[bold red]{operation}失败：{exc}[/bold red]"
                )
                self.call_from_thread(
                    self.log_line,
                    f"调试信息已尝试写入：{self._debug_path}",
                )
                self.call_from_thread(self.finish_operation, f"失败：{operation} — {exc}")

        @on(Button.Pressed, "#scan")
        def handle_scan_pressed(self, event: Button.Pressed) -> None:
            event.stop()
            self.begin_operation("正在扫描串口……")
            self.worker_scan_ports()

        @on(Button.Pressed, "#plan")
        def handle_plan_pressed(self, event: Button.Pressed) -> None:
            event.stop()
            try:
                path = self.snapshot_hex()
            except Exception as exc:
                self.log_line(f"[bold red]HEX 检查失败：{exc}[/bold red]")
                self.set_activity(f"HEX 检查失败：{exc}")
                return
            self.begin_operation("正在检查 HEX 升级计划……")
            self.worker_plan_image(path)

        @on(Button.Pressed, "#status")
        def handle_status_pressed(self, event: Button.Pressed) -> None:
            event.stop()
            try:
                port, cold = self.snapshot_port_mode()
            except Exception as exc:
                self.log_line(f"[bold red]读取状态失败：{exc}[/bold red]")
                self.set_activity(f"读取状态失败：{exc}")
                return
            self.begin_operation("正在读取 Bootloader 状态……")
            self.worker_read_status(port, cold)

        @on(Button.Pressed, "#crc")
        def handle_crc_pressed(self, event: Button.Pressed) -> None:
            event.stop()
            try:
                path = self.snapshot_hex()
                port, cold = self.snapshot_port_mode()
            except Exception as exc:
                self.log_line(f"[bold red]CRC32 校验失败：{exc}[/bold red]")
                self.set_activity(f"CRC32 校验失败：{exc}")
                return
            self.begin_operation("正在校验当前 APP CRC32……")
            self.worker_verify_crc(path, port, cold)

        @on(Button.Pressed, "#flash")
        def handle_flash_pressed(self, event: Button.Pressed) -> None:
            event.stop()
            try:
                if not self.query_one("#safety", Checkbox).value:
                    raise RuntimeError("请先勾选供电与电机安全确认")
                path = self.snapshot_hex()
                port, cold = self.snapshot_port_mode()
                full_rewrite = self.query_one("#full", Checkbox).value
            except Exception as exc:
                self.log_line(f"[bold red]无法开始升级：{exc}[/bold red]")
                self.set_activity(f"无法开始升级：{exc}")
                return
            mode = "强制全量重写" if full_rewrite else "Turbo 增量升级"
            self.begin_operation(f"正在执行{mode}……")
            self.worker_flash_image(path, port, cold, full_rewrite)

        @on(Button.Pressed, "#cancel")
        def handle_cancel_pressed(self, event: Button.Pressed) -> None:
            event.stop()
            self.action_cancel()

        def action_cancel(self) -> None:
            self.cancel_event.set()
            self.log_line("[yellow]已请求取消；将在当前协议事务结束后停止。[/yellow]")
            self.set_activity("正在请求取消……")

        @work(
            thread=True,
            exclusive=True,
            group="ports",
            exit_on_error=False,
        )
        def worker_scan_ports(self) -> None:
            operation = "串口扫描"
            try:
                from serial.tools import list_ports

                ports = list(list_ports.comports())
                if not ports:
                    self.call_from_thread(self.log_line, "未发现串口")
                    self.call_from_thread(self.finish_operation, "扫描完成：未发现串口")
                    return
                for item in ports:
                    self.call_from_thread(
                        self.log_line, f"{item.device}  {item.description}"
                    )
                self.call_from_thread(self.set_port, ports[0].device)
                self.call_from_thread(
                    self.finish_operation,
                    f"扫描完成：发现 {len(ports)} 个串口，已选择 {ports[0].device}",
                )
            except Exception as exc:
                self.report_worker_error(operation, exc)

        @work(
            thread=True,
            exclusive=True,
            group="operation",
            exit_on_error=False,
        )
        def worker_plan_image(self, path: Path) -> None:
            operation = "HEX 检查"
            try:
                image = parse_intel_hex(path)
                plan = build_flash_plan(image)
                self.call_from_thread(
                    self.log_line,
                    f"HEX：0x{image.lowest:06X}-0x{image.highest:06X}，"
                    f"CRC32=0x{image.crc32_full_app:08X}",
                )
                self.call_from_thread(
                    self.log_line,
                    f"全量最坏计划：擦除 {len(plan.erase_pages)} 页，"
                    f"写入 {len(plan.program_packets)} 包 / "
                    f"串口载荷 {plan.programmed_bytes} 字节；实际升级先比较页面 CRC",
                )
                self.call_from_thread(
                    self.finish_operation,
                    f"HEX 检查通过：CRC32=0x{image.crc32_full_app:08X} · "
                    "默认仅更新变化页",
                )
            except Exception as exc:
                self.report_worker_error(operation, exc)

        @work(
            thread=True,
            exclusive=True,
            group="operation",
            exit_on_error=False,
        )
        def worker_read_status(self, port: str, cold: bool) -> None:
            operation = "读取设备状态"
            try:
                version, status = self.make_engine(port, cold).inspect_device()
                self.call_from_thread(
                    self.log_line,
                    f"Bootloader=0x{version:04X}，状态={status.state_name}，"
                    f"size={status.image_size}，CRC32=0x{status.expected_crc32:08X}",
                )
                self.call_from_thread(
                    self.finish_operation,
                    f"Bootloader 0x{version:04X} · 状态 {status.state_name} · "
                    f"目标 CRC32 0x{status.expected_crc32:08X}",
                )
            except Exception as exc:
                self.report_worker_error(operation, exc)

        @work(
            thread=True,
            exclusive=True,
            group="operation",
            exit_on_error=False,
        )
        def worker_verify_crc(self, path: Path, port: str, cold: bool) -> None:
            operation = "APP CRC32 校验"
            try:
                image = parse_intel_hex(path)
                version, status, actual = self.make_engine(
                    port, cold
                ).verify_current_application(image)
                self.call_from_thread(
                    self.log_line,
                    f"[bold green]CRC32 PASS：device=0x{actual:08X}，"
                    f"host=0x{image.crc32_full_app:08X}[/bold green]",
                )
                self.call_from_thread(
                    self.finish_operation,
                    f"CRC32 一致 · Bootloader 0x{version:04X} · "
                    f"状态 {status.state_name} · 0x{actual:08X}",
                )
            except Exception as exc:
                self.report_worker_error(operation, exc)

        @work(
            thread=True,
            exclusive=True,
            group="operation",
            exit_on_error=False,
        )
        def worker_flash_image(
            self, path: Path, port: str, cold: bool, full_rewrite: bool
        ) -> None:
            operation = "强制全量重写" if full_rewrite else "Turbo 增量升级"
            try:
                image = parse_intel_hex(path)
                plan = build_flash_plan(image)
                self.call_from_thread(
                    self.log_line,
                    f"目标 CRC32=0x{image.crc32_full_app:08X}；"
                    f"全量上限 {len(plan.erase_pages)} 页/{len(plan.program_packets)} 包；"
                    f"模式={'强制全量' if full_rewrite else '页面 CRC 增量'}",
                )
                actual = self.make_engine(port, cold, full_rewrite).flash(image, plan)
                self.call_from_thread(
                    self.log_line,
                    f"[bold green]升级完成：设备 CRC32=0x{actual:08X}，"
                    "状态 COMMITTED[/bold green]",
                )
                self.call_from_thread(
                    self.finish_operation,
                    f"升级成功 · 设备 CRC32 0x{actual:08X} · 状态 COMMITTED",
                )
            except Exception as exc:
                self.report_worker_error(operation, exc)

    FastBootTUI().run()
