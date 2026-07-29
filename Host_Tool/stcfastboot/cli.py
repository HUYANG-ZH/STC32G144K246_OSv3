from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .engine import (
    APP_SIZE,
    FastBootEngine,
    FlashOptions,
    build_flash_plan,
    parse_intel_hex,
)


def add_common_serial(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="串口，例如 COM7")
    parser.add_argument("--cold-recovery", action="store_true", help="跳过 APP，1000000 直连 Bootloader")
    parser.add_argument("--boot-baud", type=int, default=None)


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="stcfastboot", description="STC32G144K246 FastBoot v1.0.22 Turbo")
    sub = parser.add_subparsers(dest="command", required=True)

    plan = sub.add_parser("plan", help="检查 HEX 并生成升级计划")
    plan.add_argument("hex_file", type=Path)

    flash = sub.add_parser("flash", help="页面 CRC 增量升级、设备整区 CRC32 校验并提交")
    flash.add_argument("hex_file", type=Path)
    add_common_serial(flash)
    flash.add_argument("--yes", action="store_true", help="确认执行升级")
    flash.add_argument("--full-rewrite", action="store_true", help="强制擦除并重写全部 476 页")

    status = sub.add_parser("status", help="读取 Bootloader 版本和持久升级状态")
    add_common_serial(status)

    crc = sub.add_parser("crc", help="只读计算设备完整 APP CRC32，并与 HEX 比较")
    crc.add_argument("hex_file", type=Path)
    add_common_serial(crc)

    sub.add_parser("ports", help="列出串口")
    sub.add_parser("tui", help="启动正式 Textual TUI")
    return parser


def print_plan(path: Path) -> tuple[object, object]:
    image = parse_intel_hex(path)
    plan = build_flash_plan(image)
    print(f"HEX：{path}")
    print(f"地址范围：0x{image.lowest:06X}-0x{image.highest:06X}")
    print(f"完整 APP：{APP_SIZE} 字节，CRC32=0x{image.crc32_full_app:08X}")
    print(f"非 FF 数据：{image.non_ff_count} 字节")
    print(f"全量最坏擦除：{len(plan.erase_pages)} 页")
    print(
        f"全量最坏写入：{len(plan.program_packets)} 包 / "
        f"串口载荷 {plan.programmed_bytes} 字节 / "
        f"实际非 FF IAP {plan.iap_program_bytes} 字节"
    )
    print("实际升级会先比较页面 CRC32，只处理变化页。")
    print("最终提交：0xFF0000-0xFF0002")
    return image, plan


def main(argv: list[str] | None = None) -> int:
    args = make_parser().parse_args(argv)
    if args.command == "tui":
        from .tui import run
        run()
        return 0
    if args.command == "ports":
        try:
            from serial.tools import list_ports
        except ImportError:
            print("缺少 pyserial", file=sys.stderr)
            return 2
        for port in list_ports.comports():
            print(f"{port.device}\t{port.description}")
        return 0
    if args.command == "plan":
        try:
            print_plan(args.hex_file)
            return 0
        except Exception as exc:
            print(f"FAIL：{exc}", file=sys.stderr)
            return 2

    options = FlashOptions(
        port=args.port,
        cold_recovery=args.cold_recovery,
        boot_baud=args.boot_baud,
        force_full_rewrite=getattr(args, "full_rewrite", False),
    )
    engine = FastBootEngine(options, log=print)
    mode = "冷恢复/Bootloader 直连" if options.cold_recovery else "APP 热切换"
    app_text = "/".join(str(value) for value in options.app_baud_candidates())
    print(
        f"通信：{mode}，APP入口={app_text}，"
        f"Bootloader={options.resolved_boot_baud()}"
    )

    if args.command == "status":
        try:
            version, status = engine.inspect_device()
            print(f"Bootloader：0x{version:04X}")
            print(f"状态：{status.state_name}")
            print(f"目标 CRC32：0x{status.expected_crc32:08X}")
            return 0
        except Exception as exc:
            print(f"FAIL：{exc}", file=sys.stderr)
            return 1

    if args.command == "crc":
        try:
            image, _plan = print_plan(args.hex_file)
            version, status, actual = engine.verify_current_application(image)
            print(f"Bootloader：0x{version:04X}")
            print(f"状态：{status.state_name}")
            print(f"设备 CRC32：0x{actual:08X}")
            print(f"主机 CRC32：0x{image.crc32_full_app:08X}")
            print("PASS：设备完整 APP CRC32 与 HEX 一致。")
            return 0
        except Exception as exc:
            print(f"FAIL：{exc}", file=sys.stderr)
            return 1

    try:
        image, plan = print_plan(args.hex_file)
        if not args.yes:
            print("DRY-RUN：添加 --yes 后才会连接设备；默认仅处理页面 CRC 不同的变化页。")
            return 0
        actual = engine.flash(image, plan)
        print(f"PASS：设备 CRC32=0x{actual:08X}，状态 COMMITTED。")
        return 0
    except Exception as exc:
        print(f"FAIL：{exc}", file=sys.stderr)
        return 1
