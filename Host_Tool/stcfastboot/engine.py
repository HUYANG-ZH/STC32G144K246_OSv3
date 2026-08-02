from __future__ import annotations

import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from threading import Event
from typing import Callable, Iterable

ENTRY_FRAME = bytes.fromhex("53 42 4C 52 28 D7 32 47 14 4B 0F 45")
EXPECTED_APP_ACK = bytes.fromhex("A5 5A 53 42 4C 52 28 D7 0D 0A")

DFU_CMD_CONNECT = 0xA0
DFU_CMD_PROGRAM = 0xA2
DFU_CMD_REBOOT = 0xA4
DFU_CMD_ERASE_PAGE = 0xA5
DFU_CMD_CRC32 = 0xA6
DFU_CMD_STATUS = 0xA7
DFU_CMD_UPDATE_BEGIN = 0xA8
DFU_CMD_UPDATE_VERIFY = 0xA9
DFU_CMD_UPDATE_COMMIT = 0xAA
DFU_CMD_PAGE_CRC_TABLE = 0xAB

STATUS_OK = 0x00
STATUS_PROGRAMERR = 0x03
STATUS_ERASEERR = 0x04
STATUS_CRCERR = 0x05
STATUS_STATEERR = 0x06
STATUS_METAERR = 0x07

UPDATE_STATE_EMPTY = 0xFF
UPDATE_STATE_IN_PROGRESS = 0x7F
UPDATE_STATE_VERIFIED = 0x3F
UPDATE_STATE_COMMITTED = 0x1F
STATE_NAMES = {
    UPDATE_STATE_EMPTY: "EMPTY",
    UPDATE_STATE_IN_PROGRESS: "IN_PROGRESS",
    UPDATE_STATE_VERIFIED: "VERIFIED",
    UPDATE_STATE_COMMITTED: "COMMITTED",
}

EXPECTED_VERSION = 0x0206
IAP_BASE = 0xFC3800
AP_SIZE = 242 * 1024
AP_END = IAP_BASE + AP_SIZE - 1
APP_PHYSICAL_BEGIN = 0xFC4800
APP_LOGICAL_BEGIN = APP_PHYSICAL_BEGIN - IAP_BASE
APP_PHYSICAL_END = AP_END
APP_SIZE = APP_PHYSICAL_END - APP_PHYSICAL_BEGIN + 1
FLASH_PAGE_SIZE = 0x200
RESET_VECTOR = 0xFF0000
RESET_VECTOR_COMMIT_SIZE = 3
PROGRAM_PAYLOAD = 249
PAGE_CRC_BATCH = 30

LogCallback = Callable[[str], None]
ProgressCallback = Callable[[str, int, int], None]


@dataclass(frozen=True)
class HexImage:
    memory: dict[int, int]
    dense_app: bytes
    lowest: int
    highest: int
    source: Path

    @property
    def non_ff_count(self) -> int:
        return sum(value != 0xFF for value in self.dense_app)

    @property
    def crc32_full_app(self) -> int:
        return zlib.crc32(self.dense_app) & 0xFFFFFFFF


@dataclass(frozen=True)
class ProgramPacket:
    physical_address: int
    data: bytes
    reset_vector_commit: bool = False

    @property
    def logical_address(self) -> int:
        return self.physical_address - IAP_BASE


@dataclass(frozen=True)
class FlashPlan:
    erase_pages: tuple[int, ...]
    program_packets: tuple[ProgramPacket, ...]
    programmed_bytes: int
    iap_program_bytes: int


@dataclass(frozen=True)
class DeviceStatus:
    format_version: int
    state: int
    image_size: int
    expected_crc32: int

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, f"UNKNOWN_0x{self.state:02X}")

    @property
    def recovery_pending(self) -> bool:
        return self.state in (UPDATE_STATE_IN_PROGRESS, UPDATE_STATE_VERIFIED)


@dataclass
class FlashOptions:
    port: str
    cold_recovery: bool = False
    app_baud: int = 1_000_000
    app_baud_fallbacks: tuple[int, ...] = ()
    boot_baud: int | None = None
    byte_gap_ms: float = 1.0
    entry_retries: int = 3
    ack_timeout: float = 2.0
    response_timeout: float = 3.0
    erase_timeout: float = 4.0
    verify_timeout: float = 5.0
    handoff_delay: float = 0.45
    reboot_delay: float = 1.0
    retries: int = 3
    force_full_rewrite: bool = False

    def resolved_boot_baud(self) -> int:
        if self.boot_baud is not None:
            return self.boot_baud
        return 1_000_000

    def app_baud_candidates(self) -> tuple[int, ...]:
        result: list[int] = []
        for baud in (self.app_baud, *self.app_baud_fallbacks):
            if baud > 0 and baud not in result:
                result.append(baud)
        return tuple(result)


class FastBootError(RuntimeError):
    pass


class OperationCancelled(FastBootError):
    pass


def parse_intel_hex(path: str | Path) -> HexImage:
    source = Path(path)
    memory: dict[int, int] = {}
    base = 0
    eof_seen = False

    for line_no, raw_line in enumerate(source.read_text(encoding="ascii").splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        if eof_seen:
            raise ValueError(f"第 {line_no} 行位于 EOF 记录之后")
        if not line.startswith(":"):
            raise ValueError(f"第 {line_no} 行不是 Intel HEX 记录")
        try:
            record = bytes.fromhex(line[1:])
        except ValueError as exc:
            raise ValueError(f"第 {line_no} 行包含非法十六进制字符") from exc
        if len(record) < 5:
            raise ValueError(f"第 {line_no} 行记录过短")
        count = record[0]
        if len(record) != count + 5:
            raise ValueError(f"第 {line_no} 行长度字段不匹配")
        if sum(record) & 0xFF:
            raise ValueError(f"第 {line_no} 行校验和错误")

        address = (record[1] << 8) | record[2]
        record_type = record[3]
        payload = record[4 : 4 + count]

        if record_type == 0x00:
            for offset, value in enumerate(payload):
                absolute = base + address + offset
                if not APP_PHYSICAL_BEGIN <= absolute <= APP_PHYSICAL_END:
                    raise ValueError(
                        f"第 {line_no} 行地址 0x{absolute:06X} 超出 APP 区 "
                        f"0x{APP_PHYSICAL_BEGIN:06X}-0x{APP_PHYSICAL_END:06X}"
                    )
                old = memory.get(absolute)
                if old is not None and old != value:
                    raise ValueError(f"HEX 地址 0x{absolute:06X} 出现冲突数据")
                memory[absolute] = value
        elif record_type == 0x01:
            if count != 0:
                raise ValueError(f"第 {line_no} 行 EOF 记录长度错误")
            eof_seen = True
        elif record_type == 0x02:
            if count != 2:
                raise ValueError(f"第 {line_no} 行扩展段地址长度错误")
            base = int.from_bytes(payload, "big") << 4
        elif record_type == 0x04:
            if count != 2:
                raise ValueError(f"第 {line_no} 行扩展线性地址长度错误")
            base = int.from_bytes(payload, "big") << 16
        elif record_type in (0x03, 0x05):
            continue
        else:
            raise ValueError(f"第 {line_no} 行含不支持的记录类型 0x{record_type:02X}")

    if not memory:
        raise ValueError("HEX 中没有 APP 数据记录")
    if not eof_seen:
        raise ValueError("HEX 缺少 EOF 记录")

    reset_bytes = [memory.get(RESET_VECTOR + i, 0xFF) for i in range(RESET_VECTOR_COMMIT_SIZE)]
    if all(value == 0xFF for value in reset_bytes):
        raise ValueError(
            f"复位跳转指令 0x{RESET_VECTOR:06X}-"
            f"0x{RESET_VECTOR + RESET_VECTOR_COMMIT_SIZE - 1:06X} 为空"
        )

    dense = bytearray(b"\xFF" * APP_SIZE)
    for address, value in memory.items():
        dense[address - APP_PHYSICAL_BEGIN] = value

    return HexImage(memory, bytes(dense), min(memory), max(memory), source)


def _packetize_dense_range(
    address: int, data: bytes, *, commit: bool = False
) -> list[ProgramPacket]:
    packets: list[ProgramPacket] = []
    for offset in range(0, len(data), PROGRAM_PAYLOAD):
        chunk = data[offset : offset + PROGRAM_PAYLOAD]
        # Dense chunks collapse FF holes. The MCU skips FF bytes during IAP.
        if any(value != 0xFF for value in chunk):
            packets.append(ProgramPacket(address + offset, chunk, commit))
    return packets


def build_incremental_plan(
    image: HexImage, changed_pages: Iterable[int]
) -> FlashPlan:
    reset_page = RESET_VECTOR & ~(FLASH_PAGE_SIZE - 1)
    pages = sorted(set(changed_pages))
    for page in pages:
        if not APP_PHYSICAL_BEGIN <= page <= APP_PHYSICAL_END:
            raise ValueError(f"变化页 0x{page:06X} 超出 APP 区")
        if page & (FLASH_PAGE_SIZE - 1):
            raise ValueError(f"变化页 0x{page:06X} 未按 512 字节对齐")

    erase_pages: list[int] = []
    if reset_page in pages:
        erase_pages.append(reset_page)
    erase_pages.extend(page for page in pages if page != reset_page)

    packets: list[ProgramPacket] = []
    for page in pages:
        offset = page - APP_PHYSICAL_BEGIN
        page_data = image.dense_app[offset : offset + FLASH_PAGE_SIZE]
        if page == reset_page:
            packets.extend(
                _packetize_dense_range(
                    page + RESET_VECTOR_COMMIT_SIZE,
                    page_data[RESET_VECTOR_COMMIT_SIZE:],
                )
            )
        else:
            packets.extend(_packetize_dense_range(page, page_data))

    if reset_page in pages:
        commit_offset = RESET_VECTOR - APP_PHYSICAL_BEGIN
        commit_data = image.dense_app[
            commit_offset : commit_offset + RESET_VECTOR_COMMIT_SIZE
        ]
        if len(commit_data) != RESET_VECTOR_COMMIT_SIZE or all(
            value == 0xFF for value in commit_data
        ):
            raise ValueError("复位跳转指令没有可提交数据")
        packets.append(ProgramPacket(RESET_VECTOR, commit_data, True))

    programmed_bytes = sum(len(packet.data) for packet in packets)
    iap_program_bytes = sum(
        value != 0xFF for packet in packets for value in packet.data
    )
    return FlashPlan(
        tuple(erase_pages), tuple(packets), programmed_bytes, iap_program_bytes
    )


def build_flash_plan(image: HexImage) -> FlashPlan:
    """Worst-case full rewrite plan used by dry-run and force-full mode."""
    all_pages = tuple(
        range(APP_PHYSICAL_BEGIN, APP_PHYSICAL_END + 1, FLASH_PAGE_SIZE)
    )
    plan = build_incremental_plan(image, all_pages)
    if len(plan.erase_pages) != APP_SIZE // FLASH_PAGE_SIZE:
        raise AssertionError("擦除页数量错误")
    if len(set(plan.erase_pages)) != len(plan.erase_pages):
        raise AssertionError("擦除页重复")
    if not plan.program_packets or not plan.program_packets[-1].reset_vector_commit:
        raise AssertionError("复位向量不是最后一个写入包")
    if any(
        not packet.data or len(packet.data) > PROGRAM_PAYLOAD
        for packet in plan.program_packets
    ):
        raise AssertionError("PROGRAM 分包长度非法")
    return plan


def build_request(command: int, *, address: int = 0, payload: bytes = b"", value32: int | None = None) -> bytes:
    if command == DFU_CMD_PROGRAM:
        body = bytes((command,)) + address.to_bytes(4, "big") + bytes((len(payload),)) + payload
    elif command == DFU_CMD_ERASE_PAGE:
        body = bytes((command,)) + address.to_bytes(4, "big") + b"\x00"
    elif command == DFU_CMD_CRC32:
        if value32 is None:
            raise ValueError("CRC32 command requires length")
        body = bytes((command,)) + address.to_bytes(4, "big") + value32.to_bytes(4, "big")
    elif command == DFU_CMD_UPDATE_BEGIN:
        if value32 is None:
            raise ValueError("UPDATE_BEGIN requires CRC32")
        body = bytes((command,)) + address.to_bytes(4, "big") + value32.to_bytes(4, "big")
    elif command == DFU_CMD_PAGE_CRC_TABLE:
        if len(payload) != 1 or not 1 <= payload[0] <= PAGE_CRC_BATCH:
            raise ValueError("PAGE_CRC_TABLE requires a 1..60 page count")
        body = bytes((command,)) + address.to_bytes(4, "big") + payload
    else:
        body = bytes((command,))
    if len(body) > 255:
        raise ValueError("协议请求体过长")
    frame = bytearray((ord("#"), len(body)))
    frame.extend(body)
    frame.append(ord("$"))
    frame.append((-sum(frame)) & 0xFF)
    return bytes(frame)


def read_response(ser, timeout_s: float) -> tuple[int, bytes, bytes]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if ser.read(1) == b"@":
            break
    else:
        raise FastBootError("等待 Bootloader 响应头超时")

    header = bytearray(b"@")
    while len(header) < 3 and time.monotonic() < deadline:
        chunk = ser.read(3 - len(header))
        if chunk:
            header.extend(chunk)
    if len(header) != 3:
        raise FastBootError("Bootloader 响应头不完整")

    payload_size = header[2]
    tail = bytearray()
    required = payload_size + 2
    while len(tail) < required and time.monotonic() < deadline:
        chunk = ser.read(required - len(tail))
        if chunk:
            tail.extend(chunk)
    if len(tail) != required:
        raise FastBootError("Bootloader 响应不完整")

    raw = bytes(header + tail)
    if raw[-2] != ord("$") or (sum(raw) & 0xFF):
        raise FastBootError("Bootloader 响应封装或校验和错误：" + raw.hex(" ").upper())
    return raw[1], raw[3 : 3 + payload_size], raw


class FastBootEngine:
    def __init__(
        self,
        options: FlashOptions,
        *,
        log: LogCallback | None = None,
        progress: ProgressCallback | None = None,
        cancel_event: Event | None = None,
        serial_factory=None,
    ) -> None:
        self.options = options
        self.log = log or (lambda _message: None)
        self.progress = progress or (lambda _phase, _done, _total: None)
        self.cancel_event = cancel_event or Event()
        self.serial_factory = serial_factory

    def _check_cancel(self) -> None:
        if self.cancel_event.is_set():
            raise OperationCancelled("用户取消操作")

    def _open_serial(self, baud: int):
        if self.serial_factory is not None:
            return self.serial_factory(self.options.port, baud)
        try:
            import serial
        except ImportError as exc:
            raise FastBootError("缺少 pyserial，请执行 python -m pip install -r requirements.txt") from exc
        return serial.Serial(self.options.port, baud, timeout=0.05, write_timeout=1.0)

    def _request(self, ser, frame: bytes, timeout: float, operation: str) -> tuple[int, bytes, bytes]:
        last_error: Exception | None = None
        for attempt in range(1, max(1, self.options.retries) + 1):
            self._check_cancel()
            try:
                # Normal stop-and-wait transactions are already synchronized by
                # the exact response length. Purge/flush on every packet costs
                # substantial USB-driver latency, so only purge before a retry.
                if attempt > 1:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                ser.write(frame)
                return read_response(ser, timeout)
            except Exception as exc:
                last_error = exc
                if attempt < max(1, self.options.retries):
                    self.log(f"{operation} 响应失败，重试 {attempt}/{self.options.retries}：{exc}")
                    time.sleep(0.08)
        raise FastBootError(f"{operation} 在 {self.options.retries} 次尝试后失败：{last_error}")

    def _probe_bootloader_after_entry(self) -> bool:
        """Check whether APP switched even when its ACK was lost."""
        try:
            with self._open_serial(self.options.resolved_boot_baud()) as ser:
                time.sleep(max(self.options.handoff_delay, 0.0))
                ser.reset_input_buffer()
                status, payload, raw = self._request(
                    ser, build_request(DFU_CMD_CONNECT),
                    min(self.options.response_timeout, 0.8),
                    "入口后 CONNECT 探测",
                )
                if status != STATUS_OK or len(payload) != 2:
                    return False
                version = int.from_bytes(payload, "big")
                if version != EXPECTED_VERSION:
                    return False
                self.log(
                    "APP ACK 未捕获，但 Bootloader CONNECT 成功："
                    + raw.hex(" ").upper()
                )
                return True
        except Exception:
            return False

    def _probe_device_alive(self, ser) -> None:
        """After a failed DFU request, distinguish "transient noise" from
        "device rebooted back into the APP".

        A watchdog reset before UPDATE_BEGIN makes the chip boot the APP again
        (no recovery flag yet), so retrying DFU frames would burn
        retries x timeout on every batch.  A read-only CONNECT probe answers
        that in under a second.
        """
        try:
            status, _payload, _raw = self._request(
                ser, build_request(DFU_CMD_CONNECT),
                min(self.options.response_timeout, 0.8),
                "掉线探测 CONNECT",
            )
            if status == STATUS_OK:
                self.log("设备仍在 Bootloader 中（瞬时故障，非复位）")
                return
        except Exception:
            pass
        raise FastBootError(
            "设备已离开 Bootloader（可能在整区操作期间看门狗复位回 APP）。"
            "请重新运行升级命令再次进入 DFU。"
        )

    def enter_bootloader(self) -> None:
        if self.options.cold_recovery:
            self.log(f"冷恢复/直连模式：跳过 APP，Bootloader @ {self.options.resolved_boot_baud()}")
            time.sleep(max(self.options.handoff_delay, 0.0))
            return

        baud = self.options.app_baud
        gap_s = max(self.options.byte_gap_ms, 0.0) / 1000.0
        attempts = max(1, self.options.entry_retries)
        last_received = b""

        for attempt in range(1, attempts + 1):
            self.log(
                f"APP @ {baud}：发送进入命令 "
                f"（逐字节间隔 {self.options.byte_gap_ms:g} ms，"
                f"尝试 {attempt}/{attempts}）"
            )
            with self._open_serial(baud) as ser:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                for value in ENTRY_FRAME:
                    self._check_cancel()
                    ser.write(bytes((value,)))
                    ser.flush()
                    if gap_s > 0.0:
                        time.sleep(gap_s)

                deadline = time.monotonic() + max(self.options.ack_timeout, 0.1)
                received = bytearray()
                while time.monotonic() < deadline:
                    chunk = ser.read(ser.in_waiting or 1)
                    if chunk:
                        received.extend(chunk)
                        if EXPECTED_APP_ACK in received:
                            break
                last_received = bytes(received)

            if EXPECTED_APP_ACK in last_received:
                self.log(f"APP 已在 {baud} baud 切换到用户系统区")
                time.sleep(max(self.options.handoff_delay, 0.0))
                return

            # The APP may have switched after transmitting an ACK that the USB
            # adapter/driver did not deliver. A read-only CONNECT probe avoids
            # reporting a false failure and never erases or writes Flash.
            if self._probe_bootloader_after_entry():
                return

            if attempt < attempts:
                self.log("未收到 APP ACK，且 Bootloader 探测失败；重新发送入口帧")
                time.sleep(0.08)

        tail = last_received[-32:].hex(" ").upper() if last_received else "<empty>"
        raise FastBootError(
            f"没有收到 APP ACK，Bootloader 探测也失败；"
            f"固定波特率 {baud}，接收尾部 {tail}；尚未开始擦除"
        )

    def connect(self, ser) -> int:
        status, payload, raw = self._request(
            ser, build_request(DFU_CMD_CONNECT), self.options.response_timeout, "CONNECT"
        )
        self.log("CONNECT RX: " + raw.hex(" ").upper())
        if status != STATUS_OK or len(payload) != 2:
            raise FastBootError(f"CONNECT 响应错误，状态 0x{status:02X}")
        version = int.from_bytes(payload, "big")
        if version != EXPECTED_VERSION:
            raise FastBootError(
                f"Bootloader 版本 0x{version:04X}，期望 0x{EXPECTED_VERSION:04X}"
            )
        self.log(f"Bootloader 版本 0x{version:04X}")
        return version

    def get_status(self, ser) -> DeviceStatus:
        status, payload, _raw = self._request(
            ser, build_request(DFU_CMD_STATUS), self.options.response_timeout, "STATUS"
        )
        if status != STATUS_OK or len(payload) != 10:
            raise FastBootError(f"STATUS 响应错误，状态 0x{status:02X}，长度 {len(payload)}")
        result = DeviceStatus(
            payload[0], payload[1], int.from_bytes(payload[2:6], "big"),
            int.from_bytes(payload[6:10], "big"),
        )
        self.log(
            f"升级状态：{result.state_name}，size={result.image_size}，"
            f"CRC32=0x{result.expected_crc32:08X}"
        )
        return result

    def begin_update(self, ser, image: HexImage) -> DeviceStatus:
        frame = build_request(
            DFU_CMD_UPDATE_BEGIN,
            address=APP_SIZE,
            value32=image.crc32_full_app,
        )
        status, payload, _raw = self._request(
            ser, frame, self.options.erase_timeout, "UPDATE_BEGIN"
        )
        if payload or status != STATUS_OK:
            raise FastBootError(f"UPDATE_BEGIN 失败，状态 0x{status:02X}")
        result = self.get_status(ser)
        if (
            result.state != UPDATE_STATE_IN_PROGRESS
            or result.image_size != APP_SIZE
            or result.expected_crc32 != image.crc32_full_app
        ):
            raise FastBootError("设备升级元数据与目标镜像不一致")
        return result

    def scan_changed_pages(self, ser, image: HexImage) -> tuple[int, ...]:
        pages = tuple(
            range(APP_PHYSICAL_BEGIN, APP_PHYSICAL_END + 1, FLASH_PAGE_SIZE)
        )
        changed: list[int] = []
        started = time.perf_counter()
        self.log(f"批量比较设备页面 CRC32：{len(pages)} 页")

        for batch_start in range(0, len(pages), PAGE_CRC_BATCH):
            self._check_cancel()
            batch = pages[batch_start : batch_start + PAGE_CRC_BATCH]
            logical = batch[0] - IAP_BASE
            frame = build_request(
                DFU_CMD_PAGE_CRC_TABLE,
                address=logical,
                payload=bytes((len(batch),)),
            )
            try:
                status, payload, _raw = self._request(
                    ser, frame, self.options.verify_timeout,
                    f"PAGE_CRC_TABLE 0x{batch[0]:06X}",
                )
            except FastBootError:
                # A watchdog reset during scanning boots the APP (no recovery
                # flag yet).  Probe once instead of silently burning
                # retries x timeout on every remaining batch.
                self._probe_device_alive(ser)
                raise
            if status != STATUS_OK or len(payload) != len(batch) * 4:
                raise FastBootError(
                    f"页面 CRC 表响应错误：状态 0x{status:02X}，"
                    f"长度 {len(payload)}，期望 {len(batch) * 4}"
                )

            for index, page in enumerate(batch):
                device_crc = int.from_bytes(payload[index * 4 : index * 4 + 4], "big")
                offset = page - APP_PHYSICAL_BEGIN
                target_crc = zlib.crc32(
                    image.dense_app[offset : offset + FLASH_PAGE_SIZE]
                ) & 0xFFFFFFFF
                if device_crc != target_crc:
                    changed.append(page)

            done = batch_start + len(batch)
            self.progress("scan", done, len(pages))
            self.log(
                f"页面 CRC {done}/{len(pages)}，已发现 {len(changed)} 个变化页"
            )

        elapsed = time.perf_counter() - started
        self.log(
            f"页面比较完成：变化 {len(changed)}/{len(pages)} 页，"
            f"耗时 {elapsed:.2f}s"
        )
        return tuple(changed)

    def erase_application(self, ser, plan: FlashPlan) -> float:
        total = len(plan.erase_pages)
        if total == 0:
            self.log("无需擦除页面")
            return 0.0
        started = time.perf_counter()
        self.log(f"擦除变化页：{total} 页")
        for index, physical in enumerate(plan.erase_pages, 1):
            self._check_cancel()
            frame = build_request(DFU_CMD_ERASE_PAGE, address=physical - IAP_BASE)
            status, payload, _raw = self._request(
                ser, frame, self.options.erase_timeout, f"ERASE_PAGE 0x{physical:06X}"
            )
            if payload or status != STATUS_OK:
                detail = "页擦除命令失败" if status == STATUS_ERASEERR else f"状态 0x{status:02X}"
                raise FastBootError(f"ERASE_PAGE 0x{physical:06X} {detail}")
            if index == 1 or index == total or index % 4 == 0:
                self.progress("erase", index, total)
            if index == 1 or index == total or index % 32 == 0:
                self.log(f"擦除 {index}/{total} 页，当前 0x{physical:06X}")
        elapsed = time.perf_counter() - started
        self.log(f"页面擦除完成：{total} 页，耗时 {elapsed:.2f}s")
        return elapsed

    def program_application(self, ser, plan: FlashPlan) -> float:
        total = len(plan.program_packets)
        if total == 0:
            self.log("变化页目标内容均为 FF，无需 PROGRAM")
            return 0.0
        started = time.perf_counter()
        self.log(
            f"高速 PROGRAM：{total} 包，串口载荷 {plan.programmed_bytes} 字节，"
            f"实际非 FF IAP {plan.iap_program_bytes} 字节"
        )
        for index, packet in enumerate(plan.program_packets, 1):
            self._check_cancel()
            frame = build_request(
                DFU_CMD_PROGRAM, address=packet.logical_address, payload=packet.data
            )
            status, payload, _raw = self._request(
                ser, frame, self.options.response_timeout,
                f"PROGRAM 0x{packet.physical_address:06X}",
            )
            if payload or status != STATUS_OK:
                detail = "块写入命令失败" if status == STATUS_PROGRAMERR else f"状态 0x{status:02X}"
                raise FastBootError(f"PROGRAM 0x{packet.physical_address:06X} {detail}")
            if index == 1 or index == total or index % 8 == 0 or packet.reset_vector_commit:
                self.progress("program", index, total)
            if packet.reset_vector_commit:
                self.log(
                    f"最终提交复位向量 0x{packet.physical_address:06X}，"
                    f"{len(packet.data)} 字节"
                )
            elif index == 1 or index == total or index % 64 == 0:
                self.log(f"写入 {index}/{total} 包，当前 0x{packet.physical_address:06X}")
        elapsed = time.perf_counter() - started
        rate = plan.programmed_bytes / max(elapsed, 0.001) / 1024.0
        self.log(
            f"PROGRAM 完成：{total} 包，耗时 {elapsed:.2f}s，"
            f"有效串口载荷 {rate:.1f} KiB/s"
        )
        return elapsed

    def calculate_crc32(self, ser, logical_address: int, length: int) -> int:
        if logical_address < APP_LOGICAL_BEGIN or length <= 0:
            raise FastBootError("CRC32 范围非法")
        if logical_address + length > AP_SIZE:
            raise FastBootError("CRC32 范围超出 APP 区")
        status, payload, _raw = self._request(
            ser,
            build_request(DFU_CMD_CRC32, address=logical_address, value32=length),
            self.options.verify_timeout,
            "CRC32",
        )
        if status != STATUS_OK or len(payload) != 4:
            raise FastBootError(
                f"CRC32 响应错误：状态 0x{status:02X}，长度 {len(payload)}"
            )
        return int.from_bytes(payload, "big")

    def verify_current_application(
        self, image: HexImage
    ) -> tuple[int, DeviceStatus, int]:
        """Read-only full APP CRC32 check against a supplied HEX image."""
        self.enter_bootloader()
        with self._open_serial(self.options.resolved_boot_baud()) as ser:
            time.sleep(0.08)
            ser.reset_input_buffer()
            version = self.connect(ser)
            status_info = self.get_status(ser)
            actual = 0
            try:
                self.log("设备端只读计算完整 APP CRC32")
                actual = self.calculate_crc32(ser, APP_LOGICAL_BEGIN, APP_SIZE)
                self.log(
                    f"CRC32：device=0x{actual:08X}, host=0x{image.crc32_full_app:08X}"
                )
            finally:
                # A read-only check must not strand a healthy device in the loader.
                if not self.options.cold_recovery and not status_info.recovery_pending:
                    self.reboot(ser)
            if actual != image.crc32_full_app:
                raise FastBootError(
                    f"CRC32 不一致：device=0x{actual:08X}, "
                    f"host=0x{image.crc32_full_app:08X}"
                )
            self.log(f"CRC32 PASS：0x{actual:08X}")
            return version, status_info, actual

    def verify_update(self, ser, image: HexImage) -> int:
        self.log("设备端计算完整 APP CRC32")
        status, payload, _raw = self._request(
            ser,
            build_request(DFU_CMD_UPDATE_VERIFY),
            self.options.verify_timeout,
            "UPDATE_VERIFY",
        )
        if len(payload) != 4:
            raise FastBootError("UPDATE_VERIFY 未返回 4 字节 CRC32")
        actual = int.from_bytes(payload, "big")
        if status != STATUS_OK:
            raise FastBootError(
                f"设备 CRC32 校验失败：device=0x{actual:08X}, "
                f"host=0x{image.crc32_full_app:08X}, status=0x{status:02X}"
            )
        if actual != image.crc32_full_app:
            raise FastBootError(
                f"CRC32 不一致：device=0x{actual:08X}, host=0x{image.crc32_full_app:08X}"
            )
        self.log(f"CRC32 PASS：0x{actual:08X}")
        status_info = self.get_status(ser)
        if status_info.state != UPDATE_STATE_VERIFIED:
            raise FastBootError("CRC32 通过后设备状态未进入 VERIFIED")
        return actual

    def commit_update(self, ser) -> DeviceStatus:
        status, payload, _raw = self._request(
            ser, build_request(DFU_CMD_UPDATE_COMMIT), self.options.response_timeout,
            "UPDATE_COMMIT",
        )
        if payload or status != STATUS_OK:
            raise FastBootError(f"UPDATE_COMMIT 失败，状态 0x{status:02X}")
        result = self.get_status(ser)
        if result.state != UPDATE_STATE_COMMITTED:
            raise FastBootError("设备状态没有进入 COMMITTED")
        self.log("升级状态已提交：COMMITTED")
        return result

    def reboot(self, ser) -> None:
        if self.options.cold_recovery:
            self.log("冷恢复写入完成：不发送 REBOOT；请断电、解除 P3.3 下拉后再上电")
            return
        frame = build_request(DFU_CMD_REBOOT)
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        self.log("已发送 REBOOT")
        time.sleep(max(self.options.reboot_delay, 0.0))

    def flash(self, image: HexImage, plan: FlashPlan | None = None) -> int:
        self.enter_bootloader()
        boot_baud = self.options.resolved_boot_baud()
        flash_started = False
        total_started = time.perf_counter()
        try:
            with self._open_serial(boot_baud) as ser:
                time.sleep(0.08)
                ser.reset_input_buffer()
                self.connect(ser)
                previous = self.get_status(ser)

                if previous.state == UPDATE_STATE_VERIFIED:
                    if (
                        previous.image_size != APP_SIZE
                        or previous.expected_crc32 != image.crc32_full_app
                    ):
                        raise FastBootError(
                            "设备处于 VERIFIED，但目标 CRC32 与当前 HEX 不同；"
                            "请先提交/恢复原目标镜像"
                        )
                    self.log("检测到已校验未提交状态，直接完成 COMMITTED")
                    self.commit_update(ser)
                    self.reboot(ser)
                    return image.crc32_full_app

                if previous.state == UPDATE_STATE_IN_PROGRESS:
                    flash_started = True
                    if (
                        previous.image_size != APP_SIZE
                        or previous.expected_crc32 != image.crc32_full_app
                    ):
                        raise FastBootError(
                            "设备存在另一目标镜像的 IN_PROGRESS 状态；"
                            "为避免丢失恢复标志，只允许继续写入相同 CRC32 的 HEX"
                        )
                    self.log("检测到同一目标的 IN_PROGRESS：按页面 CRC 断点续写")
                elif previous.state not in (UPDATE_STATE_EMPTY, UPDATE_STATE_COMMITTED):
                    raise FastBootError(f"不支持的升级状态 {previous.state_name}")

                if self.options.force_full_rewrite:
                    changed_pages = tuple(
                        range(
                            APP_PHYSICAL_BEGIN,
                            APP_PHYSICAL_END + 1,
                            FLASH_PAGE_SIZE,
                        )
                    )
                    self.log("已启用强制全量重写：跳过页面差异扫描")
                else:
                    changed_pages = self.scan_changed_pages(ser, image)

                if not changed_pages:
                    self.log("设备每个页面 CRC32 均与目标一致，无需擦除或写入")
                    if previous.state == UPDATE_STATE_IN_PROGRESS:
                        actual = self.verify_update(ser, image)
                        self.commit_update(ser)
                    else:
                        actual = image.crc32_full_app
                    self.reboot(ser)
                    elapsed = time.perf_counter() - total_started
                    self.log(f"零写入完成，总耗时 {elapsed:.2f}s")
                    return actual

                runtime_plan = build_incremental_plan(image, changed_pages)
                self.log(
                    f"Turbo 计划：变化 {len(runtime_plan.erase_pages)} 页，"
                    f"PROGRAM {len(runtime_plan.program_packets)} 包，"
                    f"串口载荷 {runtime_plan.programmed_bytes} 字节，"
                    f"实际非 FF IAP {runtime_plan.iap_program_bytes} 字节"
                )

                if previous.state != UPDATE_STATE_IN_PROGRESS:
                    self.begin_update(ser, image)
                    flash_started = True

                self.erase_application(ser, runtime_plan)
                self.program_application(ser, runtime_plan)

                verify_started = time.perf_counter()
                actual = self.verify_update(ser, image)
                self.log(
                    f"设备整区 CRC32 耗时 {time.perf_counter() - verify_started:.2f}s"
                )
                self.commit_update(ser)
                self.reboot(ser)
                elapsed = time.perf_counter() - total_started
                self.log(
                    f"Turbo 升级完成：变化 {len(runtime_plan.erase_pages)} 页，"
                    f"总耗时 {elapsed:.2f}s"
                )
                return actual
        except Exception:
            if flash_started:
                self.log("升级已开始；设备会保留 IN_PROGRESS/VERIFIED 标志并自动停留在 Bootloader")
            raise

    def inspect_device(self) -> tuple[int, DeviceStatus]:
        self.enter_bootloader()
        with self._open_serial(self.options.resolved_boot_baud()) as ser:
            time.sleep(0.08)
            ser.reset_input_buffer()
            version = self.connect(ser)
            status = self.get_status(ser)
            # A normal status query must not strand a healthy device in the loader.
            # Pending updates intentionally stay in recovery mode.
            if not self.options.cold_recovery and not status.recovery_pending:
                self.reboot(ser)
            return version, status


def reconstruct_plan_image(plan: FlashPlan) -> bytes:
    """Host-test helper: apply a plan to an erased APP image."""
    dense = bytearray(b"\xFF" * APP_SIZE)
    for packet in plan.program_packets:
        start = packet.physical_address - APP_PHYSICAL_BEGIN
        dense[start : start + len(packet.data)] = packet.data
    return bytes(dense)
