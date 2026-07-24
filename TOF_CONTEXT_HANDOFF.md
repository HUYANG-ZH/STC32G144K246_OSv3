# TOF Context Handoff

Updated: 2026-07-20

## Goal and hardware

- Project: STC32G144K246, Keil C251 target `stc32g144k246`.
- TOF wiring: software I2C, `P52 = SCL`, `P53 = SDA`, 7-bit address `0x29`.
- XSHUT and INT are not connected to the MCU and are disabled for both drivers.
- The hardware was originally identified as a bare VL53L0X/DL1A module, not a UART TOF200F module.
- Wireless output uses UART8 at 115200 baud on P92/P93, so it does not conflict with P52/P53.

## Current firmware mode

`project/user/main.c` is in a minimal TOF test mode. It initializes only:

1. `SystemStart()`
2. `service_timetick_init()`
3. `service_wireless_uart_init()`
4. `service_tof_init()`

All motor, encoder, IMU, inductor, negative-pressure, buzzer, packet, scheduler, and application modules are not initialized or run.

The main loop outputs at 10 Hz:

```text
tof_distance_mm,<uint16 value>
```

The timebase is 0.1 ms, so the output period is `1000` ticks. `8192U` is the invalid/not-ready sentinel; it is not a physical distance reported by the sensor.

## Layered API

The public service interface is intentionally unchanged:

```c
void service_tof_init(void);
uint16 service_tof_get_distance_mm(void);
```

Layering:

```text
main -> L2_SERVICE/service_tof -> L1_BSP/bsp_tof -> DL1A or DL1B device driver
```

Relevant files:

- `project/L2_SERVICE/service_tof.c/.h`
- `project/L1_BSP/bsp_tof.c/.h`
- `libraries/zf_device/zf_device_dl1a.c/.h`
- `libraries/zf_device/zf_device_dl1b.c/.h`
- `project/user/main.c`

The unused `project/user/vl53l0x_platform.c/.h` files are not in the Keil build. They are only an ST API platform layer and the rest of the ST API is absent.

## Driver selection

Selection is in `project/L1_BSP/bsp_tof.h`:

```c
#define BSP_TOF_DRIVER_DL1A (1U)
#define BSP_TOF_DRIVER_DL1B (2U)

#define BSP_TOF_DRIVER BSP_TOF_DRIVER_DL1B
```

Current selection: **DL1B**.

To select the original VL53L0X driver:

```c
#define BSP_TOF_DRIVER BSP_TOF_DRIVER_DL1A
```

Both branches were compiled successfully in earlier verification. Both use P52/P53 and expose the same service API. The DL1A source has been preserved.

## Current 8192 diagnosis

The current firmware outputs only `8192` for two independent reasons that must not be conflated:

1. **Likely device/driver mismatch.** The confirmed hardware was VL53L0X, but the current build selects DL1B. DL1A uses the VL53L0X 8-bit register protocol; DL1B uses a different 16-bit-register protocol. The shared I2C address `0x29` does not make them compatible. If the connected module is still VL53L0X, switch back to DL1A.
2. **The stock DL1B read logic contains defects.** In `zf_device_dl1b.c`:
   - Measurement code tests the whole `GPIO__TIO_HV_STATUS` byte with `if(data_buffer[2])`, while initialization treats bit 0 equal to zero as ready.
   - It compares the value read from `RESULT__RANGE_STATUS` against `0x89`, which is the register address, not the valid raw status. For VL53L1X-style data, valid raw status is checked as `(status & 0x1F) == 0x09`.
   - It clears the interrupt before reading result/status data.
   - It sets `dl1b_init_flag` even after the initialization wait times out.

The diagnostic fixes above were tested temporarily, but were deliberately reverted because the user asked to diagnose rather than authorize a fix. The current source and HEX therefore still contain the stock DL1B behavior.

Also note that `service_tof_init()` returns no status and the BSP maps initialization failure, not-ready, invalid range status, and out-of-range data to the same `8192U`, so wireless output alone cannot distinguish these cases.

## Build and verification

Keil command:

```powershell
& 'D:\C251keil\UV4\UV4.exe' -b seekfree.uvproj -t stc32g144k246 -j0
```

Run from:

```text
STC32G144K246_100Pin_Library/Seekfree_STC32G144K_100Pin_Opensource_Library/project/mdk
```

Latest build after restoring the stock DL1B source:

```text
0 Error(s), 0 Warning(s)
```

Outputs:

- `project/mdk/out_file/SEEKFREE.hex`
- `project/mdk/out_file/SEEKFREE.build_log.htm`
- `project/mdk/out_file/SEEKFREE.map`

Compilation success only proves source/toolchain correctness; hardware ranging still needs flashing and live testing.

## Important repository state

- The worktree is intentionally dirty and contains many generated Keil output changes.
- Do not reset or revert unrelated modifications.
- Existing non-TOF changes include newly added hardware-I2C files and common-header integration; these predated or accompanied this work and must be preserved.
- Small C251 compatibility changes were made so full builds remain warning-free, including hardware-I2C parameter naming, a wireless UART conditional local, and a local function-queue warning pragma.

## Recommended next action

First identify the physical sensor/module marking:

- If it is VL53L0X/DL1A, select `BSP_TOF_DRIVER_DL1A`, rebuild, flash, and test the 10 Hz output.
- If it is truly DL1B/VL53L1X, implement the four DL1B fixes listed above, rebuild, and add temporary diagnostic output for initialization result, ready bit, raw range status, and raw distance before converting failures to `8192U`.
