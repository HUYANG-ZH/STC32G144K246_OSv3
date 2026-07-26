# IMU 统一帧数据流

## 统一入口

应用层只读取 `service_imu_get_latest_sample()` 返回的
`service_imu_sample_t`。不要在 App 中直接读取 `imu660rc_*` 全局变量；
`service_imu_read_gyro*()` 仅为旧代码兼容保留。

```text
IMU660RC INT2 / P37（FIFO 水位：3 word）
        |
        | ISR：只锁存一次完整帧事件
        v
L1 BSP：SPI3 单次 DMA
  1 command + 3 x (tag + 6 data byte)
  [gyro] + [acc] + [SFLP game quaternion]
        |
        v
bsp_imu_sample_t 双缓冲完整帧
        |
        v
L2 Service：单位换算、gyro X/Z 零偏校准、完整帧发布
        |
        v
TIM7：motion_postprocess / element / attitude 共用同一局部快照
```

IMU FIFO 中 gyro、acc 和 SFLP game rotation vector 均配置为同一个
`BSP_IMU_QUARTERNION_RATE`。FIFO 水位固定为 3 个 word，并启用
`STOP_ON_WTM`；因此每次 DMA 只取一帧，且不会混入相邻帧。

SFLP FIFO 的 quaternion word 只存向量部分 xyz（半精度），标量 w 按单位
四元数关系在 BSP 中计算；Euler 随后由这同一帧 quaternion 计算。

## 快照语义

`sequence`、`drdy_tick`、`timestamp_tick` 和 `valid` 适用于该完整帧的全部字段。
gyro、acc、quaternion 与 Euler 不再拥有独立的序号、频率或有效标志。

项目继续使用既有的 `quaternion_0..3` 对外顺序，以避免改变已验证的车辆轴向
约定；在完成装车坐标系验证前，应用层不要自行把它们标为 x/y/z/w。

## 调度约束

- P37 ISR 不访问 SPI，也不执行 TFPU 运算。
- DMA ISR 只登记 DMA 成功/失败；半精度转换、Euler 和双缓冲发布在前台执行。
- 控制和积分只能在 `sequence` 改变时处理新帧，并以相邻
  `timestamp_tick` 的差值计算 dt。
- 若 DMA 超时或 FIFO tag 不完整，BSP 丢弃该帧并累计
  `bsp_imu_get_dma_error_count()`；最后一次有效帧保持不变，安全链可按其年龄处理。

## 板级验收

480 Hz 配置下，用逻辑分析仪确认：

1. P37 上升沿到一次 22-byte SPI3 DMA 的 CS/SCK 延迟；
2. 每次 DMA 数据含且仅含 gyro、acc、SFLP game 三个 tag；
3. `sequence` 对三类数据同步递增，`bsp_imu_get_dma_error_count()` 不增长；
4. 静止时 Euler、gyro 零偏和装车轴向符合车辆坐标约定。
