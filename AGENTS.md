# AGENTS.md

## 重要规则

- **`car2` 和 `car3` 是两辆不同的车**，参数、标定、控制逻辑可能完全不同。
- **修改任何参数或代码前，务必先检查当前在哪个 git 分支**（`git branch --show-current`），确认目标分支正确后再动手。
- 两个分支之间的差异（调参、标定、功能开关）不得随意互相覆盖。
- **在任何情况下，都不能关闭 FastBoot 模块**（`service_boot_request` 及其 UART1 SBLR 帧入口、main 中的 `service_boot_request_init/process`、XDATA 0xFFFC 热启动标记等），保证固件随时可经 FastBoot 升级/恢复。
- **调试代码可以在主循环中临时添加，但结束调试后必须恢复原样**（`git checkout` 还原或手动清除）。
- **`debugw` 作为单独模块不受影响**：其初始化/任务调用属于正式功能，不属于需清除的调试残留。
