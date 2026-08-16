# AGENTS.md

## 重要规则

- **`car2`、`car3`、`car4` 是三辆完全不同的车**，参数、标定、控制逻辑可能完全不同，任何跨车参数/标定/逻辑都不得随意互相复制。
- **`car4` 分支是基于 `car3` 引出的全新分支**：与 `car2`、`car3` 均不合并（不 merge、不 rebase、不 cherry-pick 代码），保持独立演进；仅本 AGENTS.md 的规则同步属于例外。
- **修改任何参数或代码前，务必先检查当前在哪个 git 分支**（`git branch --show-current`），确认目标分支正确后再动手。
- 分支之间的差异（调参、标定、功能开关）不得随意互相覆盖。
- **在任何情况下，都不能关闭 FastBoot 模块**（`service_boot_request` 及其 UART1 SBLR 帧入口、main 中的 `service_boot_request_init/process`、XDATA 0xFFFC 热启动标记等），保证固件随时可经 FastBoot 升级/恢复。
- **调试代码可以在主循环中临时添加，但结束调试后必须恢复原样**（`git checkout` 还原或手动清除）。
- **`debugw` 作为单独模块不受影响**：其初始化/任务调用属于正式功能，不属于需清除的调试残留。
