$NOMOD51                ; 防止标准51寄存器定义冲突
NAME TFPU_LIB           ; 模块名称

DMAIR   DATA    0EDH    ; TFPU控制寄存器

;---------------------------------------------------------
; 代码段定义
;---------------------------------------------------------
?PR?TFPU_MATH?TFPU_LIB SEGMENT ECODE
RSEG    ?PR?TFPU_MATH?TFPU_LIB

PUBLIC  tfpu_init?
PUBLIC  tfpu_add?
PUBLIC  tfpu_sub?
PUBLIC  tfpu_mul?
PUBLIC  tfpu_div?
PUBLIC  tfpu_sin?
PUBLIC  tfpu_cos?
PUBLIC  tfpu_tan?
PUBLIC  tfpu_atan?
PUBLIC  tfpu_sqrt?
PUBLIC  tfpu_int2float?
PUBLIC  tfpu_float2int?

;=========================================================
; 函数: tfpu_init
;=========================================================
tfpu_init?:
    MOV     DMAIR, #31H     ; 复位
    NOP
    NOP
    MOV     DMAIR, #32H     ; 清除异常
    NOP
    NOP
    MOV     DMAIR, #3EH     ; 使用系统时钟 (最稳)
    NOP
    NOP
    ERET

;=========================================================
; 双操作数函数 (Add, Sub, Mul, Div)
; 编译器已将 Arg1 放入 DR4 (R4-R7) -> 对应 TFPU 的 AR
; 编译器已将 Arg2 放入 DR0 (R0-R3) -> 对应 TFPU 的 BR
; 我们直接发指令即可！无需搬运！
;=========================================================

tfpu_add?:
    MOV     DMAIR, #1CH     ; Result = a + b
    ERET

tfpu_sub?:
    MOV     DMAIR, #1DH     ; Result = a - b
    ERET

tfpu_mul?:
    MOV     DMAIR, #1EH     ; Result = a * b
    ERET

tfpu_div?:
    MOV     DMAIR, #1FH     ; Result = a / b
    ERET

;=========================================================
; 单操作数函数
; Arg1 在 DR4 -> 对应 TFPU 的 AR
;=========================================================

tfpu_sqrt?:
    MOV     DMAIR, #20H
    ERET

tfpu_sin?:
    MOV     DMAIR, #2DH
    ERET

tfpu_cos?:
    MOV     DMAIR, #2EH
    ERET

tfpu_tan?:
    MOV     DMAIR, #2FH
    ERET

tfpu_atan?:
    MOV     DMAIR, #30H
    ERET

tfpu_int2float?:
    MOV     DMAIR, #29H
    ERET

tfpu_float2int?:
    MOV     DMAIR, #25H
    ERET

END