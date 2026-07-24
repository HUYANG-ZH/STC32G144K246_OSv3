$NOMOD51

NAME STC_DPU32

; STC32 DPU32 hardware implementation of the C251 runtime 32-bit divide helpers.
; This source is intentionally compiled by the current LARGE/HUGE target instead of
; linking the donor prebuilt library, whose memory model differs from this project.
DPUOP   DATA 0D8H

?PR?STC_DPU32  SEGMENT ECODE
RSEG    ?PR?STC_DPU32

PUBLIC  ?C?ULDIV?
?C?ULDIV?:
        MOV     DPUOP,#09FH
        ERET

PUBLIC  ?C?ULIDIV?
?C?ULIDIV?:
        XRL     WR0,WR0
        MOV     DPUOP,#09FH
        ERET

PUBLIC  ?C?SLDIV?
?C?SLDIV?:
        MOV     DPUOP,#0A0H
        ERET

PUBLIC  ?C?SIDIV?
?C?SIDIV?:
        MOV     DPUOP,#0A4H
        ERET

END
