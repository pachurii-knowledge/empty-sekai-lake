#ifndef _RVMODEL_MACROS_H
#define _RVMODEL_MACROS_H

#define RVMODEL_DATA_SECTION

// niigo-lake uses the standard machine-mode CSRs, so let the arch-test
// framework install its default M-mode trap handler / trampoline
// (RVTEST_BOOT_TO_MMODE). Defining RVMODEL_BOOT_TO_MMODE here would suppress
// that and leave mtvec/mscratch uninitialized, which breaks every privileged
// test (they rely on the framework trap handler for expected faults).
#define STANDARD_SM_SUPPORTED

#define RVMODEL_BOOT

#define RVMODEL_HALT_PASS \
  li a0, 10             ;\
  ecall                 ;\
  rvtest_pass_halt:     ;\
  j rvtest_pass_halt    ;\

#define RVMODEL_HALT_FAIL \
  li a0, 11             ;\
  ecall                 ;\
  rvtest_fail_halt:     ;\
  j rvtest_fail_halt    ;\

#define RVMODEL_IO_WRITE_STR(_R1, _R2, _R3, _STR_PTR)

#define RVMODEL_INTERRUPT_LATENCY 2000
#define RVMODEL_TIMER_INT_SOON_DELAY 100

// CLINT timer / software interrupt registers.
#define RVMODEL_MTIMECMP_ADDRESS  0x02004000
#define RVMODEL_MTIME_ADDRESS     0x0200BFF8
#define RVMODEL_MSIP_ADDRESS      0x02000000

// External interrupts go through the PLIC (base 0x0C000000). Source 1 is routed
// to context 0 (M-external -> mip.MEIP); source 2 to context 1 (S-external ->
// mip.SEIP). SET configures the source's priority (>0) and per-context enable
// and asserts its software-pending latch (0x1000 = set); CLR deasserts it
// (0x1004 = clear). Software interrupts use the CLINT msip (machine) and mip
// (supervisor). All of these helpers run in M-mode.
#define RVMODEL_SET_MEXT_INT(_R1, _R2)             ;\
  li _R2, 0x0C000004; li _R1, 1; sw _R1, 0(_R2)    ;\
  li _R2, 0x0C002000; li _R1, 2; sw _R1, 0(_R2)    ;\
  li _R2, 0x0C001000; li _R1, 2; sw _R1, 0(_R2)    ;

#define RVMODEL_CLR_MEXT_INT(_R1, _R2)             ;\
  li _R2, 0x0C001004; li _R1, 2; sw _R1, 0(_R2)    ;

#define RVMODEL_SET_SEXT_INT(_R1, _R2)             ;\
  li _R2, 0x0C000008; li _R1, 1; sw _R1, 0(_R2)    ;\
  li _R2, 0x0C002080; li _R1, 4; sw _R1, 0(_R2)    ;\
  li _R2, 0x0C001000; li _R1, 4; sw _R1, 0(_R2)    ;

#define RVMODEL_CLR_SEXT_INT(_R1, _R2)             ;\
  li _R2, 0x0C001004; li _R1, 4; sw _R1, 0(_R2)    ;

#define RVMODEL_SET_MSW_INT(_R1, _R2)              ;\
  li _R2, 0x02000000; li _R1, 1; sw _R1, 0(_R2)    ;

#define RVMODEL_CLR_MSW_INT(_R1, _R2)              ;\
  li _R2, 0x02000000; sw zero, 0(_R2)              ;

// Supervisor software interrupt: the framework's clr_Ssw_int handler already
// clears SSIP mode-correctly (csrc mip in M-mode, csrc sip in S-mode), so the
// model clear must be empty -- emitting csrc mip here would execute it from the
// S-mode handler (illegal: mip is M-only) when the SSI is delegated. The set is
// only invoked from M-mode, where writing mip.SSIP is legal.
#define RVMODEL_SET_SSW_INT(_R1, _R2)              ;\
  li _R1, 2; csrs mip, _R1                         ;

#define RVMODEL_CLR_SSW_INT(_R1, _R2)

#endif
