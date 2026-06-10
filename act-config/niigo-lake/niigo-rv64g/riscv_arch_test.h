// niigo-lake wrapper: pull in the standard ACT headers, then override boot
// macros that assume a writable mstatus CSR (not implemented on this DUT).
//
// Signature ELFs (-DSIGNATURE) keep the default INIT_FLOAT_VECTOR_STATE so Sail
// can enable the F extension via mstatus before touching fcsr.
#include_next "riscv_arch_test.h"

#ifndef SIGNATURE
.purgem INIT_FLOAT_VECTOR_STATE
.macro INIT_FLOAT_VECTOR_STATE
    #if defined(F_SUPPORTED) || defined(ZFINX_SUPPORTED)
      csrw fcsr, zero
    #endif
.endm
#endif
