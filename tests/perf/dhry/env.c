// env.c -- bare-metal environment for the upstream Dhrystone benchmark on niigo.
//
// The benchmark itself (dhrystone.c / dhrystone_main.c / dhrystone.h) is used
// VERBATIM from riscv-software-src/riscv-tests; only its *environment* is ported.
// Upstream's benchmarks/common/{crt.S,syscalls.c,test.ld} target the HTIF console
// (tohost/fromhost), which this SoC does not implement -- so this file supplies the
// handful of symbols dhrystone needs, over the NS16550A UART instead.
//
// Fidelity note, and the reason this file is not just a printf shim: strcpy(),
// strcmp() and memcpy() are called from INSIDE Dhrystone's timed loop (the spec
// calls this out explicitly), so their code quality is part of the score. They are
// therefore copied verbatim from upstream benchmarks/common/syscalls.c rather than
// hand-rolled, so this port scores the same routines every other riscv-tests
// Dhrystone result (Rocket, BOOM, ...) scores. printf() is NOT in the timed region,
// so a minimal version is fine and is the only routine written from scratch here.

#include <stdint.h>
#include <stddef.h>
#include "encoding.h"        // read_csr()

// Run count. run_dhrystone.sh passes -DNUMBER_OF_RUNS to every TU (and adds an
// #ifndef guard to its private copy of dhrystone.h so the command line wins);
// this fallback matches upstream's default.
#ifndef NUMBER_OF_RUNS
#define NUMBER_OF_RUNS 500
#endif

// ---------------------------------------------------------------- UART console
// NS16550A at 0x0D00_0000, reg-shift 2. THR is at +0x00; the model's transmitter
// is always ready and emits the byte to the console immediately ($write), so this
// costs a single uncached store -- no baud-rate spin to pollute the cycle count.
#define UART_THR (*(volatile unsigned char *)0x0D000000u)

static void uart_putc(char c)
{
  if (c == '\n')
    UART_THR = '\r';
  UART_THR = c;
}

static void uart_puts(const char *s)
{
  while (*s)
    uart_putc(*s++);
}

static void uart_putlong(long v)
{
  char buf[24];
  int i = 0;
  unsigned long u;

  if (v < 0) { uart_putc('-'); u = (unsigned long)(-v); }
  else        u = (unsigned long)v;

  do { buf[i++] = (char)('0' + (u % 10)); u /= 10; } while (u);
  while (i)
    uart_putc(buf[--i]);
}

// Minimal printf: enough for Dhrystone's two result lines (%ld) and the
// "measured time too small" strings. Outside the timed region, so its cost is
// irrelevant to the score. Supports %d %ld %lu %s %c %%.
int printf(const char *fmt, ...)
{
  __builtin_va_list ap;
  __builtin_va_start(ap, fmt);

  for (const char *p = fmt; *p; p++) {
    if (*p != '%') { uart_putc(*p); continue; }

    p++;
    int lng = 0;
    while (*p == 'l') { lng = 1; p++; }

    switch (*p) {
      case 'd': uart_putlong(lng ? __builtin_va_arg(ap, long)
                                 : (long)__builtin_va_arg(ap, int)); break;
      case 'u': uart_putlong(lng ? (long)__builtin_va_arg(ap, unsigned long)
                                 : (long)__builtin_va_arg(ap, unsigned int)); break;
      case 's': uart_puts(__builtin_va_arg(ap, const char *)); break;
      case 'c': uart_putc((char)__builtin_va_arg(ap, int)); break;
      case '%': uart_putc('%'); break;
      default:  uart_putc('%'); if (*p) uart_putc(*p); break;
    }
    if (!*p) break;
  }

  __builtin_va_end(ap);
  return 0;
}

// ------------------------------------------------------- timed-region counters
// Upstream dhrystone_main.c brackets the run loop with setStats(1)/setStats(0),
// immediately outside Start_Timer()/Stop_Timer(). Implementing setStats as an
// mcycle/minstret snapshot therefore yields loop-only cycles AND instructions
// (hence a loop-only IPC) with zero edits to the benchmark source. Dhrystone's own
// User_Time (rdcycle-based) remains the number the score is computed from; these
// counters are reported alongside it.
unsigned long dhry_stat_cycles;
unsigned long dhry_stat_instret;

void setStats(int enable)
{
  unsigned long c = read_csr(mcycle);
  unsigned long i = read_csr(minstret);

  if (enable) {
    dhry_stat_cycles  = c;
    dhry_stat_instret = i;
  } else {
    dhry_stat_cycles  = c - dhry_stat_cycles;
    dhry_stat_instret = i - dhry_stat_instret;
  }
}

// ------------------------------------------- string/memory routines (VERBATIM)
// Copied unmodified from riscv-tests benchmarks/common/syscalls.c. These are on
// Dhrystone's hot path -- do not "improve" them, that would silently change the score.

void* memcpy(void* dest, const void* src, size_t len)
{
  if ((((uintptr_t)dest | (uintptr_t)src | len) & (sizeof(uintptr_t)-1)) == 0) {
    const uintptr_t* s = src;
    uintptr_t *d = dest;
    uintptr_t *end = dest + len;
    while (d + 8 < end) {
      uintptr_t reg[8] = {s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]};
      d[0] = reg[0];
      d[1] = reg[1];
      d[2] = reg[2];
      d[3] = reg[3];
      d[4] = reg[4];
      d[5] = reg[5];
      d[6] = reg[6];
      d[7] = reg[7];
      d += 8;
      s += 8;
    }
    while (d < end)
      *d++ = *s++;
  } else {
    const char* s = src;
    char *d = dest;
    while (d < (char*)(dest + len))
      *d++ = *s++;
  }
  return dest;
}

void* memset(void* dest, int byte, size_t len)
{
  if ((((uintptr_t)dest | len) & (sizeof(uintptr_t)-1)) == 0) {
    uintptr_t word = byte & 0xFF;
    word |= word << 8;
    word |= word << 16;
    word |= word << 16 << 16;

    uintptr_t *d = dest;
    while (d < (uintptr_t*)(dest + len))
      *d++ = word;
  } else {
    char *d = dest;
    while (d < (char*)(dest + len))
      *d++ = byte;
  }
  return dest;
}

size_t strlen(const char *s)
{
  const char *p = s;
  while (*p)
    p++;
  return p - s;
}

int strcmp(const char* s1, const char* s2)
{
  unsigned char c1, c2;

  do {
    c1 = *s1++;
    c2 = *s2++;
  } while (c1 != 0 && c1 == c2);

  return c1 - c2;
}

char* strcpy(char* dest, const char* src)
{
  char* d = dest;
  while ((*d++ = *src++))
    ;
  return dest;
}

// ------------------------------------------------------------ entry + self-check
// Dhrystone's own result validation is printed through debug_printf(), which
// upstream defines as an empty function -- so a silently-wrong run would still
// print a plausible score. We therefore re-check the benchmark's global outputs
// against the reference values from the Dhrystone 2.1 listing after main()
// returns, and print an explicit verdict. Only the globals are checked: Ptr_Glob
// and Next_Ptr_Glob point into main()'s alloca'd frame, which is dead once main()
// has returned, so dereferencing them here would be undefined.

extern int  main(int argc, char **argv);

extern int  Int_Glob;
extern int  Bool_Glob;                 // Boolean == int
extern char Ch_1_Glob, Ch_2_Glob;
extern int  Arr_1_Glob[50];
extern int  Arr_2_Glob[50][50];
extern long User_Time;

void dhry_entry(void)
{
  int fail = 0;

  main(0, 0);

  // Reference values, Dhrystone 2.1 ("should be" lines in dhrystone_main.c).
  if (Int_Glob          != 5)              fail |= 1 << 0;
  if (Bool_Glob         != 1)              fail |= 1 << 1;
  if (Ch_1_Glob         != 'A')            fail |= 1 << 2;
  if (Ch_2_Glob         != 'B')            fail |= 1 << 3;
  if (Arr_1_Glob[8]     != 7)              fail |= 1 << 4;
  if (Arr_2_Glob[8][7]  != NUMBER_OF_RUNS + 10) fail |= 1 << 5;

  // Arr_2_Glob[8][7] == Number_Of_Runs + 10 also confirms the run count was not
  // retried/scaled by the Too_Small_Time loop, so NUMBER_OF_RUNS below is the
  // count the reported cycles actually correspond to.
  printf("DHRY-CHECK: %s (mask=%d)\n", fail ? "FAIL" : "PASS", fail);
  printf("DHRY-RUNS: %ld\n", (long)NUMBER_OF_RUNS);
  printf("DHRY-USER-TIME-CYCLES: %ld\n", User_Time);
  printf("DHRY-LOOP-CYCLES: %ld\n", (long)dhry_stat_cycles);
  printf("DHRY-LOOP-INSTRET: %ld\n", (long)dhry_stat_instret);
}
