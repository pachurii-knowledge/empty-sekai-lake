// tests/embench/boardsupport.c -- niigo bare-metal board support for Embench-IOT.
//
// Provides the Embench board hooks (initialise_board / start_trigger /
// stop_trigger), a minimal freestanding libc the benchmarks + support/beebsc.c
// need, and the harness main() -- which brackets benchmark() with mcycle/minstret
// snapshots and reports the TIMED REGION ONLY (start_trigger->stop_trigger delta,
// excluding warmup/verify) over the NS16550A UART, using the same marker
// convention as tests/perf/dhry/env.c. Built with -ffreestanding -nostdlib and
// linked with tests/perf/{start.S,bench.ld}; start.S ECALL-halts after main
// returns, firing the core's +perf_out dump. See scripts/run_embench.sh.

#include <stddef.h>
#include <stdint.h>
#include "support.h"

// ---- CSR reads (no encoding.h dependency) ----
static inline unsigned long rd_mcycle(void)   { unsigned long v; __asm__ volatile("csrr %0, mcycle"   : "=r"(v)); return v; }
static inline unsigned long rd_minstret(void) { unsigned long v; __asm__ volatile("csrr %0, minstret" : "=r"(v)); return v; }

// ---- NS16550A UART @ 0x0D00_0000 (reg-shift 2, THR at +0); the model's TX is
// always ready and $write's the byte, so a putc costs one uncached store. ----
#define UART_THR (*(volatile unsigned char *)0x0D000000u)
static void putc_(char c)        { if (c == '\n') UART_THR = '\r'; UART_THR = c; }
static void puts_(const char *s) { while (*s) putc_(*s++); }
static void putlong_(long v)     { char b[24]; int i = 0; unsigned long u = v < 0 ? (putc_('-'), -(unsigned long)v) : (unsigned long)v;
                                   do { b[i++] = '0' + u % 10; u /= 10; } while (u); while (i) putc_(b[--i]); }
static void puthex_(unsigned long u) { char b[24]; int i = 0; do { int d = u & 0xf; b[i++] = d < 10 ? '0' + d : 'a' + d - 10; u >>= 4; } while (u); while (i) putc_(b[--i]); }

// Minimal printf (off the timed path): %d/%i %u %x/%X/%p %s %c %%, l-modifier,
// width/flags skipped best-effort. Enough for benchmark verify/debug lines.
int printf(const char *fmt, ...)
{
  __builtin_va_list ap; __builtin_va_start(ap, fmt);
  for (const char *p = fmt; *p; p++) {
    if (*p != '%') { putc_(*p); continue; }
    p++;
    int lng = 0;
    while (*p == '-' || *p == '+' || *p == ' ' || *p == '#' || *p == '0') p++;
    while ((*p >= '0' && *p <= '9') || *p == '.') p++;
    while (*p == 'l') { lng = 1; p++; }
    switch (*p) {
      case 'd': case 'i': putlong_(lng ? __builtin_va_arg(ap, long) : (long)__builtin_va_arg(ap, int)); break;
      case 'u': putlong_(lng ? (long)__builtin_va_arg(ap, unsigned long) : (long)__builtin_va_arg(ap, unsigned int)); break;
      case 'x': case 'X': case 'p': puthex_(lng ? __builtin_va_arg(ap, unsigned long) : (unsigned long)__builtin_va_arg(ap, unsigned int)); break;
      case 's': puts_(__builtin_va_arg(ap, const char *)); break;
      case 'c': putc_((char)__builtin_va_arg(ap, int)); break;
      case '%': putc_('%'); break;
      default:  putc_('%'); if (*p) putc_(*p); break;
    }
    if (!*p) break;
  }
  __builtin_va_end(ap); return 0;
}

// ---- minimal freestanding libc (the benchmarks + beebsc.c call these) ----
void  *memcpy(void *d, const void *s, size_t n)       { char *dd = d; const char *ss = s; while (n--) *dd++ = *ss++; return d; }
void  *memset(void *d, int c, size_t n)               { char *dd = d; while (n--) *dd++ = (char)c; return d; }
void  *memmove(void *d, const void *s, size_t n)      { char *dd = d; const char *ss = s;
                                                        if (dd < ss) while (n--) *dd++ = *ss++;
                                                        else { dd += n; ss += n; while (n--) *--dd = *--ss; } return d; }
int    memcmp(const void *a, const void *b, size_t n) { const unsigned char *x = a, *y = b; while (n--) { if (*x != *y) return *x - *y; x++; y++; } return 0; }
size_t strlen(const char *s)                          { const char *p = s; while (*p) p++; return p - s; }
int    strcmp(const char *a, const char *b)           { unsigned char c1, c2; do { c1 = *a++; c2 = *b++; } while (c1 && c1 == c2); return c1 - c2; }
int    strncmp(const char *a, const char *b, size_t n){ while (n--) { unsigned char c1 = *a++, c2 = *b++; if (c1 != c2) return c1 - c2; if (!c1) return 0; } return 0; }
char  *strcpy(char *d, const char *s)                 { char *r = d; while ((*d++ = *s++)); return r; }
char  *strncpy(char *d, const char *s, size_t n)      { char *r = d; while (n && (*d++ = *s++)) n--; while (n--) *d++ = 0; return r; }
int    abs(int x)                                     { return x < 0 ? -x : x; }
void   free(void *p)                                  { (void)p; }   // beebs bump allocator never frees

// qsort (insertion sort; the one benchmark using it is off the timed path)
static void swp_(char *a, char *b, size_t sz) { while (sz--) { char t = *a; *a++ = *b; *b++ = t; } }
void qsort(void *base, size_t n, size_t sz, int (*cmp)(const void *, const void *))
{
  char *b = base;
  for (size_t i = 1; i < n; i++)
    for (size_t j = i; j > 0 && cmp(b + (j - 1) * sz, b + j * sz) > 0; j--)
      swp_(b + (j - 1) * sz, b + j * sz, sz);
}

// ---- static heap for the beebs bump allocator (benchmarks that malloc call
// init_heap_beebs themselves in initialise_benchmark, overriding this) ----
#ifndef EMBENCH_HEAP_SIZE
#define EMBENCH_HEAP_SIZE (64 * 1024)
#endif
static unsigned char g_heap[EMBENCH_HEAP_SIZE] __attribute__((aligned(16)));

// ---- Embench board hooks + the timed-region snapshot ----
static unsigned long t_cyc, t_ins;
void initialise_board(void) { init_heap_beebs((void *)g_heap, EMBENCH_HEAP_SIZE); }
void __attribute__((noinline)) start_trigger(void) { t_ins = rd_minstret(); t_cyc = rd_mcycle(); }
void __attribute__((noinline)) stop_trigger(void)  { unsigned long c = rd_mcycle(), i = rd_minstret();
                                                     t_cyc = c - t_cyc; t_ins = i - t_ins; }

// ---- harness main (Embench support/main.c contract + niigo UART markers) ----
extern int  benchmark(void);
extern int  verify_benchmark(int result);
extern void initialise_benchmark(void);
extern void warm_caches(int temperature);

int main(void)
{
  initialise_board();
  initialise_benchmark();
  warm_caches(WARMUP_HEAT);

  start_trigger();
  int result = benchmark();
  stop_trigger();

  int correct = verify_benchmark(result);
  printf("EMBENCH-VERIFY: %s\n", correct ? "PASS" : "FAIL");
  printf("EMBENCH-CYCLES: %lu\n", t_cyc);
  printf("EMBENCH-INSTRET: %lu\n", t_ins);
  return !correct;
}
