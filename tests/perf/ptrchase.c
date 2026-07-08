// Pointer chase: a permuted linked list in a static array. Each load depends on
// the previous -> serializes on load-use latency. Exposes memory latency, ROB
// fill while a single load is outstanding, and low IPC.
#define N 4096
#define ITERS 300
static unsigned next[N];
int main(void){
  // Build a single cycle permutation via a simple LCG-derived stride pattern.
  for (int i=0;i<N;i++) next[i]=(unsigned)((i*1103515245u+12345u)%N);
  // Ensure it is a full traversal-ish chain (not strictly a single cycle, but
  // every step is a data-dependent load, which is what we want to stress).
  unsigned long sum=0; unsigned p=0;
  for (int it=0; it<ITERS; it++){
    for (int i=0;i<N;i++){ p=next[p]; sum+=p; }
  }
  return (int)(sum ^ (sum>>32));
}
