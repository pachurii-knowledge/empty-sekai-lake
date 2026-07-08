// Sum-of-positives + parity classification over pseudo-random data. The inner
// branches are data-dependent on a PRNG => near-unpredictable => stresses the
// conditional predictor, mispredict recovery cost, and the branch stack.
#define N 8192
#define REPS 110
static int data[N];
static unsigned rng=88172645u;
static unsigned xr(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
int main(void){
  unsigned long acc=0;
  for(int r=0;r<REPS;r++){
    for(int i=0;i<N;i++) data[i]=(int)xr();
    for(int i=0;i<N;i++){
      int v=data[i];
      if(v>0) acc+=v;            // ~50% taken, unpredictable
      if((v&1)) acc^=0x5a5a;      // ~50%, unpredictable
      if(((v>>4)&3)==0) acc+=3;   // ~25%
    }
  }
  return (int)(acc ^ (acc>>32));
}
