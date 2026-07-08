// Recursive quicksort over a pseudo-random array, re-sorted many times.
#define N 2048
#define REPS 90
static int a[N];
static unsigned rng=2463534242u;
static unsigned xr(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static void qs(int*x,int lo,int hi){
  if(lo>=hi) return;
  int pv=x[(lo+hi)>>1], i=lo, j=hi;
  while(i<=j){
    while(x[i]<pv) i++;
    while(x[j]>pv) j--;
    if(i<=j){int t=x[i];x[i]=x[j];x[j]=t;i++;j--;}
  }
  qs(x,lo,j); qs(x,i,hi);
}
int main(void){
  unsigned long chk=0;
  for(int r=0;r<REPS;r++){
    for(int i=0;i<N;i++) a[i]=(int)(xr()&0xffff);
    qs(a,0,N-1);
    for(int i=0;i<N;i++) chk+=(unsigned)a[i]*(unsigned)i;
  }
  return (int)chk;
}
