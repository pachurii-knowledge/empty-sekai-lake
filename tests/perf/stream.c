// Streaming copy + reduction over a large buffer -> store/load bandwidth, MemQ
// pressure, and (on the L1D build) streaming-miss behavior.
#define N 16384
#define REPS 90
static unsigned long src[N], dst[N];
int main(void){
  for(int i=0;i<N;i++) src[i]=(unsigned long)(i*2654435761u);
  unsigned long chk=0;
  for(int r=0;r<REPS;r++){
    for(int i=0;i<N;i++) dst[i]=src[i]+(unsigned long)r;   // streaming store
    unsigned long s=0;
    for(int i=0;i<N;i++) s+=dst[i];                         // streaming load/reduce
    chk^=s;
  }
  return (int)chk;
}
