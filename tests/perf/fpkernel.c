// Double-precision SAXPY + dot product, repeated. Long-latency, serialized FP
// pipeline (CVFPU): exposes FP issue serialization and FMA latency as the limiter.
#define N 2048
#define REPS 150
static double x[N], y[N];
int main(void){
  for(int i=0;i<N;i++){x[i]=(double)((i%97)+1)*0.5;y[i]=(double)((i%53)+1)*0.25;}
  double acc=0.0;
  for(int r=0;r<REPS;r++){
    double a=1.0000001+ (double)(r&7)*1e-9;
    for(int i=0;i<N;i++) y[i]=a*x[i]+y[i];   // saxpy (FMA)
    double d=0.0;
    for(int i=0;i<N;i++) d+=x[i]*y[i];        // dot (dependent FADD chain)
    acc+=d;
  }
  return (int)acc;
}
