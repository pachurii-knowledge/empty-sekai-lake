// Dense integer matrix multiply, repeated. Good ILP, mul-heavy, regular access.
#define M 48
#define REPS 18
static int A[M][M], B[M][M], C[M][M];
int main(void){
  for(int i=0;i<M;i++)for(int j=0;j<M;j++){A[i][j]=(i*7+j*3)&255;B[i][j]=(i*3+j*5)&255;}
  unsigned long chk=0;
  for(int r=0;r<REPS;r++){
    for(int i=0;i<M;i++)for(int j=0;j<M;j++){
      int s=0; for(int k=0;k<M;k++) s+=A[i][k]*B[k][j];
      C[i][j]=s;
    }
    for(int i=0;i<M;i++)for(int j=0;j<M;j++) chk+=C[i][j];
  }
  return (int)chk;
}
