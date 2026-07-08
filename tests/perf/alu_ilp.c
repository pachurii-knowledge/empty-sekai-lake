// Pure-ALU ILP microbenchmark: 8 independent accumulator chains, NO memory in
// the hot loop. Isolates front-end dispatch width -- with abundant ILP and no
// memory stalls, sustained IPC == effective dispatch width (2 under the 2-wide
// RVC realign, up to 4 on the non-RVC 4-wide front-end).
int main(void){
  unsigned long a0=1,a1=2,a2=3,a3=4,a4=5,a5=6,a6=7,a7=8;
  for(unsigned long i=0;i<4000000UL;i++){
    a0+=i;  a1^=i;      a2+=i+i;  a3^=(i<<1);
    a4+=(i|1); a5^=(i&0x7f); a6+=(i^0x33); a7^=(i>>1);
  }
  return (int)(a0+a1+a2+a3+a4+a5+a6+a7);
}
