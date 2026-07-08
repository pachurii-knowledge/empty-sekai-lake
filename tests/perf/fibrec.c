// Deeply recursive fib -> exercises the return-address stack, call/return
// prediction, and branch-checkpoint pressure from the base-case branches.
static long fib(long n){ if(n<2) return n; return fib(n-1)+fib(n-2); }
int main(void){
  unsigned long chk=0;
  for(int r=0;r<1;r++) chk+=(unsigned long)fib(31);
  return (int)chk;
}
