// CRC32 over a buffer, many passes. ALU/shift-xor bound, small table loads,
// highly predictable loop branch -> tests sustained commit/issue width.
#define BUF 8192
#define REPS 220
static unsigned char buf[BUF];
static unsigned tbl[256];
int main(void){
  for(unsigned i=0;i<256;i++){unsigned c=i;for(int k=0;k<8;k++)c=(c&1)?(0xEDB88320u^(c>>1)):(c>>1);tbl[i]=c;}
  for(int i=0;i<BUF;i++) buf[i]=(unsigned char)((i*31+7)&0xff);
  unsigned long chk=0;
  for(int r=0;r<REPS;r++){
    unsigned crc=0xffffffffu;
    for(int i=0;i<BUF;i++) crc=tbl[(crc^buf[i])&0xff]^(crc>>8);
    chk+=crc^0xffffffffu;
  }
  return (int)chk;
}
