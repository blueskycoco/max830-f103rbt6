#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
extern void DBG_PutChar(char ptr);
int _close(int file) {
  return 0;
}

int _fstat(int file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file) {
  return 1;
}

int _lseek(int file, int ptr, int dir) {
  return 0;
}

int _open(const char *name, int flags, int mode) {
return -1;
}

int _read(int file, char *ptr, int len) {
  int todo;//,ch;
  if(len == 0)
    return 0;
 // uart_wait_rcv();
  //*ptr++ = uart_read();
  for(todo = 1; todo < len; todo++) {
  //	ch=uart_read();
   // if(ch==-1) { break; }
    //*ptr++ = ch;
  }
  return todo;
}

char *heap_end = 0;
caddr_t _sbrk(int incr) {
  extern char _end; /* Defined by the linker */
  extern char __cs3_heap_end; /* Defined by the linker */
  char *prev_heap_end;
  if (heap_end == 0) {
    heap_end = &_end;
  }  
  prev_heap_end = heap_end;
  if (heap_end + incr > &__cs3_heap_end) {
    /* Heap and stack collision */
    return (caddr_t)0;
  }
  heap_end += incr;
  return (caddr_t) prev_heap_end;
}

int _write(int file, char *ptr, int len) {
  int todo;
  for (todo = 0; todo < len; todo++) {
   DBG_PutChar(*ptr++);
  }
  return len;
}
