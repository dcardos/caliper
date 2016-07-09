#include <support.h>

void nf_unlock(int id)
{
#ifndef DEBUG
#if HAS_LOCKS
  volatile uint* loc = (uint*)((uint)UN_LOCK);  
  *loc  = id;
#endif
#else
  log("UNLOCKING LOCK %d\n", id);
#endif
}

void nf_lock(int id)
{
#ifndef DEBUG
#if HAS_LOCKS
  volatile uint* loc = (uint*)((uint)GET_LOCK);  
  *loc  = id;
#endif
#else
  log("LOCKING LOCK %d\n", id);
#endif
}


int nf_tid()
{
#ifndef DEBUG
  asm(".set    noreorder");
  asm("addu $2,$26,$0"); 
  asm("jr $31"); 
#endif
  return 0;
}

void relax()
{
#ifndef DEBUG 
  int i;
	int test = nf_time() & 0x07;
	for(i=0; i<test; i++)
	{
		asm("nop"
				:
				:
				);
	}
	asm("nop\n\t" 
      "nop\n\t" 
      "nop\n\t" 
      "nop\n\t" 
      "nop\n\t" 
      "nop\n\t" 
      "nop\n\t" 
      "nop\n\t" 
      "nop\n\t" 
      "nop" 
      : 
      :  );  
#endif 
}
