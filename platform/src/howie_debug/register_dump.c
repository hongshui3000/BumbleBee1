#include "print_howie.h"
#include "register_dump.h"

void reg_dump(int *base_addr, int count)
{
	  for(int i=0;i<count;i++)
	  {
        print_howie("register 0x%x == %d",base_addr,*base_addr);  
				base_addr++;//address ++
	  }
}
