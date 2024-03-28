/* 
 	--- MEMORY TEST ----
 */

#include "sys/alt_stdio.h"
#include <io.h>
// Define mem offset and start
#define SDRAM_OFFSET 0x2000000
#define START_ADDR 0x0 + SDRAM_OFFSET
// Define Upper Limit of test hex(33554432/32) = 0x100000
#define MAX_ADDR 0x1FFFFFF + SDRAM_OFFSET


int main()
{ 
  alt_putstr("Beginning SDRAM Test!\n");
  // Initialise MEM addr
  int currentADDR = START_ADDR;
  // Initialise error count
  int errs = 0;
  int readback = 0;

  //Begin Loop
  while (currentADDR != MAX_ADDR) {
	  // Print Progress
	  if ((currentADDR % 0x10000) == 0) {
	  	  alt_printf("Testing 0x%x\n",currentADDR-SDRAM_OFFSET);
	  }
	  // Write all ones to mem addr
	  // 0xFF is equivalent to all 1s
	  IOWR_8DIRECT(currentADDR, 0, (char)0xFF);
	  readback = IORD_8DIRECT(currentADDR, 0);
	  if (readback != 0xFF) {
		  errs++;
	  }

	  // Write all zeros to mem addr
	  IOWR_8DIRECT(currentADDR, 0, 0);
	  readback = IORD_8DIRECT(currentADDR, (char)0);
	  if (readback != 0) {
		  errs++;
	  }

	  // Increment pointer
	  currentADDR++;
  }
  alt_putstr("Test Complete!\n");
  alt_printf("There were %x errors\n",errs);

  return 0;
}
