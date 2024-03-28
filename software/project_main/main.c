#include "sys/alt_stdio.h"
#include <io.h>
#include <system.h>

// Define SDRAM BASE
#define SDRAM_BASEADDR 0x0000000
// Define VGA Characteristics
#define BUF_MAX_PIX 19200


/* Use system defines
 * PB_ADR_BASE
 * PB_DATA_BASE
 * PBUFF_WREN_BASE
 * SW_IN_BASE
 * LEDR_OUT_BASE
 * KEY_IN_BASE
 */

void clrBuffer() {
	for (int i = 0; i < BUF_MAX_PIX; i++) {
		  // Set Buffer Address
		  IOWR_32DIRECT(PB_ADR_BASE,0,i);
		  // Enable Writing
		  IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
		  // Write Data
		  IOWR_32DIRECT(PB_DATA_BASE,0,0);
		  // Disable writing
		  IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);
	  }
	return;
}

int main()
{ 
  alt_putstr("Begin VGA\n");
  // Clear buffer
  clrBuffer();

  /* Event loop never exits. */
  int data;

  for (int i = 0; i < BUF_MAX_PIX; i++) {
	  data = IORD_32DIRECT(SDRAM_BASEADDR,i*4)>>24;
	  //IOWR_32DIRECT(SDRAM_BASEADDR,i,i & 0xF);
	  //alt_printf("A%x: %x\n",SDRAM_BASEADDR+i,data);
	  // Set Buffer Address
	  IOWR_32DIRECT(PB_ADR_BASE,0,i);
	  // Enable Writing
	  IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
	  // Write Data
	  IOWR_32DIRECT(PB_DATA_BASE,0,data);
	  // Test Data
	  //IOWR_32DIRECT(PB_DATA_BASE,0,i%17);
	  // Disable writing
	  IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);
  }
  alt_putstr("PIXBUFF WRITE COMPLETE\n");
  while (1) {
  }

  return 0;
}
