#include "sys/alt_stdio.h"
#include "io.h"
#include <system.h>
#include "alt_types.h"
#include <sys/alt_irq.h>
#include "priv/alt_legacy_irq.h"
// Define SDRAM BASE
#define SDRAM_BASEADDR 0x0000000
// Define VGA Characteristics
#define BUF_MAX_PIX 19200

int * key_in_base = KEY_IN_BASE;
void * context;
void * flags;

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

void image_yflip(){
	int data1;
	int data2;
	int width = 160;
	int heightAddr = 19040;
	for (int i = 0; i < heightAddr; i = i + 160) {
		for (int j = 0; j < width/2; j++){
		  int offset1 = i+j;
		  int offset2 = i+ width - j - 1;
		  data1 = IORD_32DIRECT(SDRAM_BASEADDR,offset1*4)>>24;
		  data2 = IORD_32DIRECT(SDRAM_BASEADDR,offset2*4)>>24;
		  //data1 = IORD_32DIRECT(PB_ADR_BASE,offset1)
		  //data2 = IORD_32DIRECT(PB_ADR_BASE,offset2)
		  //Enable write
		  IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
		  // Write Data

		  IOWR_32DIRECT(PB_ADR_BASE,0,offset2);
		  IOWR_32DIRECT(PB_DATA_BASE,0,data1);

		  IOWR_32DIRECT(PB_ADR_BASE,0,offset1);
		  IOWR_32DIRECT(PB_DATA_BASE,0,data2);
		  IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);
	  }
	}
	alt_putstr("Y FlIP COMPLETE\n");
}

void image_xflip(){
	int data1;
	int data2;
	int width = 160;
	int heightAddr = 19040;
	for (int i = 0; i < heightAddr/2; i = i + 160) {
		for (int j = 0; j < width; j++){
		  int offset1 = i+j;
		  int offset2 = heightAddr + j - i - 1;
		  data1 = IORD_32DIRECT(SDRAM_BASEADDR,offset1*4)>>24;
		  data2 = IORD_32DIRECT(SDRAM_BASEADDR,offset2*4)>>24;

		  //Enable write
		  IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
		  // Write Data
		  IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
		  IOWR_32DIRECT(PB_ADR_BASE,0,offset2);
		  IOWR_32DIRECT(PB_DATA_BASE,0,data1);


		  IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
		  IOWR_32DIRECT(PB_ADR_BASE,0,offset1);
		  IOWR_32DIRECT(PB_DATA_BASE,0,data2);
		  IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);
	  }
	}
	alt_putstr("X FlIP COMPLETE\n");
}

void KEY_IN_ISR(void * isr_context, alt_u32 id)
{
	image_yflip();
	*(key_in_base + 3) = 0;
	alt_putstr("KEY0\n");




}

int main()
{ 
  *(key_in_base + 3) = 0;
  alt_ic_isr_register(KEY_IN_IRQ_INTERRUPT_CONTROLLER_ID,KEY_IN_IRQ, KEY_IN_ISR,&context, 0x0);
  *(key_in_base + 2) = 1;
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

  image_xflip();




  while (1) {

  }

  return 0;
}
