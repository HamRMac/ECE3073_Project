#include "sys/alt_stdio.h"
#include "io.h"
#include <system.h>
#include "alt_types.h"
#include "altera_avalon_pio_regs.h"
#include <sys/alt_irq.h>
#include "priv/alt_legacy_irq.h"

// Define SDRAM BASE
#define SDRAM_BASEADDR 0x0000000
// Define VGA Characteristics
#define IMG_WIDTH 160
#define BUF_LAST_ROW_ADDR 19040
#define BUF_MAX_PIX 19200

char flipImgFlags = 0x0;

int * key_in_base = KEY_IN_BASE;
void * flags;

// Define edge counter for context
volatile int edge_capture;

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

// Function it uses the last 2 bits
// 0x1 is x flip, 0x2 is y flip
void writeImage(char flipFlags) {
	alt_putstr("WRITING IMAGE TO BUFFER\n");
	alt_printf("Flags %x\n",flipFlags);

	// Setup Draw Params
	int heightAddr = BUF_LAST_ROW_ADDR-1;

	// Setup Temp Data variables
	int data1;
	int data2;
	char xFlipped = (flipFlags & 0x1) != 0;
	char yFlipped = (flipFlags & 0x2) != 0;
	char performXFlip = yFlipped ? xFlipped^yFlipped : xFlipped;

	// Define Base For Flip
	int base = 0;
	if (yFlipped) {
		base += heightAddr;
	}

	int readOffset1;
	int readOffset2;

	int pixelAddr1;
	int pixelAddr2;

	// Flip in the x-direction
	for (int i = 0; i < heightAddr; i = i + IMG_WIDTH) {
		for (int j = 0; j < IMG_WIDTH/2; j++){
			// offset1 ----->	<----- offset 2 //
			readOffset1 = i+j;
			readOffset2 = i + IMG_WIDTH - j - 1;

			pixelAddr1 = base+readOffset1*(yFlipped ? -1 : 1);
			pixelAddr2 = base+readOffset2*(yFlipped ? -1 : 1);

			data1 = IORD_32DIRECT(SDRAM_BASEADDR,readOffset1*4)>>24;
			data2 = IORD_32DIRECT(SDRAM_BASEADDR,readOffset2*4)>>24;

			IOWR_32DIRECT(PB_ADR_BASE,0,(performXFlip ? pixelAddr2 : pixelAddr1));
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
			IOWR_32DIRECT(PB_DATA_BASE,0,data1);
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);

			IOWR_32DIRECT(PB_ADR_BASE,0,(performXFlip ? pixelAddr1 : pixelAddr2));
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
			IOWR_32DIRECT(PB_DATA_BASE,0,data2);
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);
		}
	}

	alt_putstr("WRITING COMPLETE\n");
	return;
}


// Define the handler
void KEY_IN_ISR(void * isr_context, alt_u32 id)
{
	alt_putstr("Interrupt Triggered\n");
	volatile int* button_edge_capture_ptr = (volatile int*) isr_context;
	*button_edge_capture_ptr = IORD_ALTERA_AVALON_PIO_EDGE_CAP(KEY_IN_BASE);
	//alt_printf("%x\n",*button_edge_capture_ptr);
	// Write to the edge capture register to reset it.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEY_IN_BASE, 0);

	flipImgFlags = flipImgFlags ^ (*button_edge_capture_ptr);
	writeImage(flipImgFlags);
}

// Init the ISR
static void init_button_pio()
{
	// Recast the edge_capture pointer to match the
	//alt_irq_register() function prototype.
	void* button_edge_capture_ptr = (void*) &edge_capture;
	// Enable interrupts for both kes
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(KEY_IN_BASE, 0x3);
	// Reset the edge capture register.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEY_IN_BASE, 0x0);
	// Register the ISR.
	alt_ic_isr_register(
		  KEY_IN_IRQ_INTERRUPT_CONTROLLER_ID,
		  KEY_IN_IRQ,
		  KEY_IN_ISR,
		  button_edge_capture_ptr,
		  0x0
  );
}

int main()
{ 

	alt_putstr("Begin VGA\n");

	// Clear buffer
	clrBuffer();

	// Draw the Initial Image
	writeImage(0x0);

	// Init Interrupt
	alt_putstr("INIT IRQ\n");
	init_button_pio();


	while (1) {}
	return 0;
}
