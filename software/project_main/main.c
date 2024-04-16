// *********************************
//
//	ECE3073 Project
// Milestone 1 - Task 2 & 3
// Code for reading image from SDRAM and
// putting it into the pixel buffer.
// Also handles interrupts for button presses
// to flip the image
//
// Authors:
//		Hamish McCoy (32474741)
//		Thomas Huang (32501617)
// Last Edited: 1/04/2024
//
// *********************************

/* NOTE: If Eclipse can't find dependencies check that
 * the BSP folder is loaded into the workspace, clean build
 * then select 'BUILD ALL'
 */

// Include Dependencies
#include "sys/alt_stdio.h"
#include "io.h"
#include <system.h>
#include "alt_types.h"
#include "altera_avalon_pio_regs.h"
#include <sys/alt_irq.h>
#include "priv/alt_legacy_irq.h"

// -- Define constants
// Define SDRAM BASE
#define SDRAM_BASEADDR 0x0000000
// Define VGA Characteristics
#define IMG_WIDTH 160
#define BUF_LAST_ROW_ADDR 19040
#define BUF_MAX_PIX 19200

// Create global variable to define how image is flipped
char flipImgFlags = 0x0;

// Define pointer to the KEY BASE Memory address
int * key_in_base = KEY_IN_BASE;

// Define edge counter for interrupt context
// Used to tell which button was pressed
volatile int edge_capture;

/* Use system defines for the following
 * PB_ADR_BASE
 * PB_DATA_BASE
 * PBUFF_WREN_BASE
 * SW_IN_BASE
 * LEDR_OUT_BASE
 * KEY_IN_BASE
 */

/*
 *  clrBuffer()
 *  Clears the pixel buffer. Used to clear screen
 *  when processor restarts/program initialises
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

/*
 * writeImage(char flipFlags)
 * Inputs: flipFlags
 * Writes the images in SDRAM to the pixel buffer
 * Uses the 2 LSBs to determine if the image should
 * be flipped when it is drawn
 * 0x1 is x flip, 0x2 is y flip
 */
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
	// Due to the way the image is drawn to the screen
	// if yFlipped is true the processor will also
	// flip the image in the X direction (flips in x and y)
	// To correct this we check if the image is requested to be
	// flipped in y only. If it is we flip the x direction back
	// to correct for the errant x flip.
	// If y is not needed to be flipped, we only flip if x needs
	// to be flipped
	char performXFlip = yFlipped ? xFlipped^yFlipped : xFlipped;

	// Define Base For Flip and add offset if required
	int base = 0;
	if (yFlipped) {
		base += heightAddr;
	}

	// Define offset and address variables
	int readOffset1;
	int readOffset2;
	int pixelAddr1;
	int pixelAddr2;

	// Loop to draw the image to the screen
	for (int i = 0; i < heightAddr; i = i + IMG_WIDTH) {
		for (int j = 0; j < IMG_WIDTH/2; j++){
			// Offset read directions
			// offset1 ----->	<----- offset 2 //
			readOffset1 = i+j;
			readOffset2 = i + IMG_WIDTH - j - 1;

			// Calculate which pixel to read
			pixelAddr1 = base+readOffset1*(yFlipped ? -1 : 1);
			pixelAddr2 = base+readOffset2*(yFlipped ? -1 : 1);

			// Get the data from this pixel addresses above
			data1 = IORD_32DIRECT(SDRAM_BASEADDR,readOffset1*4)>>24;
			data2 = IORD_32DIRECT(SDRAM_BASEADDR,readOffset2*4)>>24;

			// Write the first pixel to the correct location in the buffer
			IOWR_32DIRECT(PB_ADR_BASE,0,(performXFlip ? pixelAddr2 : pixelAddr1));
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
			IOWR_32DIRECT(PB_DATA_BASE,0,data1);
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);

			// Write the first pixel to the correct location in the buffer
			IOWR_32DIRECT(PB_ADR_BASE,0,(performXFlip ? pixelAddr1 : pixelAddr2));
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
			IOWR_32DIRECT(PB_DATA_BASE,0,data2);
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);
		}
	}

	alt_putstr("WRITING COMPLETE\n");
	return;
}


/*KEY_IN_ISR(void * isr_context, alt_u32 id)
 * Inputs:
 * Define the key interrupt handler.
 * Currently just checks which button was pressed,
 * updates the global flip flag accordingly and then
 * redraws the image to the screen
 */
void KEY_IN_ISR(void * isr_context, alt_u32 id)
{
	alt_putstr("Interrupt Triggered\n");
	// Retrieve the memory address to store the edge capture data
	volatile int* button_edge_capture_ptr = (volatile int*) isr_context;
	// Get edge capture data
	*button_edge_capture_ptr = IORD_ALTERA_AVALON_PIO_EDGE_CAP(KEY_IN_BASE);
	// Write to the edge capture register to reset it.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEY_IN_BASE, 0);
	// Flip the flag in the global variable as required and redraw image
	flipImgFlags = flipImgFlags ^ (*button_edge_capture_ptr);
	writeImage(flipImgFlags);
}

/* init_button_pio()
 * Initialises the KEY interrupt handler
 */
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

/* main()
 * - Sets up the interrupt handler
 * - Clears the buffer
 * - Draws the initial image to the screen
 */
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

	// Infinite loop to wait for interrupts
	while (1) {}
	return 0;
}
