/* *********************************
//
//			ECE3073 Project
//             Milestone 2
//
// Authors:
//		Hamish McCoy (32474741)
//		Thomas Huang (32501617)
// Last Edited: 2/05/2024
//
// *********************************/

/* NOTE: If Eclipse can't find dependencies check that
 * the BSP folder is loaded into the workspace, clean build
 * then select 'BUILD ALL'
 */

/* ------------------------- *
 *     BEGIN GLOBAL SETUP	 *
 * ------------------------- */

// Include dependencies
#include <stdio.h>
#include "includes.h"
// MS 1 dependencies
#include "io.h"
#include <system.h>
#include "alt_types.h"
#include "altera_avalon_pio_regs.h"
#include <sys/alt_irq.h>
#include "priv/alt_legacy_irq.h"

/* Definition of Task Stacks */
#define   TASK_STACKSIZE       2048
OS_STK    mainTask_stk[TASK_STACKSIZE];
OS_STK    buttonManagerTask_stk[TASK_STACKSIZE];
OS_STK    displayImageTask_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define MAINTASK_PRIORITY      			1
#define BUTTONMANAGERTASK_PRIORITY      2
#define DISPLAYIMAGETASK_PRIORITY      			3

// -- Define constants --
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
volatile int keyPressedContext;

// Create all required semaphores
OS_EVENT* semKeyChange;

/* ------------------------- *
 *      END GLOBAL SETUP	 *
 *   BEGIN OPERATIONAL CODE	 *
 * ------------------------- */

/* ------------------------- *
 *      BEGIN MILESTONE 1	 *
 *      HELPER FUNCTIONS	 *
 * ------------------------- */

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
	return;
}

/* ------------------------- *
 *       END MILESTONE 1	 *
 *      HELPER FUNCTIONS	 *
 * ------------------------- */

/* ------------------------- *
 *     BEGIN IRQ FUNCTIONS	 *
 * ------------------------- */

/*KEY_IN_ISR(void * isr_context, alt_u32 id)
 * Inputs:
 * Define the key interrupt handler.
 * Currently just checks which button was pressed,
 * updates the global flip flag accordingly and then
 * redraws the image to the screen
 */
void KEY_IN_ISR(void * isr_context, alt_u32 id)
{
	// Retrieve the memory address to store the edge capture data
	volatile int* keyEdgeCapturePtr = (volatile int*) isr_context;
	// Get edge capture data
	*keyEdgeCapturePtr = IORD_ALTERA_AVALON_PIO_EDGE_CAP(KEY_IN_BASE);
	// Write to the edge capture register to reset it.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEY_IN_BASE, 0);
	// Post semaphore to acknowledge IRQ data is now available
	int err = OSSemPost(semKeyChange);
	if (err != OS_NO_ERR) printf("Failed to post semaphore semKeyChange: %x\n",err);
}

/* initButtonIRQ()
 * Initialises the KEY interrupt handler
 */
static void initButtonIRQ()
{
	// Recast the edge_capture pointer to match the
	//alt_irq_register() function prototype.
	void* keyEdgeCapturePtr = (void*) &keyPressedContext;
	// Enable interrupts for both key
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(KEY_IN_BASE, 0x3);
	// Reset the edge capture register.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEY_IN_BASE, 0x0);
	// Register the ISR.
	alt_ic_isr_register(
		  KEY_IN_IRQ_INTERRUPT_CONTROLLER_ID,
		  KEY_IN_IRQ,
		  (alt_isr_func) KEY_IN_ISR,
		  keyEdgeCapturePtr,
		  0x0
  );
}

/* ------------------------- *
 *      END IRQ FUNCTIONS	 *
 * ------------------------- */

/* ------------------------- *
 *    BEGIN MS2 FUNCTIONS	 *
 * ------------------------- */

INT32U conv(
		int* imgArray,
		int imgMaxX,
		int imgMaxY,
		int xPos,
		int yPos,
		int* kernel,
		int kernelSize, // Assume Kernel is Square
		int divisor
	){
	INT32U result;
	int numElements = kernelSize*kernelSize;
	// Get values from image array using supplied
	// coordinates and width/length and multiply with respective
	// Kernel value (use two for loops)


	// Divide result by divisor if necessary
	if (divisor > 1) result /= divisor;
	return result;
}

/* ------------------------- *
 *     END MS2 FUNCTIONS 	 *
 * ------------------------- */

/* ------------------------- *
 *    BEGIN SYSTEM TASKS 	 *
 * ------------------------- */

void buttonManagerTask (void* pdata) {
	INT8U semPendErr;
	while (1) {
		// Wait for interrupt
		OSSemPend(semKeyChange,0,&semPendErr);
		// Received Interrupt Semaphore (GO TIME)
		// Flip the flag in the global variable as required and redraw image
		flipImgFlags ^= keyPressedContext;
		writeImage(flipImgFlags);
		printf(" FLIP FLAGS %x\n",flipImgFlags);
	}
}

/* Displays the image to the screen */
void displayImageTask(void* pdata)
{
	printf("Image Display Task initialised\n");
	while (1)
	{
		writeImage(flipImgFlags);
		OSTimeDlyHMSM(0, 0, 0, 100);
	}
}

/* Displays the image to the screen */
void imageDownscaleTask(void* pdata)
{
	while (1)
	{

	}
}


void mainTask(void* pdata)
{
	// Compute CPU capacity with no task running
	OSStatInit();

	// Create Semaphores and IRQHs
	semKeyChange = OSSemCreate(1);
	initButtonIRQ();

	// For Debug Purposes
	//printf("OSIdleCtrMax is %ld\n", OSIdleCtrMax);

	// Check for errors
	int error_code;

	/* --------------
		CREATE TASKS
	   -------------- */
	// Create buttonManagerTask to handle IRQ
	printf("Attempting to create buttonManagerTask\n");
	error_code = OSTaskCreateExt(buttonManagerTask,
				  NULL,
				  (void *)&buttonManagerTask_stk[TASK_STACKSIZE-1],
				  BUTTONMANAGERTASK_PRIORITY,
				  BUTTONMANAGERTASK_PRIORITY,
				  buttonManagerTask_stk,
				  BUTTONMANAGERTASK_PRIORITY,
				  NULL,
				  0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating buttonManagerTask with error code %d\n", error_code); else printf("Created task successfully\n");

	printf("Attempting to create displayImageTask\n");
	error_code = OSTaskCreateExt(displayImageTask,
				  NULL,
				  (void *)&displayImageTask_stk[TASK_STACKSIZE-1],
				  DISPLAYIMAGETASK_PRIORITY,
				  DISPLAYIMAGETASK_PRIORITY,
				  displayImageTask_stk,
				  DISPLAYIMAGETASK_PRIORITY,
				  NULL,
				  0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating displayImageTask with error code %d\n", error_code); else printf("Created task successfully\n");

	/* ------------
		INDEF LOOP
	   ------------ */
	while (1)
	{
		// Delay for ages
		OSTimeDlyHMSM(1, 0, 0, 0);
	}
}

/* ------------------------- *
 *      END SYSTEM TASKS     *
 *     	  BEGIN MAIN()       *
 * ------------------------- */

/* The main function creates two task and starts multi-tasking */
int main(void)
{
	/* --------------
		PERFORM INIT
	   -------------- */
	// Clear buffer
	clrBuffer();


	/* ------------------
	    CREATE MAIN TASK
	   ------------------ */
	// Check for errors
	int error_code;
	// Attempt to create the main Task
	error_code = OSTaskCreateExt(mainTask,
				  NULL,
				  (void *)&mainTask_stk[TASK_STACKSIZE-1],
				  MAINTASK_PRIORITY,
				  MAINTASK_PRIORITY,
				  mainTask_stk,
				  MAINTASK_PRIORITY,
				  NULL,
				  0);
	if (error_code != 0) printf("Error creating mainTask with error code %d\n", error_code);
	/* --------------
		 START RTOS
	   -------------- */
	OSStart();
	return 0;
}
