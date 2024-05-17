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
// Define Compiler Directives
#define VERSION 2
#define ENABLE_DEBUG_OUTPUT 0

// Include dependencies
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
OS_STK    switchManagerTask_stk[TASK_STACKSIZE];
OS_STK    imageProcessorTask_stk[TASK_STACKSIZE];
OS_STK    imageOneTask_stk[TASK_STACKSIZE];
OS_STK    imageTwoTask_stk[TASK_STACKSIZE];
OS_STK    imageThreeTask_stk[TASK_STACKSIZE];
OS_STK    imageFourTask_stk[TASK_STACKSIZE];
OS_STK	  singleImageTask_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define MAINTASK_PRIORITY      			1
#define BUTTONMANAGERTASK_PRIORITY      2
#define SWITCHMANAGERTASK_PRIORITY      3

#define MUTEXSINGIMG_PRIORITY			4
#define MUTEXDISP1_PRIORITY				5
#define MUTEXDISP2_PRIORITY				6
#define MUTEXDISP3_PRIORITY				7
#define MUTEXDISP4_PRIORITY				8
#define IMAGEPROCESSORTASK_PRIORITY 	9
#define SINGLEIMAGETASK_PRIORITY		10
#define IMAGEONETASK_PRIORITY			11
#define IMAGETWOTASK_PRIORITY			12
#define IMAGETHREETASK_PRIORITY			13
#define IMAGEFOURTASK_PRIORITY			14

// -- Define constants --
// Define SDRAM BASE
#define SDRAM_BASEADDR 0x0000000
// Define VGA Characteristics
#define IMG_WIDTH 160
#define IMG_HEIGHT 120
#define BUF_LAST_ROW_ADDR 19040
#define BUF_MAX_PIX 19200

#define FLIPIMG_BASE 0x12C00
#define EDGEIMG_BASE 0x25800
#define BLURING_BASE 0x38400

// Create global variable to define how image is flipped
char flipImgFlags = 0x0;
int  singleImgMode;
// Define edge counter for interrupt context
// Used to tell which button was pressed
volatile int keyPressedContext;
volatile int SwChangedContext;

// Create all required semaphores
OS_EVENT* semKeyChange;
OS_EVENT* semSwChange;
OS_EVENT* semLockFlipImgPointer;
OS_EVENT* semLockBlurImgPointer;
OS_EVENT* semLockEdgeImgPointer;
OS_EVENT* semLockBaseImgPointer;
OS_EVENT* semSwitchChange;
OS_EVENT* semSingleImag;
OS_EVENT* semDisplay1;
OS_EVENT* semDisplay2;
OS_EVENT* semDisplay3;
OS_EVENT* semDisplay4;
OS_EVENT* mutexKey;



// Define reusable kernels
// Note That convolutions are performed row-wise first
int kernDownscale2x[4] = { 1, 1, 1, 1};
int kernEdgeDetectH[9] = {-1,-2,-1, 0, 0, 0, 1, 2, 1};
int kernEdgeDetectV[9] = {-1, 0, 1,-2, 0, 2,-1, 0, 1};
int kernBlur3x[9]	   = { 1, 1, 1, 1, 1, 1, 1, 1, 1};

// Define Processing Time vars
INT32U flipTime = 0;
INT32U blurTime = 0;
INT32U edgeTime = 0;
INT32U totalTime = 0;
INT32U currentlyDislayed = 0;


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
	printf("Posting semaphore semKeyChange\n");
	int err = OSSemPost(semKeyChange);
	if (err != OS_NO_ERR) printf("Failed to post semaphore semKeyChange: %x\n",err);
}

void SW_IN_ISR(void * isr_context, alt_u32 id)
{
	// Retrieve the memory address to store the edge capture data
	volatile int* swEdgeCapturePtr = (volatile int*) isr_context;
	// Get edge capture data
	*swEdgeCapturePtr = IORD_ALTERA_AVALON_PIO_EDGE_CAP(SW_IN_BASE);
	// Write to the edge capture register to reset it.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(SW_IN_BASE, 0);
	// Post semaphore to acknowledge IRQ data is now available
	int err = OSSemPost(semSwitchChange);
	if (err != OS_NO_ERR) printf("Failed to post semaphore semSwChange: %x\n",err);

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

static void initSwIRQ()
{
	// Recast the edge_capture pointer to match the
	//alt_irq_register() function prototype.
	void* SwEdgeCapturePtr = (void*) &SwChangedContext;
	// Enable interrupts for both key
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(SW_IN_BASE, 0x3FF);
	// Reset the edge capture register.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(SW_IN_BASE, 0x0);
	// Register the ISR.
	alt_ic_isr_register(
		  SW_IN_IRQ_INTERRUPT_CONTROLLER_ID,
		  SW_IN_IRQ,
		  (alt_isr_func) SW_IN_ISR,
		  SwEdgeCapturePtr,
		  0x0
  );
}

/* ------------------------- *
 *      END IRQ FUNCTIONS	 *
 * ------------------------- */

/* ------------------------- *
 *    BEGIN MS2 FUNCTIONS	 *
 * ------------------------- */
void imageToBuffer(int* imgPtr, int horizStart, int vertStart) {
	int width = IMG_WIDTH/2;
	int height = IMG_HEIGHT/2;
	int position;
	for (int col = 0; col < width; col++) {
		for (int row = 0; row < height; row++) {
			position = horizStart + col + vertStart + row*IMG_WIDTH;
			IOWR_32DIRECT(PB_ADR_BASE,0, position);
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
			IOWR_32DIRECT(PB_DATA_BASE,0,*(imgPtr+2+col+row*80));
			IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);
		}
	}
}
void imageToBufferSDRAM(int baseAddress){
	int data;
	for (int i = 0; i < BUF_MAX_PIX; i++) {
		//Read from SDRAM
		data = IORD_32DIRECT(baseAddress,i*4)>>24;
		//Write to buffer
		IOWR_32DIRECT(PB_ADR_BASE,0,i);
		IOWR_32DIRECT(PBUFF_WREN_BASE,0,1);
		IOWR_32DIRECT(PB_DATA_BASE,0,data);
		IOWR_32DIRECT(PBUFF_WREN_BASE,0,0);

	}
}

// Function to display a digit on a hex dislay
void displayDigit (int hexBayBase, int digitOffset, int digit) {
	int data =  IORD(hexBayBase, 0) & (0xffff00ffff >> (2-digitOffset)*8);
	int digitBits;
	switch (digit) {
	case 0: digitBits = 0b11000000; break;
	case 1: digitBits = 0b11111001; break;
	case 2: digitBits = 0b10100100; break;
	case 3: digitBits = 0b10110000; break;
	case 4: digitBits = 0b10011001; break;
	case 5: digitBits = 0b10010010; break;
	case 6: digitBits = 0b10000010; break;
	case 7: digitBits = 0b11111000; break;
	case 8: digitBits = 0b10000000; break;
	case 9: digitBits = 0b10011000; break;
	}
	data |= (digitBits << digitOffset*8);
	IOWR(hexBayBase, 0, data);
}

//int seperableConv (int* kernel, int* imgPatch,size){
//	int result = 0;
//	return result;
//	for (int col = 0; col<size;col++){
//
//	}
//	for (int row = 0; row<size;row++){
//		//result += kernal[] * imgPatch[];
//	}
//
//}

int conv(
		int* imgPatch,
		int* kernel,
		int dim, // Assume Kernel is Square this is the 1d dim
		int divisor
	){
	int result = 0; // Must be signed in-case we use a negative kernel
	// Get values from patch array using supplied
	// and multiply with respective
	// Kernel value (use two for loops)
	for (int col=0; col<dim;col++) {
		for (int row=0; row<dim;row++) {
			// Add the result of the kernel and array multiplication to a running total
			result += (*(imgPatch+col+row*dim))*(*(kernel+col+row*dim));
		}
	}

	// Divide result by divisor if necessary
	if (divisor > 1) result /= divisor;
	return result;
}

// [COLS,ROWS,IMGDATA...,IMGDATA...,...]

int* downscaleImg2x(int* imageArray, int cols, int rows, int padding) {
	// This requests a new block of memory to put our down-scaled image
	// Free this afterwards or we will have a memory leak!
	int outputCols = cols/2;
	int outputRows = rows/2;

	int* output = (int*)malloc((outputCols*outputRows+2)*sizeof(int));
	*(output) = outputCols;
	*(output+1) = outputRows;
	// Iterate through each row/column and get the relevant pixel, perform the conv
	int patch[4];
	int offset[2];

	int bytesToCopy = sizeof(int)*2;
	int writeIndex = 0;
	int temp;
	for (int row = 0; row < rows-1; row+=2){
		for (int col = 0; col < cols-1; col+=2){
			offset[0] = col+row*cols; 		// Calculate pixel offset row1
			offset[1] = col+(row+1)*cols;	// Calculate pixel offset row2
			memcpy(patch,imageArray+padding+offset[0],bytesToCopy); // Copy first row to patch
			memcpy(patch+2,imageArray+padding+offset[1],bytesToCopy); // Copy second row to patch
			temp = conv(patch,kernDownscale2x,2,0); // Calculate convolution
			*(output+2+writeIndex) = temp >> 2;
			writeIndex++;
		}
	}
	// Return the memory location with the downscaled image
	return output;
}

int* edgeDetectionConv(int* imageArray, int cols, int rows, int direction) {
	// This requests a new block of memory to put our image
	// Free this afterwards or we will have a memory leak!
	int* output = (int*)malloc(((cols)*(rows)+2)*sizeof(int));
	*(output) = cols;
	*(output+1) = rows;
	// Iterate through each row/column and get the relevant pixel, perform the conv
	int patch[9];
	int offset[3];
	int* kernel = direction == 0 ? kernEdgeDetectH : kernEdgeDetectV;

	int bytesToCopy = sizeof(int)*3;
	int writeIndex = 0;
	for (int row = -1; row < rows-1; row++){
		for (int col = -1; col < cols-1; col++){
			if ((row == -1) || (row == rows-2) || (col == -1) || (col == cols-2)) {
				// Skip This loop for padding
				*(output+2+writeIndex) = 0;
				writeIndex++;
				continue;
			}
			offset[0] = col+row*cols; 		// Calculate pixel offset row1
			offset[1] = col+(row+1)*cols;	// Calculate pixel offset row2
			offset[2] = col+(row+2)*cols;	// Calculate pixel offset row3
			memcpy(patch,imageArray+offset[0],bytesToCopy); // Copy first row to patch
			memcpy(patch+3,imageArray+offset[1],bytesToCopy); // Copy second row to patch
			memcpy(patch+6,imageArray+offset[2],bytesToCopy); // Copy second row to patch
			*(output+2+writeIndex) = conv(patch,kernel,3,1); // Calculate convolution
			writeIndex++;
		}
	}
	// Return the memory location with the downscaled image
	return output;
}

int* processEdgeDetection(int* conv1, int* conv2, int numPixels) {
	// Allocate memory for reversed image
	int* output = (int*)malloc((numPixels+2)*sizeof(int));
	*output = *conv1;
	*(output+1) = *(conv1+1);
	int maxValue = 0;

	// Perform different operations based on if we supply both edge detections
	if (conv2 != NULL) {
		// Sum and ABS
		for (int pixel = 0; pixel<numPixels; pixel++) {
			*(output+2+pixel) = abs((*(conv1+2+pixel))+(*(conv2+2+pixel)));
			if (*(output+2+pixel) > maxValue) maxValue = *(output+pixel);
		}
	} else {
		// ABS
		for (int pixel = 0; pixel<numPixels; pixel++) {
			*(output+2+pixel) = abs(*(conv1+2+pixel));
			if (*(output+2+pixel) > maxValue) maxValue = *(output+2+pixel);
		}
	}

	// Perform Threshold
	maxValue = maxValue/2;
	if (maxValue > 15) maxValue = 15;
	for (int pixel = 0; pixel<numPixels; pixel++) {
		if (*(output+2+pixel) > maxValue){
			*(output+2+pixel) = maxValue;
		}
	}

	// Free memory holding any convolutions as they are no longer required
	free(conv1);
	if (conv2 != NULL) {
		free(conv2);
	}
	// Return the memory location with the processed image
	return output;
}

int* fastEdgeDetection(int* imageArray, int cols, int rows) {
	// Allocate memory for reversed image
	int* output = (int*)malloc(((cols)*(rows)+2)*sizeof(int));
	*(output) = cols;
	*(output+1) = rows;

	// Iterate through each row/column and get the relevant pixel, perform the fast conv
	int offset[3];
	int pixelValueH;
	int pixelValueV;
	int pixelValue;

	// Go through all pixels
	int writeIndex = 0;
	for (int row = -1; row < rows-1; row++){
		for (int col = -1; col < cols-1; col++){
			if ((row == -1) || (row == rows-2) || (col == -1) || (col == cols-2)) {
				// Skip This loop for padding
				*(output+2+writeIndex) = 0;
				writeIndex++;
				continue;
			}
			offset[0] = col+row*cols; 		// Calculate pixel offset row1
			offset[1] = col+(row+1)*cols;	// Calculate pixel offset row2
			offset[2] = col+(row+2)*cols;	// Calculate pixel offset row3

			// Fast convolution using fixed kernels
			// Calculate Vertical Edges
			pixelValueV = (*(imageArray+offset[0])   + *(imageArray+offset[0]+2)- *(imageArray+offset[2]) - *(imageArray+offset[2]+2));
			pixelValueV += (*(imageArray+offset[0]+1) - *(imageArray+offset[2]+1)) << 1;
			// Force to be positive
			pixelValueV = abs(pixelValueV);

			// Calculate Horizontal Edges
			pixelValueH = (*(imageArray+offset[0])   + *(imageArray+offset[2])  - *(imageArray+offset[0]+2) - *(imageArray+offset[2]+2));
			pixelValueH += (*(imageArray+offset[1])   - *(imageArray+offset[1]+2)) << 1;
			// Force to be positive
			pixelValue = abs(pixelValueH)+pixelValueV;

			// Perform Thresholding
			if (pixelValue > 15) {
				*(output+2+writeIndex) = 15; // Set as edge
			} else if (pixelValue < 7) {
				*(output+2+writeIndex) = 0; // Set as edge
			} else {
				*(output+2+writeIndex) = pixelValue; // Not an edge
			}
			writeIndex++;
		}
	}

	// Return the memory location with the Edge Detected image
	return output;
}

int* hardwareEdgeDetection(int* imageArray, int cols, int rows) {
	// Allocate memory for reversed image
	int* output = (int*)malloc(((cols)*(rows)+2)*sizeof(int));
	*(output) = cols;
	*(output+1) = rows;

	// Iterate through each row/column and get the relevant pixel, perform the fast conv
	int offset[3];
	int pixelValue;
	int pixelValue2;
	IOWR(PIXEL5_INPUT_BASE,0,0);
	// Go through all pixels
	int writeIndex = 0;
	for (int row = -1; row < rows-1; row++){
		for (int col = -1; col < cols-1; col++){
			if ((row == -1) || (row == rows-2) || (col == -1) || (col == cols-2)) {
				// Skip This loop for padding
				*(output+2+writeIndex) = 0;
				writeIndex++;
				continue;
			}
			offset[0] = col+row*cols; 		// Calculate pixel offset row1
			offset[1] = col+(row+1)*cols;	// Calculate pixel offset row2
			offset[2] = col+(row+2)*cols;	// Calculate pixel offset row3
			// Calculate Vertical Edges
			//printf("%d\n",sizeof(*(imageArray+offset[0])));
			//printf("%d\n",*(imageArray+offset[0])&0xFF);
			IOWR(PIXEL1_INPUT_BASE,0,*(imageArray+offset[0]));
			IOWR(PIXEL2_INPUT_BASE,0,*(imageArray+offset[0]+1));
			IOWR(PIXEL3_INPUT_BASE,0,*(imageArray+offset[0]+2));
			IOWR(PIXEL4_INPUT_BASE,0,*(imageArray+offset[1]));
			IOWR(PIXEL6_INPUT_BASE,0,*(imageArray+offset[1]+2));
			IOWR(PIXEL7_INPUT_BASE,0,*(imageArray+offset[2]));
			IOWR(PIXEL8_INPUT_BASE,0,*(imageArray+offset[2]+1));
			IOWR(PIXEL9_INPUT_BASE,0,*(imageArray+offset[2]+2));
			//printf("CONVX %d ",IORD(CONVX_OUT_BASE,0));

			// Force to be positive

			//In Verilog signed ints are in twos compliment so we convert back

			pixelValue = IORD(CONVX_OUT_BASE,0);
			if ((pixelValue & 0x80) != 0){
				pixelValue = (pixelValue^0xFF) - 1;
			}
			pixelValue = abs(pixelValue);
			pixelValue2 = IORD(CONVY_OUT_BASE,0);
			if ((pixelValue2 & 0x80) != 0){
				pixelValue2 = (pixelValue2^0xFF) - 1;
			}
			// Force to be positive
			pixelValue += abs(pixelValue2);
			
			// Perform Thresholding
			if (pixelValue > 15) {
				*(output+2+writeIndex) = 15; // Set as edge
			} else if (pixelValue < 7) {
				*(output+2+writeIndex) = 0; // Set as edge
			} else {
				*(output+2+writeIndex) = pixelValue; // Not an edge
			}
			writeIndex++;
		}
	}

	// Return the memory location with the Edge Detected image
	return output;
}

int* blurImgConv(int* imageArray, int cols, int rows) {
	// This requests a new block of memory to put our image
	// Free this afterwards or we will have a memory leak!
	int* output = (int*)malloc(((cols)*(rows)+2)*sizeof(int));
	*(output) = cols;
	*(output+1) = rows;
	// Iterate through each row/column and get the relevant pixel, perform the conv
	int patch[9];
	int offset[3];

	int bytesToCopy = sizeof(int)*3;
	int writeIndex = 0;
	for (int row = -1; row < rows-1; row++){
		for (int col = -1; col < cols-1; col++){
			if ((row == -1) || (row == rows-2) || (col == -1) || (col == cols-2)) {
				// Skip This loop for padding
				*(output+2+writeIndex) = 0;
				writeIndex++;
				continue;
			}
			offset[0] = col+row*cols; 		// Calculate pixel offset row1
			offset[1] = col+(row+1)*cols;	// Calculate pixel offset row2
			offset[2] = col+(row+2)*cols;	// Calculate pixel offset row3
			memcpy(patch,imageArray+offset[0],bytesToCopy); // Copy first row to patch
			memcpy(patch+3,imageArray+offset[1],bytesToCopy); // Copy second row to patch
			memcpy(patch+6,imageArray+offset[2],bytesToCopy); // Copy second row to patch
			*(output+2+writeIndex) = conv(patch,kernBlur3x,3,9); // Calculate convolution
			writeIndex++;
		}
	}
	// Return the memory location with the blurred image
	return output;
}

int* fastBlurImg(int* imageArray, int cols, int rows) {
	// This requests a new block of memory to put our image
	// Free this afterwards or we will have a memory leak!
	int* output = (int*)malloc(((cols)*(rows)+2)*sizeof(int));
	*(output) = cols;
	*(output+1) = rows;
	// Iterate through each row/column and get the relevant pixel, perform the conv
	int offset[3];
	int res;

	int writeIndex = 0;
	for (int row = -1; row < rows-1; row++){
		for (int col = -1; col < cols-1; col++){
			if ((row == -1) || (row == rows-2) || (col == -1) || (col == cols-2)) {
				// Skip This loop for padding
				*(output+2+writeIndex) = 0;
				writeIndex++;
				continue;
			}
			offset[0] = col+row*cols; 		// Calculate pixel offset row1
			offset[1] = col+(row+1)*cols;	// Calculate pixel offset row2
			offset[2] = col+(row+2)*cols;	// Calculate pixel offset row3
			res  = *(imageArray+offset[0]) + *(imageArray+offset[0]+1) + *(imageArray+offset[0]+2);
			res += *(imageArray+offset[1]) + *(imageArray+offset[1]+1) + *(imageArray+offset[1]+2);
			res += *(imageArray+offset[2]) + *(imageArray+offset[2]+1) + *(imageArray+offset[2]+2);
			IOWR_8DIRECT(DIV9_TOHW_BASE,0,res);
			*(output+2+writeIndex) = IORD_8DIRECT(DIV9_TOSW_BASE,0); // Calculate convolution
			writeIndex++;
		}
	}
	// Return the memory location with the blurred image
	return output;
}

int* reverseImg(int* imageArray, int numPixels) {
	// Allocate memory for reversed image
	int* output = (int*)malloc(numPixels*sizeof(int));
	int reversePixelIdx = numPixels;

	// Store Reversed Image
	for (int pixel = 0; pixel<numPixels; pixel++) {
		reversePixelIdx--;
		*(output+pixel)=*(imageArray+reversePixelIdx);
	}

	// Return the memory location with the reversed image
	return output;
}

int* imgToPtr(int baseAddr, int pixelNum) {
	// Allocate pointer to hold image
	int* img = (int*) malloc(pixelNum*sizeof(int));
	// Extract image to allocated pointer
	for (int pixel = 0; pixel<pixelNum; pixel++) {
		*(img+pixel) = IORD_32DIRECT(baseAddr,pixel*4)>>24;
	}
	// Return image pointer
	return img;
}

void imgToSDRAM(int* imgPtr, int baseAddr, int pixelNum, int padding) {
	// Save image to SDRAM pointer
	for (int pixel = 0; pixel<pixelNum; pixel++) {
		IOWR_32DIRECT(baseAddr,pixel*4,*(imgPtr+pixel+padding)<<24);
	}
}

/* ------------------------- *
 *     END MS2 FUNCTIONS 	 *
 * ------------------------- */

/* ------------------------- *
 *    BEGIN SYSTEM TASKS 	 *
 * ------------------------- */
void imageOneTask(void* pdata) {
	//Fetch image from SDRAM, downscale then display
	// SW0 SW1 for image one
	int switchVal;
	int* img;
	int* imgDS;

	INT8U err;
	while (1) {
		OSSemPend(semDisplay1,0,&err);
		#if ENABLE_DEBUG_OUTPUT
		printf("I1T GO\n");
		#endif

		switchVal = IORD(SW_IN_BASE, 0);
		switchVal = switchVal & 0b11;
		switch (switchVal) {
		//Base image
		case 0b00:
			//downscale
			img = imgToPtr(SDRAM_BASEADDR, BUF_MAX_PIX);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 0);
			imageToBuffer(imgDS,0,0);
			break;
		//Flip
		case 0b01:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockFlipImgPointer,0,&err);
			img = imgToPtr(FLIPIMG_BASE, BUF_MAX_PIX);
			OSSemPost(semLockFlipImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 0);
			imageToBuffer(imgDS,0,0);
			break;
		//Blur
		case 0b10:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockBlurImgPointer,0,&err);
			img = imgToPtr(BLURING_BASE, BUF_MAX_PIX);
			OSSemPost(semLockBlurImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 2);
			imageToBuffer(imgDS,0,0);
			break;
		//Edge
		case 0b11:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockEdgeImgPointer,0,&err);
			img = imgToPtr(EDGEIMG_BASE, BUF_MAX_PIX);
			OSSemPost(semLockEdgeImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 2);
			imageToBuffer(imgDS,0,0);
			break;
		}
		free(img);
		free(imgDS);

		OSSemPost(semDisplay2);
		OSTimeDlyHMSM(0, 0, 0, 250);
	}
}

void imageTwoTask(void* pdata) {
	// SW2 SW3 for image two
	int switchVal;
	int* img;
	int* imgDS;

	INT8U err;
	while (1) {
		OSSemPend(semDisplay2,0,&err);
		#if ENABLE_DEBUG_OUTPUT
		printf("I2T GO\n");
		#endif
		switchVal = IORD(SW_IN_BASE, 0);
		switchVal = (switchVal & 0b1100) >> 2;
		switch (switchVal) {
		//Base image
		case 0b00:
			//downscale
			img = imgToPtr(SDRAM_BASEADDR, BUF_MAX_PIX);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 0);
			imageToBuffer(imgDS,IMG_WIDTH/2,0);
			break;
			//Flip
		case 0b01:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockFlipImgPointer,0,&err);
			img = imgToPtr(FLIPIMG_BASE, BUF_MAX_PIX);
			OSSemPost(semLockFlipImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 0);
			imageToBuffer(imgDS,IMG_WIDTH/2,0);
			break;
			//Blur
		case 0b10:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockBlurImgPointer,0,&err);
			img = imgToPtr(BLURING_BASE, BUF_MAX_PIX);
			OSSemPost(semLockBlurImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 2);
			imageToBuffer(imgDS,IMG_WIDTH/2,0);
			break;
			//Edge
		case 0b11:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockEdgeImgPointer,0,&err);
			img = imgToPtr(EDGEIMG_BASE, BUF_MAX_PIX);
			OSSemPost(semLockEdgeImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 2);
			imageToBuffer(imgDS,IMG_WIDTH/2,0);
			break;
		}
		free(img);
		free(imgDS);
		OSSemPost(semDisplay3);
		OSTimeDlyHMSM(0, 0, 0, 250);
	}
}

void imageThreeTask(void* pdata) {
	// SW4 SW5 for image three
	int switchVal;
	int* img;
	int* imgDS;
	INT8U err;
	while (1) {
		OSSemPend(semDisplay3,0,&err);
		#if ENABLE_DEBUG_OUTPUT
		printf("I3T GO\n");
		#endif
		switchVal = IORD(SW_IN_BASE, 0);
		switchVal = (switchVal & 0b110000) >> 4;
		switch (switchVal) {
		//Base image
		case 0b00:
			//downscale
			img = imgToPtr(SDRAM_BASEADDR, BUF_MAX_PIX);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 0);
			imageToBuffer(imgDS,0,IMG_HEIGHT/2*IMG_WIDTH);
			break;
		//Flip
		case 0b01:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockFlipImgPointer,0,&err);
			img = imgToPtr(FLIPIMG_BASE, BUF_MAX_PIX);
			OSSemPost(semLockFlipImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 0);
			imageToBuffer(imgDS,0,IMG_HEIGHT/2*IMG_WIDTH);
			break;
		//Blur
		case 0b10:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockBlurImgPointer,0,&err);
			img = imgToPtr(BLURING_BASE, BUF_MAX_PIX);
			OSSemPost(semLockBlurImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 2);
			imageToBuffer(imgDS,0,IMG_HEIGHT/2*IMG_WIDTH);
			break;
		//Edge
		case 0b11:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockEdgeImgPointer,0,&err);
			img = imgToPtr(EDGEIMG_BASE, BUF_MAX_PIX);
			OSSemPost(semLockEdgeImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 2);
			imageToBuffer(imgDS,0,IMG_HEIGHT/2*IMG_WIDTH);
			break;
		}
		free(img);
		free(imgDS);
		OSSemPost(semDisplay4);
		OSTimeDlyHMSM(0, 0, 0, 250);
	}
}

void imageFourTask(void* pdata) {
	// SW6 SW7 for image four
	int switchVal;
	int* img;
	int* imgDS;
	INT8U err;
	while (1) {
		OSSemPend(semDisplay4,0,&err);
		#if ENABLE_DEBUG_OUTPUT
		printf("I4T GO\n");
		#endif
		switchVal = IORD(SW_IN_BASE, 0);
		switchVal = (switchVal & 0b11000000) >> 6;
		switch (switchVal) {
		//Base image
		case 0b00:
			//downscale
			img = imgToPtr(SDRAM_BASEADDR, BUF_MAX_PIX);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 0);
			imageToBuffer(imgDS,IMG_WIDTH/2,IMG_HEIGHT/2*IMG_WIDTH);
			break;
		//Flip
		case 0b01:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockFlipImgPointer,0,&err);
			img = imgToPtr(FLIPIMG_BASE, BUF_MAX_PIX);
			OSSemPost(semLockFlipImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 0);
			imageToBuffer(imgDS,IMG_WIDTH/2,IMG_HEIGHT/2*IMG_WIDTH);
			break;
		//Blur
		case 0b10:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockBlurImgPointer,0,&err);
			img = imgToPtr(BLURING_BASE, BUF_MAX_PIX);
			OSSemPost(semLockBlurImgPointer);
      imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 2);
			imageToBuffer(imgDS,IMG_WIDTH/2,IMG_HEIGHT/2*IMG_WIDTH);
			break;
		//Edge
		case 0b11:
			// Hold Access to image while we retrieve data. Otherwise we will get tearing (if this was video data)
			OSSemPend(semLockEdgeImgPointer,0,&err);
			img = imgToPtr(EDGEIMG_BASE, BUF_MAX_PIX);
			OSSemPost(semLockEdgeImgPointer);
			imgDS = downscaleImg2x(img, IMG_WIDTH, IMG_HEIGHT, 2);
			imageToBuffer(imgDS,IMG_WIDTH/2,IMG_HEIGHT/2*IMG_WIDTH);
			break;
		}
		free(img);
		free(imgDS);
		OSSemPost(semDisplay1);
		OSTimeDlyHMSM(0, 0, 0, 250);
	}
}

void singleImageTask (void* pdata) {
	INT8U semPendErr;
	int switchVal;
	while (1) {
		// Wait for single img mode flag
		OSSemPend(semSingleImag,0,&semPendErr);

		// Go Time!
		switchVal = IORD(SW_IN_BASE, 0);
		switchVal = switchVal & 0b11;
		//printf("%d\n",switchVal);
		switch(switchVal){
		case 0b00:
			imageToBufferSDRAM(SDRAM_BASEADDR);
			break;
				//Flip
		case 0b01:
			imageToBufferSDRAM(FLIPIMG_BASE);
			break;
				//Blur
		case 0b10:
			imageToBufferSDRAM(BLURING_BASE);
			break;
				//Edge
		case 0b11:
			imageToBufferSDRAM(EDGEIMG_BASE);
			break;
		}
		OSSemPost(semSingleImag);
		OSTimeDlyHMSM(0, 0, 0, 200);
	}
}



void buttonManagerTask(void* pdata){
	INT8U semPendErr;
	while(1){
		OSSemPend(semKeyChange,0,&semPendErr);
		printf("BMT: Context %d\n",keyPressedContext);
		if (keyPressedContext & 0x1) {
			singleImgMode = (keyPressedContext & 0x1) ^ singleImgMode;
			// Check if we want SIM
			if (singleImgMode) {
				printf("BMT: SIM1\n");
				// Lock out the 4 displays
				OSSemPend(semDisplay1,0,&semPendErr);
				printf("BMT: SIM2\n");
				// Allow SIM task
				OSSemPost(semSingleImag);
				printf("BMT: SIM3\n");
			} else {
				printf("BMT: QIM1\n");
				// Lock out SIM task
				OSSemPend(semSingleImag,0,&semPendErr);
				printf("BMT: QIM2\n");
				// Allow the 4 displays
				OSSemPost(semDisplay1);
				printf("BMT: QIM3\n");
			}
		}

	}
}

void switchManagerTask (void* pdata) {
	INT8U semPendErr;
	while (1) {
		// Wait for interrupt
		OSSemPend(semSwitchChange,0,&semPendErr);

		if ((SwChangedContext >> 8) != 0) {
			currentlyDislayed = IORD_16DIRECT(SW_IN_BASE,0) >> 8;
		}
	}

}

/* Processes Images */
void imageProcessorTask(void* pdata)
{
	// Create Sem Error Variable
	INT8U semPendErr;

	// Define Img Effect Pointers
	int* img;
	int* imgFlipped = NULL;
	int* imgBlurred = NULL;
	int* imgEdgeDetect = NULL;

	// Preload image (this will be replaced when
	// we implement image switching with the switches)
	OSSemPend(semLockBaseImgPointer,0,&semPendErr);
	img = imgToPtr(SDRAM_BASEADDR,BUF_MAX_PIX);
	OSSemPost(semLockBaseImgPointer);

	#if VERSION == 1
	int* edgeResH = NULL;
	int* edgeResV = NULL;
	#endif

	INT32U timeTemp = 0;
	while (1)
	{
		#if ENABLE_DEBUG_OUTPUT
		printf("ImgProcess GO!\n");
		#endif
		// Lock Pointers
		OSSemPend(semLockFlipImgPointer,0,&semPendErr);
		OSSemPend(semLockBlurImgPointer,0,&semPendErr);
		OSSemPend(semLockEdgeImgPointer,0,&semPendErr);

		// Free Pointers to prevent Memory Leak
		free(imgFlipped);
		free(imgBlurred);
		free(imgEdgeDetect);
		//  -- Process Image --
		// Flip Image
		timeTemp = OSTimeGet();
		imgFlipped = reverseImg(img, BUF_MAX_PIX);
		flipTime = OSTimeGet()-timeTemp;

		// Blur image
		timeTemp = OSTimeGet();
		#if VERSION == 2
		imgBlurred = fastBlurImg(img, IMG_WIDTH, IMG_HEIGHT);
		#else
		imgBlurred = blurImgConv(img, IMG_WIDTH, IMG_HEIGHT);
		#endif
		blurTime = OSTimeGet()-timeTemp;

		// Perform Edge detection
		timeTemp = OSTimeGet();
		#if VERSION == 2
		imgEdgeDetect = hardwareEdgeDetection(img, IMG_WIDTH, IMG_HEIGHT);
		#else
		edgeResH = edgeDetectionConv(img, IMG_WIDTH, IMG_HEIGHT, 0);
		edgeResV = edgeDetectionConv(img, IMG_WIDTH, IMG_HEIGHT, 1);
		imgEdgeDetect = processEdgeDetection(edgeResH, edgeResV, (*edgeResH)*(*(edgeResH+1)));
		#endif
		edgeTime = OSTimeGet()-timeTemp;

		totalTime = flipTime+edgeTime+blurTime;
		// Save to SDRAM
		imgToSDRAM(imgFlipped,    FLIPIMG_BASE, BUF_MAX_PIX, 0);
		imgToSDRAM(imgEdgeDetect, EDGEIMG_BASE, BUF_MAX_PIX, 2);
		imgToSDRAM(imgBlurred,    BLURING_BASE, BUF_MAX_PIX, 2);

		// Release img pointers so they can be displayed
		OSSemPost(semLockFlipImgPointer);
		OSSemPost(semLockBlurImgPointer);
		OSSemPost(semLockEdgeImgPointer);

		// Delay for 500ms
		OSTimeDlyHMSM(0, 0, 0, 500);
	}
}


void mainTask(void* pdata)
{
	// Compute CPU capacity with no task running
	INT8U err;
	OSStatInit();

	//Create mutex
	mutexKey = OSMutexCreate(MUTEXSINGIMG_PRIORITY,&err);

	// Create Semaphores and IRQHs
	semLockFlipImgPointer = OSSemCreate(1);
	semLockBlurImgPointer = OSSemCreate(1);
	semLockEdgeImgPointer = OSSemCreate(1);
	semLockBaseImgPointer = OSSemCreate(1);

	semSwitchChange = OSSemCreate(1);
	semKeyChange = OSSemCreate(1);
	OSSemPend(semSwitchChange,0,&err);
	OSSemPend(semKeyChange,0,&err);

	semSingleImag = OSSemCreate(1);

	// Semaphores to block the image display tasks
	semDisplay1 = OSSemCreate(1);
	semDisplay2 = OSSemCreate(1);
	semDisplay3 = OSSemCreate(1);
	semDisplay4 = OSSemCreate(1);

	// Do this to cause the image tasks
	// to run in a Round-Robin style
	// Otherwise Disp4 takes forever
	OSSemPend(semDisplay2,0,&err);
	OSSemPend(semDisplay3,0,&err);
	OSSemPend(semDisplay4,0,&err);

	// Set default to normal mode
	singleImgMode = 0;
	OSSemPend(semSingleImag,0,&err);

	initButtonIRQ();
	initSwIRQ();

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

	printf("Attempting to create imageProcessorTask\n");
	error_code = OSTaskCreateExt(imageProcessorTask,
				  NULL,
				  (void *)&imageProcessorTask_stk[TASK_STACKSIZE-1],
				  IMAGEPROCESSORTASK_PRIORITY,
				  IMAGEPROCESSORTASK_PRIORITY,
				  imageProcessorTask_stk,
				  IMAGEPROCESSORTASK_PRIORITY,
				  NULL,
				  0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating imageProcessorTask with error code %d\n", error_code); else printf("Created task successfully\n");
	//First Quadrant image
	printf("Attempting to create imageOneTask\n");
	error_code = OSTaskCreateExt(imageOneTask,
		NULL,
		(void*)&imageOneTask_stk[TASK_STACKSIZE - 1],
		IMAGEONETASK_PRIORITY,
		IMAGEONETASK_PRIORITY,
		imageOneTask_stk,
		IMAGEONETASK_PRIORITY,
		NULL,
		0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating imageOneTask with error code %d\n", error_code); else printf("Created task successfully\n");

	//Second quadrant image
	printf("Attempting to create imageTwoTask\n");
	error_code = OSTaskCreateExt(imageTwoTask,
		NULL,
		(void*)&imageTwoTask_stk[TASK_STACKSIZE - 1],
		IMAGETWOTASK_PRIORITY,
		IMAGETWOTASK_PRIORITY,
		imageTwoTask_stk,
		IMAGETWOTASK_PRIORITY,
		NULL,
		0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating imageOneTask with error code %d\n", error_code); else printf("Created task successfully\n");

	//Third quadrant image
	printf("Attempting to create imageThreeTask\n");
	error_code = OSTaskCreateExt(imageThreeTask,
		NULL,
		(void*)&imageThreeTask_stk[TASK_STACKSIZE - 1],
		IMAGETHREETASK_PRIORITY,
		IMAGETHREETASK_PRIORITY,
		imageThreeTask_stk,
		IMAGETHREETASK_PRIORITY,
		NULL,
		0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating imageOneTask with error code %d\n", error_code); else printf("Created task successfully\n");

	//Fourth quadrant image
	printf("Attempting to create imageFourTask\n");
	error_code = OSTaskCreateExt(imageFourTask,
		NULL,
		(void*)&imageFourTask_stk[TASK_STACKSIZE - 1],
		IMAGEFOURTASK_PRIORITY,
		IMAGEFOURTASK_PRIORITY,
		imageFourTask_stk,
		IMAGEFOURTASK_PRIORITY,
		NULL,
		0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating imageOneTask with error code %d\n", error_code); else printf("Created task successfully\n");

	//Switch manager
	printf("Attempting to create SwitchTask\n");
	error_code = OSTaskCreateExt(switchManagerTask,
		NULL,
		(void*)&switchManagerTask_stk[TASK_STACKSIZE - 1],
		SWITCHMANAGERTASK_PRIORITY,
		SWITCHMANAGERTASK_PRIORITY,
		switchManagerTask_stk,
		SWITCHMANAGERTASK_PRIORITY,
		NULL,
		0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating imageOneTask with error code %d\n", error_code); else printf("Created task successfully\n");

	printf("Attempting to create singleImageTask_stk\n");
			error_code = OSTaskCreateExt(singleImageTask,
			NULL,
			(void*)&singleImageTask_stk[TASK_STACKSIZE - 1],
			SINGLEIMAGETASK_PRIORITY,
			SINGLEIMAGETASK_PRIORITY,
			singleImageTask_stk,
			SINGLEIMAGETASK_PRIORITY,
			NULL,
			0);
	printf(" --> ");
	if (error_code != 0) printf("Error creating SingleImageTask with error code %d\n", error_code); else printf("Created task successfully\n");
	/* ------------
		INDEF LOOP
	   ------------ */

	/*
	// Reverse Image example usage
	//int* res = reverseImg(img, BUF_MAX_PIX);
	//imgToSDRAM(res, 0x12c00, BUF_MAX_PIX, 0);
	// Downscaling example usage


	int* edgeResH = edgeDetectionConv(img, IMG_WIDTH, IMG_HEIGHT, 0);
	int* edgeResV = edgeDetectionConv(img, IMG_WIDTH, IMG_HEIGHT, 1);
	int* res = processEdgeDetection(edgeResH, edgeResV, (*edgeResH)*(*(edgeResH+1)));
	//int* res = blurImgConv(img, IMG_WIDTH, IMG_HEIGHT);
	printf("Test Process: %d,%d -- %d,%d\n",*res,*(res+1),*(res+2),*(res+3));
	imgToSDRAM(res, 0x12c00, (*res)*(*(res+1)), 2);
	free(res);
	free(img);
	*/
	INT32U dispTime;
	while (1)
	{
		displayDigit (HEXDISPLAYS5TO3_BASE, 2, ((OSCPUUsage/10)%10));
		displayDigit (HEXDISPLAYS5TO3_BASE, 1, OSCPUUsage%10);
		if (currentlyDislayed == 0){
			dispTime = flipTime;
		}
		else if (currentlyDislayed == 1){
			dispTime = blurTime;
		}
		else if (currentlyDislayed == 2){
			dispTime = edgeTime;
		}
		else if (currentlyDislayed == 3){
			dispTime = totalTime;
		}
		displayDigit (HEXDISPLAYS5TO3_BASE, 0, (dispTime/1000)%10);
		displayDigit (HEXDISPLAYS2TO0_BASE, 2, (dispTime/100)%10);
		displayDigit (HEXDISPLAYS2TO0_BASE, 1, (dispTime/10)%10);
		displayDigit (HEXDISPLAYS2TO0_BASE, 0, dispTime%10);

		// Delay 1s
		OSTimeDlyHMSM(0, 0, 0, 200);
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

	// Clear Hex displays
	IOWR(HEXDISPLAYS2TO0_BASE, 0, 0xffffff);
	IOWR(HEXDISPLAYS5TO3_BASE, 0, 0xffffff);


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
