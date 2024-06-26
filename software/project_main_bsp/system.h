/*
 * system.h - SOPC Builder system and BSP software package information
 *
 * Machine generated for CPU 'NIOS_PROC' in SOPC Builder design 'niosII_processor'
 * SOPC Builder design path: ../../niosII_processor.sopcinfo
 *
 * Generated: Sat Apr 06 20:07:42 EST 2024
 */

/*
 * DO NOT MODIFY THIS FILE
 *
 * Changing this file will have subtle consequences
 * which will almost certainly lead to a nonfunctioning
 * system. If you do modify this file, be aware that your
 * changes will be overwritten and lost when this file
 * is generated again.
 *
 * DO NOT MODIFY THIS FILE
 */

/*
 * License Agreement
 *
 * Copyright (c) 2008
 * Altera Corporation, San Jose, California, USA.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This agreement shall be governed in all respects by the laws of the State
 * of California and by the laws of the United States of America.
 */

#ifndef __SYSTEM_H_
#define __SYSTEM_H_

/* Include definitions from linker script generator */
#include "linker.h"


/*
 * CPU configuration
 *
 */

#define ALT_CPU_ARCHITECTURE "altera_nios2_gen2"
#define ALT_CPU_BIG_ENDIAN 0
#define ALT_CPU_BREAK_ADDR 0x04080820
#define ALT_CPU_CPU_ARCH_NIOS2_R1
#define ALT_CPU_CPU_FREQ 50000000u
#define ALT_CPU_CPU_ID_SIZE 1
#define ALT_CPU_CPU_ID_VALUE 0x00000000
#define ALT_CPU_CPU_IMPLEMENTATION "fast"
#define ALT_CPU_DATA_ADDR_WIDTH 0x1b
#define ALT_CPU_DCACHE_BYPASS_MASK 0x80000000
#define ALT_CPU_DCACHE_LINE_SIZE 32
#define ALT_CPU_DCACHE_LINE_SIZE_LOG2 5
#define ALT_CPU_DCACHE_SIZE 2048
#define ALT_CPU_EXCEPTION_ADDR 0x04040020
#define ALT_CPU_FLASH_ACCELERATOR_LINES 0
#define ALT_CPU_FLASH_ACCELERATOR_LINE_SIZE 0
#define ALT_CPU_FLUSHDA_SUPPORTED
#define ALT_CPU_FREQ 50000000
#define ALT_CPU_HARDWARE_DIVIDE_PRESENT 0
#define ALT_CPU_HARDWARE_MULTIPLY_PRESENT 1
#define ALT_CPU_HARDWARE_MULX_PRESENT 0
#define ALT_CPU_HAS_DEBUG_CORE 1
#define ALT_CPU_HAS_DEBUG_STUB
#define ALT_CPU_HAS_EXTRA_EXCEPTION_INFO
#define ALT_CPU_HAS_ILLEGAL_INSTRUCTION_EXCEPTION
#define ALT_CPU_HAS_JMPI_INSTRUCTION
#define ALT_CPU_ICACHE_LINE_SIZE 32
#define ALT_CPU_ICACHE_LINE_SIZE_LOG2 5
#define ALT_CPU_ICACHE_SIZE 4096
#define ALT_CPU_INITDA_SUPPORTED
#define ALT_CPU_INST_ADDR_WIDTH 0x1b
#define ALT_CPU_NAME "NIOS_PROC"
#define ALT_CPU_NUM_OF_SHADOW_REG_SETS 0
#define ALT_CPU_OCI_VERSION 1
#define ALT_CPU_RESET_ADDR 0x04040000


/*
 * CPU configuration (with legacy prefix - don't use these anymore)
 *
 */

#define NIOS2_BIG_ENDIAN 0
#define NIOS2_BREAK_ADDR 0x04080820
#define NIOS2_CPU_ARCH_NIOS2_R1
#define NIOS2_CPU_FREQ 50000000u
#define NIOS2_CPU_ID_SIZE 1
#define NIOS2_CPU_ID_VALUE 0x00000000
#define NIOS2_CPU_IMPLEMENTATION "fast"
#define NIOS2_DATA_ADDR_WIDTH 0x1b
#define NIOS2_DCACHE_BYPASS_MASK 0x80000000
#define NIOS2_DCACHE_LINE_SIZE 32
#define NIOS2_DCACHE_LINE_SIZE_LOG2 5
#define NIOS2_DCACHE_SIZE 2048
#define NIOS2_EXCEPTION_ADDR 0x04040020
#define NIOS2_FLASH_ACCELERATOR_LINES 0
#define NIOS2_FLASH_ACCELERATOR_LINE_SIZE 0
#define NIOS2_FLUSHDA_SUPPORTED
#define NIOS2_HARDWARE_DIVIDE_PRESENT 0
#define NIOS2_HARDWARE_MULTIPLY_PRESENT 1
#define NIOS2_HARDWARE_MULX_PRESENT 0
#define NIOS2_HAS_DEBUG_CORE 1
#define NIOS2_HAS_DEBUG_STUB
#define NIOS2_HAS_EXTRA_EXCEPTION_INFO
#define NIOS2_HAS_ILLEGAL_INSTRUCTION_EXCEPTION
#define NIOS2_HAS_JMPI_INSTRUCTION
#define NIOS2_ICACHE_LINE_SIZE 32
#define NIOS2_ICACHE_LINE_SIZE_LOG2 5
#define NIOS2_ICACHE_SIZE 4096
#define NIOS2_INITDA_SUPPORTED
#define NIOS2_INST_ADDR_WIDTH 0x1b
#define NIOS2_NUM_OF_SHADOW_REG_SETS 0
#define NIOS2_OCI_VERSION 1
#define NIOS2_RESET_ADDR 0x04040000


/*
 * Define for each module class mastered by the CPU
 *
 */

#define __ALTERA_AVALON_JTAG_UART
#define __ALTERA_AVALON_NEW_SDRAM_CONTROLLER
#define __ALTERA_AVALON_ONCHIP_MEMORY2
#define __ALTERA_AVALON_PIO
#define __ALTERA_AVALON_TIMER
#define __ALTERA_NIOS2_GEN2


/*
 * KEY_IN configuration
 *
 */

#define ALT_MODULE_CLASS_KEY_IN altera_avalon_pio
#define KEY_IN_BASE 0x4081050
#define KEY_IN_BIT_CLEARING_EDGE_REGISTER 0
#define KEY_IN_BIT_MODIFYING_OUTPUT_REGISTER 0
#define KEY_IN_CAPTURE 1
#define KEY_IN_DATA_WIDTH 2
#define KEY_IN_DO_TEST_BENCH_WIRING 0
#define KEY_IN_DRIVEN_SIM_VALUE 0
#define KEY_IN_EDGE_TYPE "RISING"
#define KEY_IN_FREQ 50000000
#define KEY_IN_HAS_IN 1
#define KEY_IN_HAS_OUT 0
#define KEY_IN_HAS_TRI 0
#define KEY_IN_IRQ 3
#define KEY_IN_IRQ_INTERRUPT_CONTROLLER_ID 0
#define KEY_IN_IRQ_TYPE "EDGE"
#define KEY_IN_NAME "/dev/KEY_IN"
#define KEY_IN_RESET_VALUE 0
#define KEY_IN_SPAN 16
#define KEY_IN_TYPE "altera_avalon_pio"


/*
 * LEDR_OUT configuration
 *
 */

#define ALT_MODULE_CLASS_LEDR_OUT altera_avalon_pio
#define LEDR_OUT_BASE 0x4081070
#define LEDR_OUT_BIT_CLEARING_EDGE_REGISTER 0
#define LEDR_OUT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define LEDR_OUT_CAPTURE 0
#define LEDR_OUT_DATA_WIDTH 10
#define LEDR_OUT_DO_TEST_BENCH_WIRING 0
#define LEDR_OUT_DRIVEN_SIM_VALUE 0
#define LEDR_OUT_EDGE_TYPE "NONE"
#define LEDR_OUT_FREQ 50000000
#define LEDR_OUT_HAS_IN 0
#define LEDR_OUT_HAS_OUT 1
#define LEDR_OUT_HAS_TRI 0
#define LEDR_OUT_IRQ -1
#define LEDR_OUT_IRQ_INTERRUPT_CONTROLLER_ID -1
#define LEDR_OUT_IRQ_TYPE "NONE"
#define LEDR_OUT_NAME "/dev/LEDR_OUT"
#define LEDR_OUT_RESET_VALUE 0
#define LEDR_OUT_SPAN 16
#define LEDR_OUT_TYPE "altera_avalon_pio"


/*
 * NIOS_MEM configuration
 *
 */

#define ALT_MODULE_CLASS_NIOS_MEM altera_avalon_onchip_memory2
#define NIOS_MEM_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define NIOS_MEM_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define NIOS_MEM_BASE 0x4040000
#define NIOS_MEM_CONTENTS_INFO ""
#define NIOS_MEM_DUAL_PORT 0
#define NIOS_MEM_GUI_RAM_BLOCK_TYPE "AUTO"
#define NIOS_MEM_INIT_CONTENTS_FILE "niosII_processor_NIOS_MEM"
#define NIOS_MEM_INIT_MEM_CONTENT 0
#define NIOS_MEM_INSTANCE_ID "NONE"
#define NIOS_MEM_IRQ -1
#define NIOS_MEM_IRQ_INTERRUPT_CONTROLLER_ID -1
#define NIOS_MEM_NAME "/dev/NIOS_MEM"
#define NIOS_MEM_NON_DEFAULT_INIT_FILE_ENABLED 0
#define NIOS_MEM_RAM_BLOCK_TYPE "AUTO"
#define NIOS_MEM_READ_DURING_WRITE_MODE "DONT_CARE"
#define NIOS_MEM_SINGLE_CLOCK_OP 0
#define NIOS_MEM_SIZE_MULTIPLE 1
#define NIOS_MEM_SIZE_VALUE 132000
#define NIOS_MEM_SPAN 132000
#define NIOS_MEM_TYPE "altera_avalon_onchip_memory2"
#define NIOS_MEM_WRITABLE 1


/*
 * PBUFF_WREN configuration
 *
 */

#define ALT_MODULE_CLASS_PBUFF_WREN altera_avalon_pio
#define PBUFF_WREN_BASE 0x4081020
#define PBUFF_WREN_BIT_CLEARING_EDGE_REGISTER 0
#define PBUFF_WREN_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PBUFF_WREN_CAPTURE 0
#define PBUFF_WREN_DATA_WIDTH 1
#define PBUFF_WREN_DO_TEST_BENCH_WIRING 0
#define PBUFF_WREN_DRIVEN_SIM_VALUE 0
#define PBUFF_WREN_EDGE_TYPE "NONE"
#define PBUFF_WREN_FREQ 50000000
#define PBUFF_WREN_HAS_IN 0
#define PBUFF_WREN_HAS_OUT 1
#define PBUFF_WREN_HAS_TRI 0
#define PBUFF_WREN_IRQ -1
#define PBUFF_WREN_IRQ_INTERRUPT_CONTROLLER_ID -1
#define PBUFF_WREN_IRQ_TYPE "NONE"
#define PBUFF_WREN_NAME "/dev/PBUFF_WREN"
#define PBUFF_WREN_RESET_VALUE 0
#define PBUFF_WREN_SPAN 16
#define PBUFF_WREN_TYPE "altera_avalon_pio"


/*
 * PB_ADR configuration
 *
 */

#define ALT_MODULE_CLASS_PB_ADR altera_avalon_pio
#define PB_ADR_BASE 0x4081040
#define PB_ADR_BIT_CLEARING_EDGE_REGISTER 0
#define PB_ADR_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PB_ADR_CAPTURE 0
#define PB_ADR_DATA_WIDTH 15
#define PB_ADR_DO_TEST_BENCH_WIRING 0
#define PB_ADR_DRIVEN_SIM_VALUE 0
#define PB_ADR_EDGE_TYPE "NONE"
#define PB_ADR_FREQ 50000000
#define PB_ADR_HAS_IN 0
#define PB_ADR_HAS_OUT 1
#define PB_ADR_HAS_TRI 0
#define PB_ADR_IRQ -1
#define PB_ADR_IRQ_INTERRUPT_CONTROLLER_ID -1
#define PB_ADR_IRQ_TYPE "NONE"
#define PB_ADR_NAME "/dev/PB_ADR"
#define PB_ADR_RESET_VALUE 0
#define PB_ADR_SPAN 16
#define PB_ADR_TYPE "altera_avalon_pio"


/*
 * PB_DATA configuration
 *
 */

#define ALT_MODULE_CLASS_PB_DATA altera_avalon_pio
#define PB_DATA_BASE 0x4081030
#define PB_DATA_BIT_CLEARING_EDGE_REGISTER 0
#define PB_DATA_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PB_DATA_CAPTURE 0
#define PB_DATA_DATA_WIDTH 4
#define PB_DATA_DO_TEST_BENCH_WIRING 0
#define PB_DATA_DRIVEN_SIM_VALUE 0
#define PB_DATA_EDGE_TYPE "NONE"
#define PB_DATA_FREQ 50000000
#define PB_DATA_HAS_IN 0
#define PB_DATA_HAS_OUT 1
#define PB_DATA_HAS_TRI 0
#define PB_DATA_IRQ -1
#define PB_DATA_IRQ_INTERRUPT_CONTROLLER_ID -1
#define PB_DATA_IRQ_TYPE "NONE"
#define PB_DATA_NAME "/dev/PB_DATA"
#define PB_DATA_RESET_VALUE 0
#define PB_DATA_SPAN 16
#define PB_DATA_TYPE "altera_avalon_pio"


/*
 * SDRAM configuration
 *
 */

#define ALT_MODULE_CLASS_SDRAM altera_avalon_new_sdram_controller
#define SDRAM_BASE 0x0
#define SDRAM_CAS_LATENCY 3
#define SDRAM_CONTENTS_INFO
#define SDRAM_INIT_NOP_DELAY 0.0
#define SDRAM_INIT_REFRESH_COMMANDS 2
#define SDRAM_IRQ -1
#define SDRAM_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SDRAM_IS_INITIALIZED 1
#define SDRAM_NAME "/dev/SDRAM"
#define SDRAM_POWERUP_DELAY 100.0
#define SDRAM_REFRESH_PERIOD 15.625
#define SDRAM_REGISTER_DATA_IN 1
#define SDRAM_SDRAM_ADDR_WIDTH 0x19
#define SDRAM_SDRAM_BANK_WIDTH 2
#define SDRAM_SDRAM_COL_WIDTH 10
#define SDRAM_SDRAM_DATA_WIDTH 16
#define SDRAM_SDRAM_NUM_BANKS 4
#define SDRAM_SDRAM_NUM_CHIPSELECTS 1
#define SDRAM_SDRAM_ROW_WIDTH 13
#define SDRAM_SHARED_DATA 0
#define SDRAM_SIM_MODEL_BASE 0
#define SDRAM_SPAN 67108864
#define SDRAM_STARVATION_INDICATOR 0
#define SDRAM_TRISTATE_BRIDGE_SLAVE ""
#define SDRAM_TYPE "altera_avalon_new_sdram_controller"
#define SDRAM_T_AC 5.5
#define SDRAM_T_MRD 3
#define SDRAM_T_RCD 20.0
#define SDRAM_T_RFC 70.0
#define SDRAM_T_RP 20.0
#define SDRAM_T_WR 14.0


/*
 * SW_IN configuration
 *
 */

#define ALT_MODULE_CLASS_SW_IN altera_avalon_pio
#define SW_IN_BASE 0x4081060
#define SW_IN_BIT_CLEARING_EDGE_REGISTER 0
#define SW_IN_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SW_IN_CAPTURE 1
#define SW_IN_DATA_WIDTH 10
#define SW_IN_DO_TEST_BENCH_WIRING 0
#define SW_IN_DRIVEN_SIM_VALUE 0
#define SW_IN_EDGE_TYPE "RISING"
#define SW_IN_FREQ 50000000
#define SW_IN_HAS_IN 1
#define SW_IN_HAS_OUT 0
#define SW_IN_HAS_TRI 0
#define SW_IN_IRQ 2
#define SW_IN_IRQ_INTERRUPT_CONTROLLER_ID 0
#define SW_IN_IRQ_TYPE "EDGE"
#define SW_IN_NAME "/dev/SW_IN"
#define SW_IN_RESET_VALUE 0
#define SW_IN_SPAN 16
#define SW_IN_TYPE "altera_avalon_pio"


/*
 * System configuration
 *
 */

#define ALT_DEVICE_FAMILY "MAX 10"
#define ALT_ENHANCED_INTERRUPT_API_PRESENT
#define ALT_IRQ_BASE NULL
#define ALT_LOG_PORT "/dev/null"
#define ALT_LOG_PORT_BASE 0x0
#define ALT_LOG_PORT_DEV null
#define ALT_LOG_PORT_TYPE ""
#define ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS 0
#define ALT_NUM_INTERNAL_INTERRUPT_CONTROLLERS 1
#define ALT_NUM_INTERRUPT_CONTROLLERS 1
#define ALT_STDERR "/dev/UART_JTAG"
#define ALT_STDERR_BASE 0x4081080
#define ALT_STDERR_DEV UART_JTAG
#define ALT_STDERR_IS_JTAG_UART
#define ALT_STDERR_PRESENT
#define ALT_STDERR_TYPE "altera_avalon_jtag_uart"
#define ALT_STDIN "/dev/UART_JTAG"
#define ALT_STDIN_BASE 0x4081080
#define ALT_STDIN_DEV UART_JTAG
#define ALT_STDIN_IS_JTAG_UART
#define ALT_STDIN_PRESENT
#define ALT_STDIN_TYPE "altera_avalon_jtag_uart"
#define ALT_STDOUT "/dev/UART_JTAG"
#define ALT_STDOUT_BASE 0x4081080
#define ALT_STDOUT_DEV UART_JTAG
#define ALT_STDOUT_IS_JTAG_UART
#define ALT_STDOUT_PRESENT
#define ALT_STDOUT_TYPE "altera_avalon_jtag_uart"
#define ALT_SYSTEM_NAME "niosII_processor"


/*
 * TIMER configuration
 *
 */

#define ALT_MODULE_CLASS_TIMER altera_avalon_timer
#define TIMER_ALWAYS_RUN 0
#define TIMER_BASE 0x4081000
#define TIMER_COUNTER_SIZE 32
#define TIMER_FIXED_PERIOD 0
#define TIMER_FREQ 50000000
#define TIMER_IRQ 0
#define TIMER_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_LOAD_VALUE 49999
#define TIMER_MULT 0.001
#define TIMER_NAME "/dev/TIMER"
#define TIMER_PERIOD 1
#define TIMER_PERIOD_UNITS "ms"
#define TIMER_RESET_OUTPUT 0
#define TIMER_SNAPSHOT 1
#define TIMER_SPAN 32
#define TIMER_TICKS_PER_SEC 1000
#define TIMER_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_TYPE "altera_avalon_timer"


/*
 * UART_JTAG configuration
 *
 */

#define ALT_MODULE_CLASS_UART_JTAG altera_avalon_jtag_uart
#define UART_JTAG_BASE 0x4081080
#define UART_JTAG_IRQ 1
#define UART_JTAG_IRQ_INTERRUPT_CONTROLLER_ID 0
#define UART_JTAG_NAME "/dev/UART_JTAG"
#define UART_JTAG_READ_DEPTH 64
#define UART_JTAG_READ_THRESHOLD 8
#define UART_JTAG_SPAN 8
#define UART_JTAG_TYPE "altera_avalon_jtag_uart"
#define UART_JTAG_WRITE_DEPTH 64
#define UART_JTAG_WRITE_THRESHOLD 8


/*
 * hal configuration
 *
 */

#define ALT_INCLUDE_INSTRUCTION_RELATED_EXCEPTION_API
#define ALT_MAX_FD 4
#define ALT_SYS_CLK none
#define ALT_TIMESTAMP_CLK none

#endif /* __SYSTEM_H_ */
