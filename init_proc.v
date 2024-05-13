// *********************************
//
//	ECE3073 Project
// Milestone 1 - Task 2 & 3
// Initialises the NIOS processor with required peripherals
// 
// Authors:
//		Hamish McCoy (32474741)
//		Thomas Huang (32501617)
// Last Edited: 1/04/2024
//
//
// *********************************

module init_proc(
	CLOCK_50,
	SW,
	KEY,
	LEDR,
	HEX0,
	HEX1,
	HEX2,
	HEX3,	
	HEX4,	
	HEX5,	
	VGA_R,
	VGA_B,
	VGA_G,
	VGA_HS,
	VGA_VS,
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N
	
);

	// Define Standard IO
	input CLOCK_50;
	input [9:0] SW;
	input [9:0] LEDR;
	input [1:0] KEY;
	output [7:0] HEX0;
	output [7:0] HEX1;
	output [7:0] HEX2;
	output [7:0] HEX3;
	output [7:0] HEX4;
	output [7:0] HEX5;
	
	// Define SDRAM IO
	output [12:0] DRAM_ADDR;
	output [1:0] DRAM_BA;
	output DRAM_CAS_N;
	output DRAM_CKE;
	output DRAM_CLK;
	output DRAM_CS_N;
	inout [15:0] DRAM_DQ;
	output DRAM_LDQM;
	output DRAM_RAS_N;
	output DRAM_UDQM;
	output DRAM_WE_N;
	
	// Define VGA IO
	wire VGA_CLOCK;

	output [3:0] VGA_R;
	output [3:0] VGA_G;
	output [3:0] VGA_B;
	output VGA_HS;
	output VGA_VS;
	wire [18:0] VGA_ADDR;
	wire [3:0] PB_input;
	wire [14:0] PB_WA;
	wire PB_WE;
	wire [3:0] PB_output;
	wire [7:0] div9_value;
	wire [7:0] div9_result;
	
	
	// Define VGA Clock Using PLL
	pll_VGA vga_pll(
		.inclk0(CLOCK_50),
		.c0(VGA_CLOCK)
	);

	// Define VGA Controller
	vga_controller vga_ctl(
		.VGA_DATA(PB_output[3:0]),
		.VGA_CLK(VGA_CLOCK),
		
		.VGA_ADDR(VGA_ADDR[18:0]),
		.VGA_R(VGA_R[3:0]),
		.VGA_G(VGA_G[3:0]),
		.VGA_B(VGA_B[3:0]),
		.VGA_HS(VGA_HS),
		.VGA_VS(VGA_VS)
	);

	// Define Video Ram (pixel buffer)
	ram2port pixel_buffer(
		.clock(VGA_CLOCK),
		.data(PB_input[3:0]),
		.rdaddress(VGA_ADDR[14:0]),
		.wraddress(PB_WA[14:0]),
		.wren(PB_WE),
		.q(PB_output[3:0])
	);
	
	// Instantiate the SDRAM PLL
	pll_SDRAM sdramPll(
		.inclk0(CLOCK_50),
		.c0(DRAM_CLK)
	);
	
	// Instantiate NiosII proc
	niosII_ms2HW nios2_proc(
			.clk_clk(CLOCK_50),
			.reset_reset_n(1'b1),
			.key_export(KEY[1:0]),
			.ledr_export(LEDR[9:0]),
			.pb_adr_export(PB_WA[14:0]),
			.pb_data_export(PB_input[3:0]),
			.sw_export(SW[9:0]),
			.pbuff_wren_export(PB_WE),
			.sdram_wire_addr(DRAM_ADDR[12:0]),
			.sdram_wire_ba(DRAM_BA[1:0]),
			.sdram_wire_cas_n(DRAM_CAS_N),
			.sdram_wire_cke(DRAM_CKE),
			.sdram_wire_cs_n(DRAM_CS_N),
			.sdram_wire_dq(DRAM_DQ[15:0]),
			.sdram_wire_dqm({DRAM_UDQM,DRAM_LDQM}),
			.sdram_wire_ras_n(DRAM_RAS_N),
			.sdram_wire_we_n(DRAM_WE_N),
			.hexdisplays2to0_export({HEX2, HEX1, HEX0}),
		   .hexdisplays5to3_export({HEX5, HEX4, HEX3}),
			.div9_tohw_external_export(div9_value),
			.div9_tosw_external_export(div9_result)
	);
	
	// Instantiate Hardware 3x3 Blur Divide-by-9
	// Perform sum in software since it will be faster
	// Than writing 9 values to 9 PIOs
	divide9 (
		.denom(5'd9),
		.numer(div9_value),
		.quotient(div9_result),
		.remain()
	);

endmodule